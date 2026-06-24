/*
 * UASP (USB Attached SCSI Protocol) device class driver for ESP32-S3.
 *
 * Registers as a custom TinyUSB class driver via usbd_app_driver_get_cb().
 * Storage backend: ESP wear-leveling on SPI flash.
 *
 * Protocol flow (no command queuing — one command at a time):
 *   WAIT_CMD  → xfer_cb(EP_CMD)    → parse IU
 *   DATA_OUT  → xfer_cb(EP_DOUT)   → write sectors, send Sense IU
 *   DATA_IN   → xfer_cb(EP_DIN)    → read sectors, send next chunk or Sense IU
 *   SEND_STS  → xfer_cb(EP_STS)    → resubmit recv on EP_CMD → WAIT_CMD
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "wear_levelling.h"
#include "uasp.h"

static const char *TAG = "uasp";

// ---------------------------------------------------------------
// Endpoint addresses (must match descriptor order in main.c)
// ---------------------------------------------------------------
#define EP_CMD_OUT  0x01   // Bulk OUT – command pipe
#define EP_STS_IN   0x82   // Bulk IN  – status pipe  [SWAPPED: EP2 for persistent Status URB diagnostic]
#define EP_DIN_IN   0x81   // Bulk IN  – data-in pipe [SWAPPED: EP1 for per-cmd Data-In URB diagnostic]
#define EP_DOUT_OUT 0x02   // Bulk OUT – data-out pipe

// DWC2 IN-endpoint DIEPCTL register access for ESP32-S3.
// After edpt_activate() sets USBACTEP=1 without SNAK, the DWC2 responds
// to IN tokens with ZLPs (NAKSTS=0, EPENA=0, empty TX FIFO). The host's
// Data-IN URB then completes with 0 bytes and is not resubmitted.
// Writing SNAK (bit 27) forces the endpoint to NAK until we arm it with
// usbd_edpt_xfer(), which clears NAK via CNAK+EPENA.
#define DWC2_BASE       0x60080000UL
#define DWC2_DIEPCTL(n) (*(volatile uint32_t *)(DWC2_BASE + 0x900 + (n)*0x20))
#define DWC2_DIEPINT(n) (*(volatile uint32_t *)(DWC2_BASE + 0x900 + (n)*0x20 + 0x08))
#define DWC2_DAINTMSK   (*(volatile uint32_t *)(DWC2_BASE + 0x81C))
#define DIEPCTL_SNAK    (1u << 27)

// ---------------------------------------------------------------
// Driver state
// ---------------------------------------------------------------
typedef enum {
    STATE_INIT,
    STATE_WAIT_CMD,
    STATE_DATA_OUT,
    STATE_DATA_IN,           // DIN pending (sector-based multi-chunk); STS not yet armed
    STATE_DATA_IN_AND_STS,   // DIN and STS both armed simultaneously (was concurrent path)
    STATE_SEND_STS,          // STS pending only (no-data or data-out command)
    STATE_STS_BEFORE_DIN,    // STS armed first; DIN will be armed after STS fires
    STATE_DIN_AFTER_STS,     // STS already sent; DIN now pending; re-arm CMD on DIN done
} uas_state_t;

// Chunk size for data transfers (multiple of block size, fits in DRAM)
#define DATA_BUF_BLOCKS  8u
#define DATA_BUF_SIZE    (DATA_BUF_BLOCKS * UASP_BLOCK_SIZE)  // 4096 bytes

// All DMA buffers in internal DRAM, 4-byte aligned
static CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN uint8_t s_cmd_buf[sizeof(uas_cmd_iu_t) + 16];  // +16 for extra CDB
static CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN uint8_t s_sts_buf[sizeof(uas_sense_iu_t)];
static CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN uint8_t s_data_buf[DATA_BUF_SIZE];

static uas_state_t   s_state;
static wl_handle_t   s_wl;
static const uint8_t *s_ep_start;  // pointer to first EP desc in config (flash)
static bool          s_active;     // true when hardware endpoints are open

// Current command context
static uint16_t s_tag;           // echoed in status IU
static uint32_t s_lba;           // next LBA to transfer
static uint32_t s_lba_remaining; // sectors still to transfer
static bool     s_write_error;
// Concurrent-pipe tracking for STATE_DATA_IN_AND_STS
static bool     s_din_done;      // EP82 xfer has completed
static bool     s_sts_done;      // EP81 xfer has completed
// Sequential STS-before-DIN tracking for STATE_STS_BEFORE_DIN / STATE_DIN_AFTER_STS
static uint16_t s_pending_din_len;  // DIN byte count to arm after STS fires

// Cached storage geometry
static uint32_t s_total_sectors;
static uint32_t s_wl_sec_size;   // WL erase granularity (bytes)

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------

static inline uint16_t be16(uint16_t v) { return __builtin_bswap16(v); }
static inline uint32_t be32(uint32_t v) { return __builtin_bswap32(v); }

// Extract big-endian 32-bit value from an unaligned byte pointer
static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

static uint16_t get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

// ---------------------------------------------------------------
// Status IU construction
// ---------------------------------------------------------------

// Build a "GOOD" Sense IU (no sense data).
static uint16_t build_sense_good(uint16_t tag)
{
    uas_sense_iu_t *s = (uas_sense_iu_t *)s_sts_buf;
    memset(s, 0, sizeof(uas_sense_iu_t));
    s->iu_id  = UAS_IU_ID_SENSE;
    s->tag    = be16(tag);
    s->status = SCSI_STATUS_GOOD;
    // sense_len = 0 → send 16-byte header (sense_data starts at byte 16)
    return 16;
}

// Build a CHECK CONDITION Sense IU with fixed-format sense data.
static uint16_t build_sense_check(uint16_t tag,
                                  uint8_t sense_key,
                                  uint8_t asc,
                                  uint8_t ascq)
{
    uas_sense_iu_t *s = (uas_sense_iu_t *)s_sts_buf;
    memset(s, 0, sizeof(uas_sense_iu_t));
    s->iu_id  = UAS_IU_ID_SENSE;
    s->tag    = be16(tag);
    s->status = SCSI_STATUS_CHECK_CONDITION;

    // SCSI fixed-format sense data (SPC-4 §4.5.3)
    uint8_t *sd = s->sense_data;
    sd[0]  = SCSI_SENSE_CURRENT;         // response code, valid=0
    sd[2]  = sense_key & 0x0F;
    sd[7]  = 10;                         // additional sense length = 18 - 8
    sd[12] = asc;
    sd[13] = ascq;
    uint16_t sdlen = 18;

    s->sense_len = be16(sdlen);
    return (uint16_t)(16 + sdlen);
}

// ---------------------------------------------------------------
// Storage read / write helpers
// ---------------------------------------------------------------

// Read up to DATA_BUF_BLOCKS sectors starting at s_lba.
// Returns number of sectors actually staged into s_data_buf, 0 on error.
static uint32_t stage_read_chunk(void)
{
    uint32_t n = s_lba_remaining;
    if (n > DATA_BUF_BLOCKS) n = DATA_BUF_BLOCKS;

    size_t off = (size_t)s_lba * UASP_BLOCK_SIZE;
    esp_err_t err = wl_read(s_wl, off, s_data_buf, n * UASP_BLOCK_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wl_read lba=%"PRIu32" err=%d", s_lba, err);
        return 0;
    }
    return n;
}

// Write a complete chunk of n sectors (already in s_data_buf) at s_lba.
// Performs read-modify-write when n*512 isn't a full WL sector.
static esp_err_t flush_write_chunk(uint32_t n_sectors)
{
    uint32_t sectors_per_wl = s_wl_sec_size / UASP_BLOCK_SIZE;  // e.g. 8
    uint32_t cur_lba = s_lba;
    const uint8_t *src = s_data_buf;
    uint32_t rem = n_sectors;
    // Temporary RMW buffer lives on heap to avoid large stack allocation
    static uint8_t s_rmw_buf[4096];  // must be <= s_wl_sec_size

    while (rem > 0) {
        uint32_t wl_blk         = cur_lba / sectors_per_wl;
        uint32_t offset_in_wl   = cur_lba % sectors_per_wl;
        uint32_t avail_in_wl    = sectors_per_wl - offset_in_wl;
        uint32_t to_write       = rem < avail_in_wl ? rem : avail_in_wl;

        size_t wl_byte_off = wl_blk * s_wl_sec_size;

        if (offset_in_wl != 0 || to_write < sectors_per_wl) {
            // Partial WL sector — read first
            esp_err_t err = wl_read(s_wl, wl_byte_off, s_rmw_buf, s_wl_sec_size);
            if (err != ESP_OK) return err;
        }

        // Patch the relevant 512-byte slots
        memcpy(s_rmw_buf + offset_in_wl * UASP_BLOCK_SIZE,
               src,
               to_write * UASP_BLOCK_SIZE);

        // Erase then write back the full WL sector
        esp_err_t err = wl_erase_range(s_wl, wl_byte_off, s_wl_sec_size);
        if (err != ESP_OK) return err;
        err = wl_write(s_wl, wl_byte_off, s_rmw_buf, s_wl_sec_size);
        if (err != ESP_OK) return err;

        cur_lba += to_write;
        src     += to_write * UASP_BLOCK_SIZE;
        rem     -= to_write;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------
// SCSI command dispatcher
// Called after a Command IU arrives.  Returns the number of bytes
// to transfer (0 for no-data), and sets s_lba / s_lba_remaining for
// data commands.  Direction is encoded by *data_in.
// ---------------------------------------------------------------
typedef enum { DIR_NONE, DIR_IN, DIR_OUT } scsi_dir_t;

static uint16_t s_sts_len;   // bytes to send on EP_STS when we reach SEND_STS

static scsi_dir_t dispatch_scsi(const uint8_t *cdb, uint32_t *xfer_len_out)
{
    *xfer_len_out = 0;
    scsi_dir_t dir = DIR_NONE;

    switch (cdb[0]) {

    // ---- TEST UNIT READY --------------------------------------------------
    case SCSI_CMD_TEST_UNIT_READY:
        s_sts_len = build_sense_good(s_tag);
        break;

    // ---- REQUEST SENSE ----------------------------------------------------
    case SCSI_CMD_REQUEST_SENSE: {
        uint8_t alloc = cdb[4];
        uint8_t *d = s_data_buf;
        memset(d, 0, 18);
        d[0] = SCSI_SENSE_CURRENT;
        d[2] = SENSE_KEY_NO_SENSE;
        d[7] = 10;
        uint32_t n = alloc < 18 ? alloc : 18;
        *xfer_len_out = n;
        dir = DIR_IN;
        s_lba_remaining = 0;    // handled via raw bytes, not sectors
        s_sts_len = build_sense_good(s_tag);
        break;
    }

    // ---- INQUIRY ----------------------------------------------------------
    case SCSI_CMD_INQUIRY: {
        uint16_t alloc = get_be16(cdb + 3);
        if (cdb[1] & 0x01) {
            // EVPD=1: unsupported VPD page → ILLEGAL REQUEST
            s_sts_len = build_sense_check(s_tag,
                SENSE_KEY_ILLEGAL_REQUEST, ASC_INVALID_FIELD, 0x00);
            break;
        }
        uint8_t *d = s_data_buf;
        memset(d, 0, 36);
        d[0]  = 0x00;   // Direct-access block device
        d[1]  = 0x80;   // Removable
        d[2]  = 0x05;   // SPC-3
        d[3]  = 0x02;   // Response data format
        d[4]  = 31;     // Additional length (36 - 5)
        memcpy(d + 8,  "ESP32-S3", 8);
        memcpy(d + 16, "UASP Storage    ", 16);
        memcpy(d + 32, "1.00", 4);
        uint32_t n = alloc < 36 ? alloc : 36;
        *xfer_len_out = n;
        dir = DIR_IN;
        s_lba_remaining = 0;
        s_sts_len = build_sense_good(s_tag);
        ESP_LOGI(TAG, "INQUIRY: alloc=%u sending %"PRIu32" bytes on EP_DIN_IN", alloc, n);
        break;
    }

    // ---- MODE SENSE (6) ---------------------------------------------------
    case SCSI_CMD_MODE_SENSE_6: {
        uint8_t alloc = cdb[4];
        uint8_t *d = s_data_buf;
        memset(d, 0, 4);
        d[0] = 3;   // mode data length (3 bytes follow)
        // medium type = 0, device-specific = 0, block descriptor length = 0
        uint32_t n = alloc < 4 ? alloc : 4;
        *xfer_len_out = n;
        dir = DIR_IN;
        s_lba_remaining = 0;
        s_sts_len = build_sense_good(s_tag);
        break;
    }

    // ---- MODE SELECT (6) --------------------------------------------------
    case SCSI_CMD_MODE_SELECT_6:
        // Accept but ignore; host may send this at init
        s_sts_len = build_sense_good(s_tag);
        break;

    // ---- PREVENT ALLOW MEDIUM REMOVAL ------------------------------------
    case SCSI_CMD_PREVENT_ALLOW_REMOVAL:
        s_sts_len = build_sense_good(s_tag);
        break;

    // ---- START STOP UNIT -------------------------------------------------
    case SCSI_CMD_START_STOP_UNIT:
        s_sts_len = build_sense_good(s_tag);
        break;

    // ---- READ CAPACITY (10) ----------------------------------------------
    case SCSI_CMD_READ_CAPACITY_10: {
        uint8_t *d = s_data_buf;
        uint32_t last_lba = s_total_sectors - 1;
        d[0] = (last_lba >> 24) & 0xFF;
        d[1] = (last_lba >> 16) & 0xFF;
        d[2] = (last_lba >> 8)  & 0xFF;
        d[3] =  last_lba        & 0xFF;
        d[4] = (UASP_BLOCK_SIZE >> 24) & 0xFF;
        d[5] = (UASP_BLOCK_SIZE >> 16) & 0xFF;
        d[6] = (UASP_BLOCK_SIZE >> 8)  & 0xFF;
        d[7] =  UASP_BLOCK_SIZE        & 0xFF;
        *xfer_len_out = 8;
        dir = DIR_IN;
        s_lba_remaining = 0;
        s_sts_len = build_sense_good(s_tag);
        break;
    }

    // ---- READ CAPACITY (16) via SERVICE ACTION IN(16) --------------------
    case SCSI_CMD_READ_CAPACITY_16: {
        if (cdb[1] != 0x10) {
            // Not READ CAPACITY 16, unsupported
            s_sts_len = build_sense_check(s_tag,
                SENSE_KEY_ILLEGAL_REQUEST, ASC_INVALID_COMMAND, 0x00);
            break;
        }
        uint8_t *d = s_data_buf;
        memset(d, 0, 32);
        uint64_t last = (uint64_t)s_total_sectors - 1;
        d[0] = (last >> 56) & 0xFF; d[1] = (last >> 48) & 0xFF;
        d[2] = (last >> 40) & 0xFF; d[3] = (last >> 32) & 0xFF;
        d[4] = (last >> 24) & 0xFF; d[5] = (last >> 16) & 0xFF;
        d[6] = (last >>  8) & 0xFF; d[7] =  last        & 0xFF;
        d[8]  = (UASP_BLOCK_SIZE >> 24) & 0xFF;
        d[9]  = (UASP_BLOCK_SIZE >> 16) & 0xFF;
        d[10] = (UASP_BLOCK_SIZE >> 8)  & 0xFF;
        d[11] =  UASP_BLOCK_SIZE        & 0xFF;
        *xfer_len_out = 32;
        dir = DIR_IN;
        s_lba_remaining = 0;
        s_sts_len = build_sense_good(s_tag);
        break;
    }

    // ---- READ (10) -------------------------------------------------------
    case SCSI_CMD_READ_10: {
        uint32_t lba   = get_be32(cdb + 2);
        uint16_t count = get_be16(cdb + 7);
        if (lba + count > s_total_sectors) {
            s_sts_len = build_sense_check(s_tag,
                SENSE_KEY_ILLEGAL_REQUEST, ASC_INVALID_FIELD, 0x00);
            break;
        }
        s_lba           = lba;
        s_lba_remaining = count;
        *xfer_len_out   = (uint32_t)count * UASP_BLOCK_SIZE;
        dir = DIR_IN;
        s_sts_len = 0;  // will be set after all data is sent
        break;
    }

    // ---- WRITE (10) ------------------------------------------------------
    case SCSI_CMD_WRITE_10: {
        uint32_t lba   = get_be32(cdb + 2);
        uint16_t count = get_be16(cdb + 7);
        if (lba + count > s_total_sectors) {
            s_sts_len = build_sense_check(s_tag,
                SENSE_KEY_ILLEGAL_REQUEST, ASC_INVALID_FIELD, 0x00);
            break;
        }
        s_lba           = lba;
        s_lba_remaining = count;
        s_write_error   = false;
        *xfer_len_out   = (uint32_t)count * UASP_BLOCK_SIZE;
        dir = DIR_OUT;
        s_sts_len = 0;  // will be set after all data is received
        break;
    }

    // ---- SYNCHRONIZE CACHE (10) ------------------------------------------
    case SCSI_CMD_SYNC_CACHE_10:
        // Flash writes are synchronous; nothing to do
        s_sts_len = build_sense_good(s_tag);
        break;

    // ---- Unknown command -------------------------------------------------
    default:
        ESP_LOGW(TAG, "Unsupported SCSI cmd 0x%02X", cdb[0]);
        s_sts_len = build_sense_check(s_tag,
            SENSE_KEY_ILLEGAL_REQUEST, ASC_INVALID_COMMAND, 0x00);
        break;
    }

    return dir;
}

// Pre-clear NAK on an IN endpoint before arming it with usbd_edpt_xfer.
//
// After SNAK is applied (open-time or after-status), NAKSTS=1 sits in dep->ctl.
// edpt_schedule_packets reads dep->ctl and writes it back with CNAK+EPENA; on
// ESP32-S3 DWC2, writing NAKSTS=1 simultaneously with CNAK=1 can leave the
// endpoint stuck in NAK (no data is sent).  Writing CNAK alone first atomically
// clears NAKSTS so that the subsequent CNAK+EPENA write sees nak_status=0.
static bool uas_send_status(uint8_t rhport, uint8_t *buf, uint16_t len)
{
    DWC2_DIEPCTL(tu_edpt_number(EP_STS_IN)) |= (1u << 26);  // pre-CNAK
    return usbd_edpt_xfer(rhport, EP_STS_IN, buf, len);
}

static bool uas_arm_data_in(uint8_t rhport, uint8_t *buf, uint16_t len)
{
    DWC2_DIEPCTL(tu_edpt_number(EP_DIN_IN)) |= (1u << 26);  // pre-CNAK
    return usbd_edpt_xfer(rhport, EP_DIN_IN, buf, len);
}

// ---------------------------------------------------------------
// TinyUSB class driver callbacks
// ---------------------------------------------------------------

static void uas_init(void)
{
    s_state = STATE_INIT;
    ESP_LOGI(TAG, "uas_init called");
}

static void uas_reset(uint8_t rhport)
{
    (void)rhport;
    s_state  = STATE_INIT;
    s_active = false;
    ESP_LOGD(TAG, "reset");
}

static uint16_t uas_open(uint8_t rhport,
                          tusb_desc_interface_t const *itf_desc,
                          uint16_t max_len)
{
    (void)rhport; (void)max_len;

    if (itf_desc->bInterfaceClass    != UAS_INTERFACE_CLASS ||
        itf_desc->bInterfaceSubClass != UAS_INTERFACE_SUBCLASS) {
        return 0;
    }

    uint16_t drv_len = sizeof(tusb_desc_interface_t);
    const uint8_t *p = (const uint8_t *)itf_desc + drv_len;

    // If called with alt=0 (the idle/BOT placeholder, 0 endpoints), skip past it
    // to find alt=1 which carries the UAS endpoints.  We consume both in one shot
    // so TinyUSB doesn't re-call open() on alt=1 (which would fire a duplicate
    // interface assert in process_set_config).
    if (itf_desc->bAlternateSetting == 0) {
        if (itf_desc->bNumEndpoints != 0) {
            ESP_LOGE(TAG, "uas_open: alt=0 expected 0 endpoints, got %u",
                     itf_desc->bNumEndpoints);
            return 0;
        }
        // Next descriptor must be alt=1 interface with UAS protocol and 4 endpoints
        const tusb_desc_interface_t *alt1 = (const tusb_desc_interface_t *)p;
        if (alt1->bDescriptorType    != TUSB_DESC_INTERFACE      ||
            alt1->bAlternateSetting  != 1                        ||
            alt1->bInterfaceProtocol != UAS_INTERFACE_PROTOCOL   ||
            alt1->bNumEndpoints      != 4) {
            ESP_LOGE(TAG, "uas_open: bad alt=1: type=0x%02X alt=%u proto=0x%02X eps=%u",
                     alt1->bDescriptorType, alt1->bAlternateSetting,
                     alt1->bInterfaceProtocol, alt1->bNumEndpoints);
            return 0;
        }
        p += sizeof(tusb_desc_interface_t);
        drv_len += sizeof(tusb_desc_interface_t);
    } else if (itf_desc->bInterfaceProtocol != UAS_INTERFACE_PROTOCOL) {
        return 0;
    }

    // p now points to the first endpoint descriptor (in alt=1)
    s_ep_start = p;

    for (int i = 0; i < 4; i++) {
        const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
        if (ep->bDescriptorType != TUSB_DESC_ENDPOINT) {
            ESP_LOGE(TAG, "uas_open: EP%d: expected ENDPOINT, got 0x%02X",
                     i, ep->bDescriptorType);
            return 0;
        }
        p += sizeof(tusb_desc_endpoint_t);
        drv_len += sizeof(tusb_desc_endpoint_t);

        if (p[0] != 4 || p[1] != USB_DT_PIPE_USAGE) {
            ESP_LOGE(TAG, "uas_open: EP%d: bad Pipe Usage desc len=%d type=0x%02X",
                     i, p[0], p[1]);
            return 0;
        }
        p += 4;
        drv_len += 4;
    }

    ESP_LOGI(TAG, "uas_open: claimed %u bytes (alt=0+alt=1), awaiting SET_INTERFACE(1)",
             drv_len);
    return drv_len;
}

static bool uas_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const *req)
{
    if (req->bRequest != TUSB_REQ_SET_INTERFACE) {
        return false;
    }

    if (stage == CONTROL_STAGE_SETUP) {
        uint8_t const alt = (uint8_t)(req->wValue & 0xFF);

        if (!s_active && s_ep_start) {
            // First activation: open all 4 hardware endpoints.  Hardware is
            // ready before the ACK ZLP so the host can post URBs the moment
            // usb_set_interface() returns on the host side.
            const uint8_t *p = s_ep_start;
            for (int i = 0; i < 4; i++) {
                const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
                if (!usbd_edpt_open(rhport, ep)) {
                    ESP_LOGE(TAG, "SET_INTERFACE(alt=%u): edpt_open failed 0x%02X",
                             alt, ep->bEndpointAddress);
                    return false;
                }
                // Without SNAK, the DWC2 responds to IN tokens on idle IN endpoints
                // with ZLPs (EPENA=0, NAKSTS=0).  For EP81 (status pipe) the Linux
                // uas_stat_cmplt handler resubmits on short receive, so ZLPs are
                // tolerated.  For EP82 (data pipe) uas_data_cmplt does NOT resubmit,
                // so a ZLP before the command arrives kills the Data-In URB permanently.
                // Force NAK on all IN endpoints until we arm them with real data.
                if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                    DWC2_DIEPCTL(tu_edpt_number(ep->bEndpointAddress)) |= DIEPCTL_SNAK;
                }
                p += sizeof(tusb_desc_endpoint_t) + 4;
            }
            s_active = true;
            s_state = STATE_WAIT_CMD;
            if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                ESP_LOGE(TAG, "SET_INTERFACE: arm EP_CMD_OUT failed");
            ESP_LOGI(TAG, "SET_INTERFACE(alt=%u): UAS activated, DAINTMSK=0x%08lX",
                     alt, DWC2_DAINTMSK);
        } else if (s_active) {
            // Re-activate: hardware is already open; just reset state and
            // re-arm the command pipe if it isn't already queued.
            s_state = STATE_WAIT_CMD;
            if (!usbd_edpt_busy(rhport, EP_CMD_OUT)) {
                if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                    ESP_LOGE(TAG, "SET_INTERFACE re-arm: arm EP_CMD_OUT failed");
            }
            ESP_LOGI(TAG, "SET_INTERFACE(alt=%u): re-armed", alt);
        }

        return tud_control_status(rhport, req);
    }

    return true;  // CONTROL_STAGE_ACK: handled
}

static bool uas_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                         xfer_result_t result, uint32_t xferred_bytes)
{
    if (result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "ep 0x%02X xfer failed result=%d", ep_addr, result);
        // Restart command reception
        s_state = STATE_WAIT_CMD;
        if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
            ESP_LOGE(TAG, "error recovery: arm EP_CMD_OUT failed");
        return true;
    }

    // ---- Status pipe sent ------------------------------------------------
    if (ep_addr == EP_STS_IN) {
        ESP_LOGI(TAG, "STS sent tag=%u state=%d", s_tag, s_state);
        DWC2_DIEPCTL(tu_edpt_number(EP_STS_IN)) |= DIEPCTL_SNAK;

        if (s_state == STATE_DATA_IN_AND_STS) {
            // EP82 and EP81 were armed simultaneously.  EP82 may still be pending.
            s_sts_done = true;
            if (!s_din_done) return true;  // wait for EP82 to finish
            // Both complete — fall through to re-arm
        } else if (s_state == STATE_STS_BEFORE_DIN) {
            // STS just fired; now arm DIN.  Tests whether Data-In URB is still
            // alive on the host after STATUS is received.
            s_state = STATE_DIN_AFTER_STS;
            ESP_LOGI(TAG, "STS done, arming DIN len=%u DIEPCTL1=%08lX",
                     s_pending_din_len, DWC2_DIEPCTL(tu_edpt_number(EP_DIN_IN)));
            if (!uas_arm_data_in(rhport, s_data_buf, s_pending_din_len))
                ESP_LOGE(TAG, "xfer EP_DIN_IN failed (DIN_AFTER_STS)");
            return true;
        }
        s_state = STATE_WAIT_CMD;
        if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
            ESP_LOGE(TAG, "re-arm EP_CMD_OUT failed");
        return true;
    }

    // ---- Command pipe received ------------------------------------------
    if (ep_addr == EP_CMD_OUT && s_state == STATE_WAIT_CMD) {
        if (xferred_bytes < sizeof(uas_cmd_iu_t)) {
            ESP_LOGW(TAG, "Short IU (%"PRIu32" bytes)", xferred_bytes);
            if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                ESP_LOGE(TAG, "short IU: re-arm EP_CMD_OUT failed");
            return true;
        }

        const uint8_t iu_id = s_cmd_buf[0];

        if (iu_id == UAS_IU_ID_TASK_MGMT) {
            // Task Management: respond with FUNCTION COMPLETE (0x00)
            uas_response_iu_t *r = (uas_response_iu_t *)s_sts_buf;
            const uas_tmf_iu_t *tmf = (const uas_tmf_iu_t *)s_cmd_buf;
            memset(r, 0, sizeof(*r));
            r->iu_id         = UAS_IU_ID_RESPONSE;
            r->tag           = tmf->tag;
            r->response_code = 0x00;  // FUNCTION COMPLETE
            s_state = STATE_SEND_STS;
            if (!uas_send_status(rhport, s_sts_buf, sizeof(uas_response_iu_t)))
                ESP_LOGE(TAG, "xfer EP_STS_IN failed (task mgmt)");
            return true;
        }

        if (iu_id != UAS_IU_ID_COMMAND) {
            ESP_LOGW(TAG, "Unknown IU ID 0x%02X, ignoring", iu_id);
            if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                ESP_LOGE(TAG, "unknown IU: re-arm EP_CMD_OUT failed");
            return true;
        }

        // Parse Command IU
        const uas_cmd_iu_t *cmd = (const uas_cmd_iu_t *)s_cmd_buf;
        s_tag = be16(cmd->tag);
        ESP_LOGI(TAG, "CMD tag=%u op=0x%02X", s_tag, cmd->cdb[0]);

        uint32_t data_len = 0;
        scsi_dir_t dir = dispatch_scsi(cmd->cdb, &data_len);

        if (dir == DIR_NONE || data_len == 0) {
            // No data phase — send status directly
            s_state = STATE_SEND_STS;
            if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                ESP_LOGE(TAG, "xfer EP_STS_IN failed (no-data cmd 0x%02X)", cmd->cdb[0]);
            return true;
        }

        if (dir == DIR_IN) {
            if (s_lba_remaining == 0) {
                // Short data-in (INQUIRY, READ CAPACITY, etc.) — data already in buf.
                // Arm STS first; arm DIN only after STS fires (in EP_STS_IN xfer_cb).
                // This tests whether the host's DATA-IN URB is still live after STATUS
                // is received, which diagnoses whether the concurrent-arm approach was
                // causing the xHCI to cancel the DATA-IN URB on STATUS completion.
                uint32_t n = data_len < DATA_BUF_SIZE ? data_len : DATA_BUF_SIZE;
                s_pending_din_len = (uint16_t)n;
                s_state = STATE_STS_BEFORE_DIN;
                ESP_LOGI(TAG, "STS_BEFORE_DIN: cmd=0x%02X din_len=%"PRIu32
                         " DIEPCTL2=%08lX DIEPCTL1=%08lX",
                         cmd->cdb[0], n, DWC2_DIEPCTL(2), DWC2_DIEPCTL(1));
                if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                    ESP_LOGE(TAG, "xfer EP_STS_IN failed (STS_BEFORE_DIN cmd 0x%02X)", cmd->cdb[0]);
            } else {
                // Sector-based read
                uint32_t n = stage_read_chunk();
                if (n == 0) {
                    s_sts_len = build_sense_check(s_tag,
                        SENSE_KEY_MEDIUM_ERROR, ASC_READ_ERROR, 0x00);
                    s_lba_remaining = 0;
                    s_state = STATE_SEND_STS;
                    if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                        ESP_LOGE(TAG, "xfer EP_STS_IN failed (read err)");
                } else {
                    s_state = STATE_DATA_IN;
                    if (!uas_arm_data_in(rhport, s_data_buf,
                                        (uint16_t)(n * UASP_BLOCK_SIZE)))
                        ESP_LOGE(TAG, "xfer EP_DIN_IN failed (sector read n=%"PRIu32")", n);
                }
            }
            return true;
        }

        if (dir == DIR_OUT) {
            // Receive first chunk from host
            uint32_t recv = s_lba_remaining;
            if (recv > DATA_BUF_BLOCKS) recv = DATA_BUF_BLOCKS;
            s_state = STATE_DATA_OUT;
            if (!usbd_edpt_xfer(rhport, EP_DOUT_OUT, s_data_buf,
                                 (uint16_t)(recv * UASP_BLOCK_SIZE)))
                ESP_LOGE(TAG, "xfer EP_DOUT_OUT failed");
            return true;
        }
    }

    if (ep_addr == EP_DIN_IN) {
        ESP_LOGI(TAG, "EP_DIN_IN xfer_cb: xferred=%"PRIu32" state=%d lba_rem=%"PRIu32,
                 xferred_bytes, s_state, s_lba_remaining);

        if (s_state == STATE_DATA_IN_AND_STS) {
            // Short data-in: EP82 and EP81 were armed simultaneously.
            // Mark EP82 done; re-arm EP01 only once EP81 also completes.
            s_din_done = true;
            if (!s_sts_done) return true;
            s_state = STATE_WAIT_CMD;
            if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                ESP_LOGE(TAG, "re-arm EP_CMD_OUT failed");
            return true;
        }

        if (s_state == STATE_DIN_AFTER_STS) {
            // Sequential path: STS already sent, DIN now completed.
            ESP_LOGI(TAG, "DIN sent tag=%u xferred=%"PRIu32, s_tag, xferred_bytes);
            s_state = STATE_WAIT_CMD;
            if (!usbd_edpt_xfer(rhport, EP_CMD_OUT, s_cmd_buf, sizeof(s_cmd_buf)))
                ESP_LOGE(TAG, "re-arm EP_CMD_OUT failed (DIN_AFTER_STS)");
            return true;
        }

        if (s_state == STATE_DATA_IN) {
            if (s_lba_remaining == 0) {
                s_sts_len = build_sense_good(s_tag);
                s_state = STATE_SEND_STS;
                if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                    ESP_LOGE(TAG, "xfer EP_STS_IN failed (DATA_IN lba_rem=0)");
                return true;
            }
            uint32_t sent = xferred_bytes / UASP_BLOCK_SIZE;
            s_lba           += sent;
            s_lba_remaining -= sent;
            if (s_lba_remaining == 0) {
                s_sts_len = build_sense_good(s_tag);
                s_state = STATE_SEND_STS;
                if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                    ESP_LOGE(TAG, "xfer EP_STS_IN failed (DATA_IN done)");
                return true;
            }
            uint32_t n = stage_read_chunk();
            if (n == 0) {
                s_sts_len = build_sense_check(s_tag,
                    SENSE_KEY_MEDIUM_ERROR, ASC_READ_ERROR, 0x00);
                s_lba_remaining = 0;
                s_state = STATE_SEND_STS;
                if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                    ESP_LOGE(TAG, "xfer EP_STS_IN failed (DATA_IN read err)");
                return true;
            }
            if (!uas_arm_data_in(rhport, s_data_buf,
                                 (uint16_t)(n * UASP_BLOCK_SIZE)))
                ESP_LOGE(TAG, "xfer EP_DIN_IN failed (DATA_IN next chunk n=%"PRIu32")", n);
            return true;
        }
    }

    // ---- Data-OUT chunk received ----------------------------------------
    if (ep_addr == EP_DOUT_OUT && s_state == STATE_DATA_OUT) {
        uint32_t recvd = xferred_bytes / UASP_BLOCK_SIZE;

        if (recvd > 0) {
            esp_err_t err = flush_write_chunk(recvd);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "wl_write error %d", err);
                s_write_error = true;
            }
            s_lba           += recvd;
            s_lba_remaining -= recvd;
        }

        if (s_lba_remaining == 0) {
            if (s_write_error) {
                s_sts_len = build_sense_check(s_tag,
                    SENSE_KEY_MEDIUM_ERROR, ASC_WRITE_ERROR, 0x00);
            } else {
                s_sts_len = build_sense_good(s_tag);
            }
            s_state = STATE_SEND_STS;
            if (!uas_send_status(rhport, s_sts_buf, s_sts_len))
                ESP_LOGE(TAG, "xfer EP_STS_IN failed (DATA_OUT done)");
            return true;
        }

        // Receive next chunk
        uint32_t recv = s_lba_remaining;
        if (recv > DATA_BUF_BLOCKS) recv = DATA_BUF_BLOCKS;
        if (!usbd_edpt_xfer(rhport, EP_DOUT_OUT, s_data_buf,
                            (uint16_t)(recv * UASP_BLOCK_SIZE)))
            ESP_LOGE(TAG, "xfer EP_DOUT_OUT failed (next chunk)");
        return true;
    }

    ESP_LOGW(TAG, "Unhandled xfer_cb ep=0x%02X state=%d", ep_addr, s_state);
    return true;
}

// ---------------------------------------------------------------
// TinyUSB application driver registration
// ---------------------------------------------------------------

static usbd_class_driver_t const s_uas_driver = {
    .name            = "UAS",
    .init            = uas_init,
    .deinit          = NULL,
    .reset           = uas_reset,
    .open            = uas_open,
    .control_xfer_cb = uas_control_xfer_cb,
    .xfer_cb         = uas_xfer_cb,
    .xfer_isr        = NULL,
    .sof             = NULL,
};

// Weak-override: TinyUSB calls this to discover application class drivers
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    ESP_LOGI(TAG, "usbd_app_driver_get_cb: registering UAS driver");
    *driver_count = 1;
    return &s_uas_driver;
}

// ---------------------------------------------------------------
// Public init
// ---------------------------------------------------------------
esp_err_t uasp_init(wl_handle_t wl_handle)
{
    s_wl         = wl_handle;
    s_wl_sec_size = wl_sector_size(wl_handle);
    size_t total  = wl_size(wl_handle);
    s_total_sectors = (uint32_t)(total / UASP_BLOCK_SIZE);

    ESP_LOGI(TAG, "storage: %"PRIu32" sectors of %u bytes, WL sector=%"PRIu32,
             s_total_sectors, UASP_BLOCK_SIZE, (uint32_t)s_wl_sec_size);

    if (s_wl_sec_size > sizeof(s_data_buf)) {
        ESP_LOGE(TAG, "WL sector size %"PRIu32" exceeds DATA_BUF_SIZE %d",
                 (uint32_t)s_wl_sec_size, DATA_BUF_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
