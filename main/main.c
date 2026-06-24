#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "wear_levelling.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "device/usbd.h"
#include "uasp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

// ---------------------------------------------------------------
// USB descriptor constants
// ---------------------------------------------------------------
#define EPNUM_CMD_OUT   0x01   // Bulk OUT – command pipe
#define EPNUM_STS_IN    0x82   // Bulk IN  – status pipe
#define EPNUM_DIN_IN    0x81   // Bulk IN  – data-in pipe
#define EPNUM_DOUT_OUT  0x02   // Bulk OUT – data-out pipe

enum {
    ITF_NUM_UAS = 0,
    ITF_NUM_TOTAL
};

// alt-0 interface (9) + alt-1 interface (9) + 4 × [endpoint (7) + pipe-usage (4)] = 62 bytes
#define UASP_ITF_DESC_LEN  (9 + 9 + 4 * (7 + 4))
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + UASP_ITF_DESC_LEN)

// ---------------------------------------------------------------
// USB descriptors
// ---------------------------------------------------------------
static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,   // class info in interface descriptors
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,  // Espressif
    .idProduct          = 0x4005,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

// Pipe Usage Descriptor helper: bLength=4, bDescriptorType=0x24, bPipeID, reserved
#define PIPE_USAGE(pipe_id)  4, 0x24, (pipe_id), 0

static const uint8_t s_fs_config_desc[] = {
    // Configuration descriptor
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Alt-0: idle interface, no endpoints (BOT placeholder; host issues SET_INTERFACE(0,1) for UAS)
    9, TUSB_DESC_INTERFACE, ITF_NUM_UAS, 0, 0,
    0x08, 0x06, 0x50,   // class=MSC, subclass=SCSI, protocol=BOT
    0,                  // iInterface

    // Alt-1: UAS interface with 4 pipe endpoints
    9, TUSB_DESC_INTERFACE, ITF_NUM_UAS, 1, 4,
    0x08, 0x06, 0x62,   // class=MSC, subclass=SCSI, protocol=UAS
    0,                  // iInterface

    // Command pipe: Bulk OUT EP1, max 64 bytes
    7, TUSB_DESC_ENDPOINT, EPNUM_CMD_OUT,  TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
    PIPE_USAGE(UAS_PIPE_COMMAND),

    // Status pipe: Bulk IN EP1, max 64 bytes
    7, TUSB_DESC_ENDPOINT, EPNUM_STS_IN,  TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
    PIPE_USAGE(UAS_PIPE_STATUS),

    // Data-IN pipe: Bulk IN EP2, max 64 bytes
    7, TUSB_DESC_ENDPOINT, EPNUM_DIN_IN,  TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
    PIPE_USAGE(UAS_PIPE_DATA_IN),

    // Data-OUT pipe: Bulk OUT EP2, max 64 bytes
    7, TUSB_DESC_ENDPOINT, EPNUM_DOUT_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
    PIPE_USAGE(UAS_PIPE_DATA_OUT),
};

#if TUD_OPT_HIGH_SPEED
static const tusb_desc_device_qualifier_t s_qualifier_desc = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0,
};

// High-speed config: endpoints use 512-byte max packet
static const uint8_t s_hs_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    9, TUSB_DESC_INTERFACE, ITF_NUM_UAS, 0, 0,
    0x08, 0x06, 0x50, 0,

    9, TUSB_DESC_INTERFACE, ITF_NUM_UAS, 1, 4,
    0x08, 0x06, 0x62, 0,

    7, TUSB_DESC_ENDPOINT, EPNUM_CMD_OUT,  TUSB_XFER_BULK, U16_TO_U8S_LE(512), 0,
    PIPE_USAGE(UAS_PIPE_COMMAND),
    7, TUSB_DESC_ENDPOINT, EPNUM_STS_IN,  TUSB_XFER_BULK, U16_TO_U8S_LE(512), 0,
    PIPE_USAGE(UAS_PIPE_STATUS),
    7, TUSB_DESC_ENDPOINT, EPNUM_DIN_IN,  TUSB_XFER_BULK, U16_TO_U8S_LE(512), 0,
    PIPE_USAGE(UAS_PIPE_DATA_IN),
    7, TUSB_DESC_ENDPOINT, EPNUM_DOUT_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(512), 0,
    PIPE_USAGE(UAS_PIPE_DATA_OUT),
};
#endif  // TUD_OPT_HIGH_SPEED

static const char *s_string_desc[] = {
    (const char[]) { 0x09, 0x04 },  // 0: language = English
    "Espressif",                     // 1: Manufacturer
    "ESP32-S3 UASP",                // 2: Product
    "0123456789AB",                 // 3: Serial number
};

// ---------------------------------------------------------------
// LUN 0: wear-leveling flash backend
// ---------------------------------------------------------------
typedef struct {
    wl_handle_t wl;
    uint32_t    sec_size;       // WL erase granularity (bytes)
    uint8_t     rmw_buf[4096];  // scratch for read-modify-write; must be >= sec_size
} wl_ctx_t;

static wl_ctx_t s_wl_ctx;

static esp_err_t wl_lun_read(void *ctx, uint32_t lba,
                              uint8_t *buf, uint32_t n_sectors)
{
    wl_ctx_t *c = (wl_ctx_t *)ctx;
    return wl_read(c->wl, (size_t)lba * UASP_BLOCK_SIZE,
                   buf, n_sectors * UASP_BLOCK_SIZE);
}

static esp_err_t wl_lun_write(void *ctx, uint32_t lba,
                               const uint8_t *buf, uint32_t n_sectors)
{
    wl_ctx_t       *c              = (wl_ctx_t *)ctx;
    uint32_t        sectors_per_wl = c->sec_size / UASP_BLOCK_SIZE;
    uint32_t        cur_lba        = lba;
    const uint8_t  *src            = buf;
    uint32_t        rem            = n_sectors;

    while (rem > 0) {
        uint32_t wl_blk       = cur_lba / sectors_per_wl;
        uint32_t offset_in_wl = cur_lba % sectors_per_wl;
        uint32_t avail_in_wl  = sectors_per_wl - offset_in_wl;
        uint32_t to_write     = rem < avail_in_wl ? rem : avail_in_wl;
        size_t   wl_byte_off  = (size_t)wl_blk * c->sec_size;

        if (offset_in_wl != 0 || to_write < sectors_per_wl) {
            esp_err_t err = wl_read(c->wl, wl_byte_off,
                                    c->rmw_buf, c->sec_size);
            if (err != ESP_OK) return err;
        }
        memcpy(c->rmw_buf + offset_in_wl * UASP_BLOCK_SIZE,
               src, to_write * UASP_BLOCK_SIZE);
        esp_err_t err = wl_erase_range(c->wl, wl_byte_off, c->sec_size);
        if (err != ESP_OK) return err;
        err = wl_write(c->wl, wl_byte_off, c->rmw_buf, c->sec_size);
        if (err != ESP_OK) return err;

        cur_lba += to_write;
        src     += to_write * UASP_BLOCK_SIZE;
        rem     -= to_write;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------
// LUN 1: SPIRAM backend (optional)
// ---------------------------------------------------------------
typedef struct {
    uint8_t *base;
} spiram_ctx_t;

static spiram_ctx_t s_spiram_ctx;

static esp_err_t spiram_lun_read(void *ctx, uint32_t lba,
                                  uint8_t *buf, uint32_t n_sectors)
{
    spiram_ctx_t *c = (spiram_ctx_t *)ctx;
    memcpy(buf, c->base + (size_t)lba * UASP_BLOCK_SIZE,
           n_sectors * UASP_BLOCK_SIZE);
    return ESP_OK;
}

static esp_err_t spiram_lun_write(void *ctx, uint32_t lba,
                                   const uint8_t *buf, uint32_t n_sectors)
{
    spiram_ctx_t *c = (spiram_ctx_t *)ctx;
    memcpy(c->base + (size_t)lba * UASP_BLOCK_SIZE,
           buf, n_sectors * UASP_BLOCK_SIZE);
    return ESP_OK;
}

static void usb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    const char *names[] = {"ATTACHED", "DETACHED", "SUSPENDED", "RESUMED"};
    ESP_LOGI(TAG, "USB event: %s", (event->id < 4) ? names[event->id] : "?");
}

// ---------------------------------------------------------------
// app_main
// ---------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Initialising UASP storage device");

    lun_t   luns[2];
    uint8_t lun_count = 0;

    // LUN 0: wear-leveling flash (always present)
    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    if (!part) {
        ESP_LOGE(TAG, "FAT 'storage' partition not found");
        return;
    }
    ESP_ERROR_CHECK(wl_mount(part, &s_wl_ctx.wl));
    s_wl_ctx.sec_size = wl_sector_size(s_wl_ctx.wl);
    luns[lun_count++] = (lun_t){
        .total_sectors = (uint32_t)(wl_size(s_wl_ctx.wl) / UASP_BLOCK_SIZE),
        .read          = wl_lun_read,
        .write         = wl_lun_write,
        .ctx           = &s_wl_ctx,
    };
    ESP_LOGI(TAG, "LUN0 WL flash: %"PRIu32" sectors", luns[0].total_sectors);

    // LUN 1: SPIRAM RAM disk (optional — only if PSRAM is present)
    size_t spiram_avail = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    spiram_avail = (spiram_avail / UASP_BLOCK_SIZE) * UASP_BLOCK_SIZE;
    if (spiram_avail > 0) {
        void *spiram = heap_caps_malloc(spiram_avail, MALLOC_CAP_SPIRAM);
        if (spiram) {
            s_spiram_ctx.base = (uint8_t *)spiram;
            luns[lun_count++] = (lun_t){
                .total_sectors = (uint32_t)(spiram_avail / UASP_BLOCK_SIZE),
                .read          = spiram_lun_read,
                .write         = spiram_lun_write,
                .ctx           = &s_spiram_ctx,
            };
            ESP_LOGI(TAG, "LUN1 SPIRAM: %zu bytes", spiram_avail);
        }
    }

    ESP_ERROR_CHECK(uasp_init(luns, lun_count));

    // Install TinyUSB driver
    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device              = &s_device_desc;
    cfg.descriptor.full_speed_config   = s_fs_config_desc;
    cfg.descriptor.string              = s_string_desc;
    cfg.descriptor.string_count        =
        sizeof(s_string_desc) / sizeof(s_string_desc[0]);
#if TUD_OPT_HIGH_SPEED
    cfg.descriptor.high_speed_config   = s_hs_config_desc;
    cfg.descriptor.qualifier           = &s_qualifier_desc;
#endif

    cfg.event_cb  = usb_event_cb;
    cfg.event_arg = NULL;

    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
    ESP_LOGI(TAG, "UASP device ready");

    bool last_connected = false, last_mounted = false;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        bool connected = tud_connected();
        bool mounted   = tud_mounted();
        if (connected != last_connected || mounted != last_mounted) {
            ESP_LOGI(TAG, "USB: connected=%d mounted=%d", connected, mounted);
            last_connected = connected;
            last_mounted   = mounted;
        }
    }
}
