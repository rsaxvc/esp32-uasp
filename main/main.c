#include <stdio.h>
#include "esp_log.h"
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

    // Mount the FAT partition via wear levelling
    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_DATA_FAT,
                                 "storage");
    if (!part) {
        ESP_LOGE(TAG, "FAT 'storage' partition not found — check partitions.csv");
        return;
    }

    wl_handle_t wl;
    ESP_ERROR_CHECK(wl_mount(part, &wl));

    ESP_ERROR_CHECK(uasp_init(wl));

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
