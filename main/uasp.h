#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "wear_levelling.h"

// ---------------------------------------------------------------
// UAS (USB Attached SCSI) Information Unit IDs  (T10/07-144)
// ---------------------------------------------------------------
#define UAS_IU_ID_COMMAND        0x01
#define UAS_IU_ID_SENSE          0x03
#define UAS_IU_ID_RESPONSE       0x04
#define UAS_IU_ID_TASK_MGMT      0x05
#define UAS_IU_ID_TASK_MGMT_RESP 0x06

// Pipe Usage Descriptor pipe IDs
#define UAS_PIPE_COMMAND     1
#define UAS_PIPE_STATUS      2
#define UAS_PIPE_DATA_IN     3
#define UAS_PIPE_DATA_OUT    4

// USB class/subclass/protocol for UAS
#define UAS_INTERFACE_CLASS     0x08  // Mass Storage
#define UAS_INTERFACE_SUBCLASS  0x06  // SCSI Transparent Command Set
#define UAS_INTERFACE_PROTOCOL  0x62  // USB Attached SCSI

// Descriptor type for Pipe Usage Descriptor (class-specific)
#define USB_DT_PIPE_USAGE 0x24

// ---------------------------------------------------------------
// UAS Information Unit structures (all multi-byte fields are big-endian)
// ---------------------------------------------------------------

// Command IU: host → device on command pipe
typedef struct {
    uint8_t  iu_id;       // 0x01
    uint8_t  reserved1;
    uint16_t tag;         // big-endian; echoed back in status
    uint8_t  prio_attr;   // bits[2:0] = task attributes, bits[7:3] = priority
    uint8_t  reserved5;
    uint8_t  add_cdb_len; // additional CDB length in DWORDs beyond 16 bytes
    uint8_t  reserved7;
    uint8_t  lun[8];
    uint8_t  cdb[16];
} __attribute__((packed)) uas_cmd_iu_t;

// Sense IU: device → host on status pipe (used for all SCSI command completions)
// T10/07-144 Table 23: byte 6 = Status, bytes 4-5 = Status Qualifier, bytes 12-13 = Sense length
typedef struct {
    uint8_t  iu_id;            // byte 0: 0x03
    uint8_t  reserved1;        // byte 1
    uint16_t tag;              // bytes 2-3 (big-endian)
    uint16_t status_qualifier; // bytes 4-5 (big-endian, 0x0000)
    uint8_t  status;           // byte 6 (SCSI status)
    uint8_t  reserved7[5];     // bytes 7-11
    uint16_t sense_len;        // bytes 12-13 (big-endian, length of sense_data)
    uint8_t  sense_data[96];   // bytes 14-109
} __attribute__((packed)) uas_sense_iu_t;

// Response IU: device → host on status pipe (task management responses)
typedef struct {
    uint8_t  iu_id;       // 0x04
    uint8_t  reserved1;
    uint16_t tag;         // big-endian
    uint8_t  add_resp[3];
    uint8_t  response_code;
} __attribute__((packed)) uas_response_iu_t;

// Task Management IU: host → device on command pipe
typedef struct {
    uint8_t  iu_id;       // 0x05
    uint8_t  reserved1;
    uint16_t tag;         // big-endian
    uint8_t  function;
    uint8_t  reserved5;
    uint16_t task_tag;    // big-endian
    uint8_t  lun[8];
} __attribute__((packed)) uas_tmf_iu_t;

// ---------------------------------------------------------------
// SCSI constants
// ---------------------------------------------------------------
#define SCSI_STATUS_GOOD             0x00
#define SCSI_STATUS_CHECK_CONDITION  0x02

#define SCSI_SENSE_CURRENT           0x70
#define SENSE_KEY_NO_SENSE           0x00
#define SENSE_KEY_NOT_READY          0x02
#define SENSE_KEY_MEDIUM_ERROR       0x03
#define SENSE_KEY_ILLEGAL_REQUEST    0x05
#define SENSE_KEY_UNIT_ATTENTION     0x06

// Additional Sense Codes
#define ASC_INVALID_COMMAND     0x20
#define ASC_INVALID_FIELD       0x24
#define ASC_NOT_READY           0x04
#define ASCQ_NOT_READY          0x00
#define ASC_WRITE_ERROR         0x0C
#define ASC_READ_ERROR          0x11

// SCSI command op-codes we handle
#define SCSI_CMD_TEST_UNIT_READY         0x00
#define SCSI_CMD_REQUEST_SENSE           0x03
#define SCSI_CMD_INQUIRY                 0x12
#define SCSI_CMD_MODE_SELECT_6           0x15
#define SCSI_CMD_MODE_SENSE_6            0x1A
#define SCSI_CMD_START_STOP_UNIT         0x1B
#define SCSI_CMD_PREVENT_ALLOW_REMOVAL   0x1E
#define SCSI_CMD_READ_CAPACITY_10        0x25
#define SCSI_CMD_READ_10                 0x28
#define SCSI_CMD_WRITE_10                0x2A
#define SCSI_CMD_SYNC_CACHE_10           0x35
#define SCSI_CMD_READ_CAPACITY_16        0x9E

// Block size exposed to SCSI host
#define UASP_BLOCK_SIZE  512u

// ---------------------------------------------------------------
// Public API
// ---------------------------------------------------------------

// Initialize the UASP layer; must be called before tinyusb_driver_install().
// wl_handle must be a valid, mounted wear-leveling handle.
esp_err_t uasp_init(wl_handle_t wl_handle);
