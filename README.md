# Foreword

This is my first embedded firmware AI project using Claude Pro.
This hasn't been rigorously tested, and currently only supports
a single block transfer at a time. The rest of this is AI generated:

# ESP32-S3 UASP Device

A USB Attached SCSI Protocol (UASP) mass-storage device driver for the
ESP32-S3, built on ESP-IDF 6 and TinyUSB.

The device exposes two logical units over a single USB interface:

| LUN | Backend                  | Notes |
|-----|--------------------------|-------|
|   0 | SPI flash (wear-leveled) | Always present; ~6 MB usable on an 8 MB part |
|   1 | PSRAM (RAM disk)         | Present only when PSRAM is detected at boot |

UASP achieves higher throughput and lower CPU overhead than the older
Bulk-Only Transport (BOT/MSC) protocol by using four dedicated bulk
endpoints — command, status, data-in, and data-out — and pipelining
transfers without waiting for a command-status wrapper round-trip.

## Hardware

- **Module:** ESP32-S3-WROOM-1 (8 MB flash, 8 MB octal PSRAM)
- **Serial port** (UART0): flashing and console monitoring
- **USB OTG port:** exposes the UASP device to the host

The USB serial/JTAG peripheral is disabled so the OTG port is
exclusively available for UASP.

## Performance (Full Speed USB, ~1 MB/s theoretical max)

| LUN# | Storage |    Read    |    Write   |
|------|---------|------------|------------|
|  0   | WL flash|  ~870 KB/s |   ~43 KB/s |
|  1   | PSRAM   | ~1000 KB/s | ~1000 KB/s |

PSRAM saturates the Full Speed USB bus in both directions. Flash write
speed is limited by the SPI NOR erase-before-write cycle (~50 ms per
4 KB sector); reads are fast enough not to be the bottleneck.

## Protocol details

The driver implements the UASP protocol from the T10 UAS specification (07-144r14).

```
Data-IN:  CMD → [Status pipe] Read Ready IU → arm Data-IN EP → data → Sense IU
Data-OUT: CMD → [Status pipe] Write Ready IU → arm Data-OUT EP → data → Sense IU
No-data:  CMD → [Status pipe] Sense IU
```

The Read Ready IU (0x06) is required before arming the Data-IN endpoint.
Linux's UAS driver (`use_streams=0` for Full Speed devices) does not
submit the Data-In URB until it receives this IU.

SCSI commands handled: `TEST UNIT READY`, `INQUIRY`, `REQUEST SENSE`,
`MODE SENSE (6)`, `READ CAPACITY (10/16)`, `READ (10)`, `WRITE (10)`,
`START STOP UNIT`, `PREVENT ALLOW MEDIUM REMOVAL`, `SYNC CACHE (10)`,
`REPORT LUNS`.

## Project layout

```
main/
  main.c        — USB descriptors, WL and PSRAM backends, app_main
  uasp.c        — TinyUSB custom class driver (state machine, SCSI dispatcher)
  uasp.h        — Public API, IU/SCSI constants, lun_t backend interface
partitions.csv  — 2 MB app + ~6 MB FAT storage partition
sdkconfig.defaults — Build defaults (240 MHz CPU, PSRAM, partition table)
docs/           — UASP and UAS specification PDFs
```

## Storage backend interface

New backends can be added by filling a `lun_t` struct and passing it to
`uasp_init()`:

```c
typedef struct {
    uint32_t    total_sectors;
    esp_err_t (*read) (void *ctx, uint32_t lba, uint8_t       *buf, uint32_t n_sectors);
    esp_err_t (*write)(void *ctx, uint32_t lba, const uint8_t *buf, uint32_t n_sectors);
    void       *ctx;
} lun_t;

esp_err_t uasp_init(const lun_t *luns, uint8_t lun_count);
```

Up to `UASP_MAX_LUNS` (4) logical units are supported.

## Building and flashing

Requires ESP-IDF v6.x with the `esp-tinyusb` component.

```bash
idf.py build
idf.py flash monitor
```

On Linux the device appears as `/dev/sdX` via the `uas` kernel module.
Use `lsblk` or `dmesg | grep uas` to confirm enumeration.

## Known issues / quirks

- **DWC2 ZLP bug:** After `edpt_activate()` the DWC2 USB OTG controller
  starts responding to IN tokens with ZLPs (NAKSTS=0, EPENA=0, empty
  FIFO). The driver writes `SNAK` (bit 27 of `DIEPCTL`) immediately after
  opening the endpoint to force proper NAK until the endpoint is
  explicitly armed.
- **Pre-CNAK race:** Simultaneously writing `CNAK` and `NAKSTS=1` to
  `DIEPCTL` can leave the endpoint stuck in NAK. The driver applies the
  workaround described in the DWC2 programming guide (bit 26 `CNAK` guard)
  before arming IN endpoints.
