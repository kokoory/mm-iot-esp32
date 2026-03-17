# CLAUDE.md - Project Guide for MM-IoT-ESP32

## Project Overview

This is the **Morse Micro Wi-Fi HaLow IoT SDK for ESP-IDF** (Alpha Port, v2.10.4-esp32).
It provides drivers and sample applications to add Wi-Fi HaLow (802.11ah) connectivity
to ESP32 microcontrollers via Morse Micro MM6108/MM8108 transceiver chips.

**Target Hardware:** Seeed Studio XIAO ESP32S3 + Wio-WM6180 Wi-Fi HaLow Module

## Repository Structure

```
mm-iot-esp32/
├── README.md                          # Setup and usage guide
├── CLAUDE.md                          # This file
├── VERSION.md                         # SDK version (v2.10.4-esp32)
├── LICENSES/                          # Apache 2.0, GPL, MIT, BSD, etc.
├── examples/                          # Example applications
│   ├── porting_assistant/             # Hardware validation (SPI/GPIO self-test)
│   ├── scan/                          # Wi-Fi HaLow AP discovery
│   ├── sta_connect/                   # Station mode connection
│   ├── sta_reboot/                    # Station with reboot resilience
│   ├── ap_mode/                       # Access point mode
│   ├── iperf/                         # Network throughput measurement
│   ├── rf-test/                       # RF/radio testing
│   ├── transfer_reset/                # Transfer and reset testing
│   └── web_camera/                    # Camera streaming over HaLow (XIAO Sense)
└── framework/                         # Core SDK components
    ├── mm_shims/                      # Hardware abstraction layer (HAL)
    │   ├── Kconfig                    # Pin config (XIAO ESP32S3 defaults)
    │   ├── mmhal_wlan.c               # SPI communication driver
    │   ├── mmhal_os.c                 # OS integration (init, logging, reset)
    │   ├── mmhal_core.c               # Core HAL (random, sleep veto)
    │   ├── mmhal_wlan_binaries.c      # Firmware/BCF binary linking
    │   ├── mmosal_shim_freertos_esp32.c  # FreeRTOS adaptation layer
    │   └── crypto_mbedtls_mm.c        # Crypto wrapper (mbedTLS)
    ├── morselib/                       # Pre-compiled WLAN driver library
    ├── morsefirmware/                  # MM6108/MM8108 firmware & BCF files
    │   ├── mm6108/bcfs/               # MM6108 board config files
    │   └── mm8108/bcfs/               # MM8108 board config files
    └── src/                           # Framework source components
        ├── hostap/                    # WPA supplicant (WPA/WPA2/WPA3)
        ├── mmipal/                    # IP abstraction (lwIP integration)
        ├── mmiperf/                   # Performance testing
        ├── mmpktmem/                  # Packet memory management
        ├── mmregdb/                   # Regulatory database
        ├── mmutils/                   # Utility functions
        └── slip/                      # SLIP protocol
```

## Build System

- **Build framework:** CMake + ESP-IDF (v5.1.1+, v5.2.2+ for ESP32-C6)
- **RTOS:** FreeRTOS
- **Required env var:** `MMIOT_ROOT` must point to this repo root

### Build Commands

```bash
export MMIOT_ROOT=~/mm-iot-esp32
cd examples/<example_name>
idf.py set-target esp32s3
idf.py menuconfig     # Optional: adjust pin/firmware config
idf.py build
idf.py flash monitor
```

### Country Code

All examples require a country code for regulatory compliance. Set it either:
- In source: uncomment `#define COUNTRY_CODE "US"` in the example's source
- Via CMake: `idf.py -DCOUNTRY_CODE=US build`

## Pin Configuration (XIAO ESP32S3 + WM6180 HaLow)

Default pin mapping configured in `framework/mm_shims/Kconfig`:

| Signal    | GPIO | XIAO Pin | Direction           |
|-----------|------|----------|---------------------|
| RESET_N   | 1    | D0       | Output              |
| WAKE      | 2    | D1       | Output              |
| SPI_IRQ   | 3    | D2       | Input               |
| SPI_CS    | 4    | D3       | Output              |
| BUSY      | 5    | D4       | Input (NOT WIRED)   |
| SPI_SCK   | 7    | D8       | SPI Clock           |
| SPI_MISO  | 8    | D9       | SPI MISO            |
| SPI_MOSI  | 9    | D10      | SPI MOSI            |

**Important:** The BUSY pin is not wired on Seeed XIAO HaLow boards. All examples
call `mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED)` as a workaround.

To change pin configuration for a different board: `idf.py menuconfig` →
`Component config → Morse Micro Shim Configuration`

## Key Architecture Notes

- **HAL Pattern:** `mmhal_*` functions provide platform-independent interface
- **Pre-compiled library:** Core WLAN driver (`morselib`) is a static library
- **Firmware embedding:** `.mbin` firmware files are embedded via objcopy at build time
- **Component system:** ESP-IDF component manager with `idf_component.yml` dependencies
- **SPI communication:** 40MHz SPI clock, GPIO-based interrupt signaling

## Camera Example (web_camera)

The `web_camera` example is specific to **XIAO ESP32S3 Sense** which has an OV2640/OV5640
camera module. It streams MJPEG video over the Wi-Fi HaLow network.

**Note:** The XIAO ESP32S3 Sense SD card pins (GPIO 7, 8, 9) conflict with the HaLow
SPI bus. SD card cannot be used simultaneously with the HaLow module.

Camera pins are hardcoded (fixed on the board):
- XCLK=GPIO10, SIOD=GPIO40, SIOC=GPIO39
- Y2-Y9: GPIO15,17,18,16,14,12,11,48
- VSYNC=GPIO38, HREF=GPIO47, PCLK=GPIO13

## Supported Hardware Targets

| Target     | Architecture   | Notes                        |
|------------|---------------|------------------------------|
| ESP32-S3   | Xtensa 32-bit | Primary target (XIAO)        |
| ESP32-C6   | RISC-V 32     | Requires ESP-IDF v5.2.2+     |
| ESP32-C3   | RISC-V 32     | Compact variant              |
| ESP32-P4   | RISC-V 32     | Low-power option             |

## Common Tasks

### Adding a new example
1. Copy structure from an existing example (e.g., `iperf/`)
2. Create `CMakeLists.txt`, `main/CMakeLists.txt`, `main/idf_component.yml`
3. Reuse `mm_app_common.c/h` and `mm_app_loadconfig.c/h` for WLAN init
4. Add `sdkconfig.defaults` and target-specific variants

### Switching to MM8108 chip
```
CONFIG_MM_BCF_FILE="bcf_mf15457.mbin"
CONFIG_MM_FW_FILE="mm8108b2-rl.mbin"
CONFIG_MMHAL_CHIP_TYPE_MM8108=y
```

## Changes Made (Branch: claude/analyze-repository-Vuwd1)

### 1. XIAO ESP32S3 Pin Configuration
- Updated `framework/mm_shims/Kconfig` defaults from generic ESP32-S3 devkit
  to Seeed Studio XIAO ESP32S3 + WM6180 HaLow module pin mapping
- Reference: [Xiao-Halow-to-WiFi-Bridge](https://github.com/gtgreenw/Xiao-Halow-to-WiFi-Bridge)

### 2. BUSY Pin Workaround
- Added `mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED)` to all 7 examples
  (scan, sta_connect, sta_reboot, iperf, rf-test, transfer_reset, ap_mode already had it)
- Seeed XIAO HaLow boards do not wire the BUSY pin

### 3. Web Camera Example
- Created `examples/web_camera/` for XIAO ESP32S3 Sense + HaLow camera streaming
- MJPEG streaming over HTTP with endpoints: `/`, `/stream`, `/capture`
- Uses `espressif/esp32-camera` component with PSRAM for frame buffers

### 4. README Update
- Added XIAO-specific pin mapping table and documentation
- Updated GPIO log output examples to match new pin configuration
- Added note about BUSY pin workaround
