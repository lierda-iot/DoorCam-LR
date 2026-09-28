# Third-Party Notices

This document records third-party source material, hardware files, and managed
components used by AM36-DoorCam.

## AM36 Hardware and Enclosure Files

The DK01 schematic, PCB source, development-kit photograph, and seven STEP
models under `docs/hardware/`, `docs/3d-print/`, and
`docs/images/am36-development-kit.jpg` are copied from the public
`lierda-iot/WildCam-LR` repository. The board render in
`docs/images/dk01-board.png` is taken from that project's feature guide with
its key labels removed. Both projects use the same L-LRMAM36-FANN4-DK01
hardware.

- Upstream project: `lierda-iot/WildCam-LR`
- Upstream revision: `1983fc872d422de1e949ce7d18f851ba02905a92`
- License: BSD-3-Clause
- Source: https://github.com/lierda-iot/WildCam-LR

The applicable BSD-3-Clause text is reproduced in the root `LICENSE` file.

## SP0A39 Register Settings

The SP0A39 YUV422 register settings in `main/sp0a39_regs.h` are derived from
Espressif Systems' `esp-video-components` project and were modified for an
8-bit DVP interface and project-specific exposure targets.

- Upstream project: `espressif/esp-video-components`
- Upstream file: `esp_cam_sensor/sensors/sp0a39/private_include/sp0a39_spi_4bit_24Minput_yuv422_640x480_15fps.h`
- Copyright: 2026 Espressif Systems (Shanghai) CO LTD
- License: Apache License 2.0
- Source: https://github.com/espressif/esp-video-components

The original copyright and SPDX license identifiers are retained in the
derived source file.

## Managed Components

ESP-IDF Component Manager downloads the following direct dependencies during
configuration. Their source is not copied into this repository. Exact resolved
versions, transitive dependencies, registries, and integrity hashes are stored
in `dependencies.lock`.

| Component | Locked version | License supplied by the component | Source |
| --- | --- | --- | --- |
| `78/esp-opus` | 1.0.5 | Opus three-clause BSD-style license and patent notices | https://components.espressif.com/components/78/esp-opus/versions/1.0.5 |
| `espressif/esp-sr` | 2.5.1 | ESPRESSIF MIT License, limited to Espressif products | https://components.espressif.com/components/espressif/esp-sr/versions/2.5.1 |
| `lierda-iot/esp_lora_driver` | 0.0.7 | Clear BSD License | https://components-staging.espressif.com/components/lierda-iot/esp_lora_driver/versions/0.0.7 |
| `lvgl/lvgl` | 8.4.0 | MIT License | https://components.espressif.com/components/lvgl/lvgl/versions/8.4.0 |
| `espressif/esp_new_jpeg` | 1.0.2 | ESPRESSIF MIT License, limited to Espressif products | https://components.espressif.com/components/espressif/esp_new_jpeg/versions/1.0.2 |
| `espressif/esp_cam_sensor` | 1.7.0 | Apache License 2.0 | https://components.espressif.com/components/espressif/esp_cam_sensor/versions/1.7.0 |
| `espressif/esp_image_effects` | 1.1.0 | ESPRESSIF MIT License, limited to Espressif products | https://components.espressif.com/components/espressif/esp_image_effects/versions/1.1.0 |

The downloaded component packages contain their complete license texts. Those
terms continue to apply to the component source and to redistributed firmware
that incorporates it.
