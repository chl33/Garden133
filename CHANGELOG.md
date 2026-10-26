# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-10-26

### Added
- Support for **Board v7.0** hardware: P-channel MOSFET (`AO3401A` on GPIO25) power switching for the capacitive moisture sensor with an 800 ms settle window (`HAVE_MOISTURE_POWER_CTL`).
- Support for **Board v8.0** hardware: Ai-Thinker Ra-01SH (`SX1262`) LoRa transceiver support (`HAVE_AI_THINKER_LORA`) via `og3x-lora` v0.7.0, and active-low debug DIP switch on GPIO4 with external pull-up.
- OpenSCAD stencil fixture (`OpenSCAD/stencil_fixture/`) and desiccant enclosure STLs (`OpenSCAD/WaterproofEnclosure/`), plus updated KiCAD schematics, PCB layouts, and BOMs for v7.0 and v8.0.

### Changed
- Upgraded dependencies to released `chl33/og3@^0.7.0` and `chl33/og3x-lora@^0.7.0`, and pinned `espressif32@^7.1.3` and `svelteesp32@3.2.5`.
- Renamed `LOG_METHOD` build flag to `OG3_LOG_METHOD` to avoid macro collisions with `nanopb`.
- Reworked normal-mode startup and deep-sleep state machine to overlap moisture sensor settling with boot and wait between description packets and sensor readings.

### Fixed
- Skip updating the filtered soil moisture estimate when the 5V supply rail drops below 3.2V (preventing invalid readings during low-battery conditions).
- Explicitly configure `INPUT_PULLUP` on TP4056 `CHRG` and `STBY` pins (`HAVE_STANDBY_INPUT`) before reading status.

## [1.0.0] - 2026-03-29

### Added
- New responsive Svelte web interface for status monitoring and configuration (Debug Mode).
- JSON API for full device control and status reporting.
- Standardized `camelCase` naming for all API keys and internal variables.
- Dedicated Moisture Calibration page with live ADC count monitoring.
- Explicit support for Frequency, Spreading Factor, and Signal Bandwidth configuration via Web UI.
- Support for larger partition tables (`min_spiffs.csv`) to accommodate web assets and restore OTA functionality.

### Changed
- Improved moisture sensor calibration accuracy and reliability.
- Reorganized web server routes (legacy HTML interface moved to `/old`).
- Standardized variable naming across firmware and frontend.

### Fixed
- Fixed bug where moisture sensor configuration was not being correctly registered/saved.
- Resolved `undefined` property errors in the web UI for sensor readings.
- Fixed empty dropdowns for LoRa parameters on page reload.
- Corrected ADC voltage sensor naming to match project standards.
