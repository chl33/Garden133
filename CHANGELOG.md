# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
