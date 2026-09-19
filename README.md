# esphome-elero

ESPHome component for controlling Elero wireless blinds and lights via an ESP32 with a CC1101, SX1262, or SX1276 868 MHz RF transceiver. Bidirectional -- sends commands and receives status feedback.

[![ESPHome](https://img.shields.io/badge/ESPHome-Component-blue)](https://esphome.io/)
[![License](https://img.shields.io/badge/License-MIT-blue)](LICENSE)

## Features

- **Blinds, shutters, awnings and lights** over Elero's 868.35 MHz RF (CC1101, SX1262 or SX1276).
- **Bidirectional** -- sends commands and receives status feedback (state, direction, problems, RSSI).
- **Position tracking** -- derives cover position from measured open/close travel times. Intermediate positions and stop-at-target work without endstops.
- **Venetian tilt support** -- models the "tilt before lift" behaviour of slat blinds: position stays frozen while the slats rotate, and Home Assistant gets a continuous 0-100 % tilt slider. See [Tilt model](docs/CONFIGURATION.md#tilt-model).
- **Devices live in NVS** -- discover, add, edit and remove devices at runtime in the built-in web UI; no per-device YAML. Back up and restore the whole device list as JSON.
- **Home Assistant** via the native API (`elero_nvs:`) or MQTT discovery (`elero_mqtt:`).
- **Web UI** -- live RF packet view, device discovery, learn-in, device editor and hub settings.
- **Groups** -- send one RF command to several blinds that share a remote.
- **ESPHome 2026.8** support, including LilyGO LoRa32 native API builds.

## Quick Start

**ESPHome 2026.8.2 fork:** For the ESP32-PICO-D4 LilyGO LoRa32 with SX1276
and native Home Assistant API, use the [build and hardware-testing guide](docs/ESPHOME-2026-LILYGO.md)
and [native API/NVS configuration](configs/config.lilygo-lora32-api-nvs.yaml).
The generated web UI is downloaded from the matching GitHub release asset, so
ESPHome can pull this fork directly:

```yaml
external_components:
  - source: github://aSauerwein/esphome-elero@main
    refresh: 0s
```

### 1. Choose your hardware

Pick a device config for your board:

- [ESP32 + CC1101](docs/devices/esp32-cc1101.md) -- generic ESP32 with external CC1101 module
- [Heltec WiFi LoRa 32 V4](docs/devices/heltec-lora-v4.md) -- onboard SX1262 (experimental)
- [LILYGO LoRa32 V2.1](docs/devices/lilygo-lora32-sx1276.md) -- onboard SX1276 (experimental)
- [LilyGO T-Embed](docs/devices/lilygo-t-embed.md) -- onboard CC1101

### 2. Choose your output adapter

Devices always live in NVS and are managed at runtime through the web UI. The adapter only decides how Home Assistant sees them:

**`elero_nvs:`** -- ESPHome native API. Recommended for HA-only setups.

```yaml
external_components:
  - source: github://manuschillerdev/esphome-elero

# ... board + SPI + elero config from your device page ...

api:
elero_nvs:
elero_web:
```

**`elero_mqtt:`** -- MQTT HA discovery. Use when you already run an MQTT broker, or when you want device CRUD to apply without rebooting.

```yaml
external_components:
  - source: github://manuschillerdev/esphome-elero

# ... board + SPI + elero config from your device page ...

mqtt:
  broker: !secret mqtt_broker

elero_mqtt:
  topic_prefix: elero
  discovery_prefix: homeassistant
  device_name: "Elero Gateway"

elero_web:
```

### 3. Flash, discover, and add devices

1. Flash your config (`uv run esphome run your-config.yaml`).
2. Open `http://<device-ip>/elero`.
3. Press buttons on your physical Elero remote -- RF packets appear in real time and remotes/blinds are auto-discovered.
4. Save discovered devices in the web UI.
5. (Optional) **Backup**: Hub → Backup & Restore → Download. Keep this JSON safe -- it's how you recover devices after flashing a replacement chip.

### 4. Tune per-device settings

Per-device fields (travel durations, tilt, HA device class, protocol bytes) are edited in the web UI -- see the [Configuration Reference](docs/CONFIGURATION.md).

**Position tracking** (`open_duration` / `close_duration`): stopwatch the full wall-clock travel time and enter it in seconds. Leave both at `0` if you only need open/close.

**Venetian tilt** (`tilt_duration_ms`): for slat blinds that rotate their slats before travelling ("tilt before lift"):

1. Toggle **Tilt** on for the cover in the web UI.
2. Stop the blind mid-travel, tilt the slats closed, and stopwatch one full slat sweep (typically 0.8-2.5 s). Enter it as the tilt duration.
3. Measure `open_duration` / `close_duration` from the fully closed position (slats closed) -- the tilt sweep is already included in the wall-clock time.

With `tilt_duration_ms` set, Home Assistant gets a continuous tilt slider, position stays frozen while the slats rotate, and tilt-only moves are supported. With `tilt_duration_ms = 0` the legacy behaviour applies: tilt commands drive the motor's stored tilt favourite and tilt reports the movement direction extremes. Full details: [Tilt model](docs/CONFIGURATION.md#tilt-model).

### Migrating from older versions (YAML-defined devices)

If you're upgrading from a version where devices were defined under `cover: - platform: elero` / `light: - platform: elero`, see [docs/MIGRATION-yaml-to-nvs.md](docs/MIGRATION-yaml-to-nvs.md). The TL;DR: run `uv run scripts/migrate_yaml_to_json.py old.yaml -o backup.json`, remove the `cover:` / `light:` blocks from YAML, flash, then upload the backup via the web UI.

## Documentation

- [Configuration Reference](docs/CONFIGURATION.md) -- full parameter tables for all modes
- [Installation Guide](docs/INSTALLATION.md) -- step-by-step hardware setup
- [Backup &amp; Restore](docs/BACKUP-RESTORE.md) -- exporting/importing your NVS device list
- [Migration from YAML devices](docs/MIGRATION-yaml-to-nvs.md) -- upgrading from pre-0.11.0
- [Device Configs](docs/devices/) -- board-specific wiring and config
- [example.yaml](example.yaml) -- minimal working config

## Credits

Based on protocol research by [QuadCorei8085/elero_protocol](https://github.com/QuadCorei8085/elero_protocol) (MIT), [stanleypa/eleropy](https://github.com/stanleypa/eleropy) (GPLv3), and [andyboeh/esphome-elero](https://github.com/andyboeh/esphome-elero).
