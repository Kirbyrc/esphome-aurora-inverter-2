# Inverter Monitor

ESPHome integration that monitors an ABB/Power-One Aurora **PVI-5000-6000-OUTD-US** solar inverter over its RS-485 port and exposes live telemetry (voltages, currents, power, temperatures, cumulated energy, grid readings) to Home Assistant.

Config: [`inverter.yaml`](../inverter.yaml)
Library code: [`aurora_inverter/`](.)
Repository: [github.com/Kirbyrc/esphome](https://github.com/Kirbyrc/esphome)

## Credit

The RS-485/Aurora protocol library (`aurora_inverter/ABBAurora.*`, `aurora_inverter/ABBAuroraStrings.*`, `aurora_inverter/ABBAuroraEnums.h`) was leveraged from [michelsciortino/esphome-aurora-inverter](https://github.com/michelsciortino/esphome-aurora-inverter).

## Hardware

- **Board:** ESP32-S3 (Waveshare ESP32-S3 board with an onboard RS-485 transceiver), `board: esp32-s3-devkitc-1`, `variant: esp32s3`, Arduino framework.
- **Inverter:** ABB/Power-One Aurora PVI-5000-6000-OUTD-US (single-phase, dual-MPPT string inverter, ~6000W class).
- **RS-485 connection** (matches the board's silkscreen RS485 column — `TXD1`/`RXD1`/`RS485_EN`):
  - GPIO17 — TX
  - GPIO18 — RX
  - GPIO21 — direction control (RS485 transmit/receive enable)
- **RS-485 wiring (network cable, T568B):** T/R+ on Green/White (pin 3), T/R- on Blue/White (pin 5).
- **Inverter RS-485 address:** `2` (factory default, set via `#define INVERTER_ADDRESS` in `InverterMonitor.h`).

These pins are owned directly by the Aurora library via raw `Serial2`/`digitalWrite` calls, not by ESPHome's `uart:`/`switch:` components — those are deliberately *not* declared in `inverter.yaml` to avoid double-claiming the same physical pins.

## Software Architecture

| File | Purpose |
|---|---|
| `aurora_inverter/ABBAuroraEnums.h` | Protocol constants — DSP value types, cumulated-energy periods, etc. |
| `aurora_inverter/ABBAuroraStrings.h/.cpp` | Human-readable string tables for protocol state/error codes. |
| `aurora_inverter/ABBAurora.h/.cpp` | Low-level Aurora communication protocol: frame building, CRC16, send/receive over the UART, per-command read methods (`ReadState`, `ReadDSPValue`, `ReadCumulatedEnergy`, etc). |
| `aurora_inverter/InverterMonitor.h` | ESPHome-facing singleton that owns an `ABBAurora` instance, exposes ESPHome `Sensor`/`TextSensor` objects for each telemetry value, and drives polling. |

`InverterMonitor` is a plain C++ singleton (`InverterMonitor::get_instance()`), not a component ESPHome's codegen knows about. Because of that, it is **not** driven through ESPHome's normal component `loop()`/`PollingComponent` scheduling — instead:

- `esphome.on_boot` calls `InverterMonitor::get_instance()->setup()` once, directly.
- A YAML `interval:` block calls `InverterMonitor::get_instance()->update()` on a real ESPHome-scheduled timer.

(The `PollingComponent` base class and its constructor interval are vestigial — nothing calls `App.register_component_()` on this object, so `PollingComponent`'s own scheduling never actually runs.)

## Polling & Reporting Intervals

- **RS-485 poll (device ↔ inverter):** every 15 seconds — the `interval:` block in `inverter.yaml`.
- **Sensor reporting (device → Home Assistant):** every 30 seconds — the `&default_sensor` anchor used by every sensor's `update_interval`.

Each `update()` cycle does one `ReadState()` call, and — if that succeeds — roughly 30 further `ReadDSPValue`/`ReadCumulatedEnergy` calls, one per telemetry field.

## Web UI Sensor Groups

The local ESPHome web page (`inverter.local`) organizes sensors into custom groups (`web_server.sorting_groups` in `inverter.yaml`):

1. **General** — connection status, temperatures, cumulated energy, isolation resistance, serial reset count.
2. **DC** — PV string voltage/current/power, bulk voltage fields.
3. **Grid** — grid voltage/current/power/frequency.

The **Diagnostic** entities (IP address, WiFi strength, uptime, ESPHome version, reset reason, reboot button — defined in the shared `common_interface.yaml`) render *before* these three groups rather than after, because any page with custom sorting groups causes ungrouped entities to render first; this is a cosmetic ordering quirk, not a functional issue.

## Reliability Features

- **Overnight handling:** the inverter powers itself down at night. `ReadState()` will fail continuously for hours; this is normal, not a fault. `connection_status` reports `DISCONNECTED` and clears automatically on the next real success at sunrise.
- **UART lockup detection & self-heal:** if `ReadState()` fails **10 cycles in a row** (~2.5 minutes at the current 15s interval), `InverterMonitor` assumes the ESP32's UART driver may be wedged (as opposed to the inverter simply being offline) and calls `ABBAurora::reset_serial()`, which tears down and re-initializes the UART peripheral. This logs a `WARNING` (`INVERTER_MONITOR` and `ABB_AURORA` tags) and only fires **once per outage** — it does not keep resetting every 2.5 minutes through a multi-hour overnight outage; the counter re-arms on the next real success.
- **`Serial Reset Count` sensor** — a persistent counter (General group) tracking how many times the self-heal reset has fired since boot.
- **`Reset Reason` sensor** (from `common_interface.yaml`, applies to every device) — reports the ESP32's actual last reset cause (OTA reboot, task watchdog, brownout, power-on, etc.), useful for telling a deliberate reflash apart from a crash.

## Debug Logging

Per-read timing and low-level protocol tracing exist but are dormant by default (`logger.level: INFO` in `inverter.yaml`). To re-enable:

```yaml
logger:
  level: DEBUG
  logs:
    INVERTER_MONITOR: DEBUG   # per-read timing: "ReadDSPValue 5 411.86 (62 ms)"
    ABB_AURORA: DEBUG         # raw TX/RX bytes and CRC check results
```

`WARNING`-level messages (lockup detection/reset) always show regardless of this setting.

## Known Limitations

- Only a fraction of the Aurora protocol's DSP value types are supported by this inverter model — fields specific to 3-phase central inverters (phase R/S/T), fans, and wind-converter models were removed from `InverterMonitor.h`/`inverter.yaml` since the PVI-5000-6000-OUTD-US never responds to them.
- `V_BULK_POSITIVE`/`V_BULK_NEGATIVE` and `Panel Micro Voltage` were removed — they consistently read `0.0` on this model (a valid response, just an unpopulated field, not a communication failure).
