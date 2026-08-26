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
- **RS-485 wiring (network cable, T568B):** T/R+ on Green/White (pin 3), T/R- on Blue/White (pin 5). 120Ω termination resistors are in place at both ends of the ~12ft run.
- **Inverter RS-485 address:** `2` (factory default, set via `#define INVERTER_ADDRESS` in `InverterMonitor.h`).

These pins are owned directly by the Aurora library via raw `Serial2`/`digitalWrite` calls, not by ESPHome's `uart:`/`switch:` components — those are deliberately *not* declared in `inverter.yaml` to avoid double-claiming the same physical pins.

## Software Architecture

| File | Purpose |
|---|---|
| `aurora_inverter/ABBAuroraEnums.h` | Protocol constants — DSP value types, cumulated-energy periods, etc. |
| `aurora_inverter/ABBAuroraStrings.h/.cpp` | Human-readable string tables for protocol state/error codes. |
| `aurora_inverter/ABBAurora.h/.cpp` | Low-level Aurora communication protocol: frame building, CRC16, send/receive over the UART, per-command read methods (`ReadState`, `ReadDSPValue`, `ReadCumulatedEnergy`, etc). |
| `aurora_inverter/InverterMonitor.h` | ESPHome-facing singleton that owns an `ABBAurora` instance, caches each telemetry field as a plain `float`, and drives polling. |

`InverterMonitor` is a plain C++ singleton (`InverterMonitor::get_instance()`), not a component ESPHome's codegen knows about. Because of that, it is **not** driven through ESPHome's normal component `loop()`/`PollingComponent` scheduling — instead:

- `esphome.on_boot` calls `InverterMonitor::get_instance()->setup()` once, directly.
- A YAML `interval:` block calls `InverterMonitor::get_instance()->update()` on a real ESPHome-scheduled timer.

(The `PollingComponent` base class and its constructor interval are vestigial — nothing calls `App.register_component_()` on this object, so `PollingComponent`'s own scheduling never actually runs.)

### Raw value storage

The 29 telemetry fields (`v_in_1`, `temperature_inverter`, `cumulated_energy_today`, etc.) are plain `float` members on `InverterMonitor`, not ESPHome `Sensor` objects — the actual, HA-facing entities are the separate `platform: template` sensors defined in `inverter.yaml`, which read these floats directly (e.g. `InverterMonitor::get_instance()->v_in_1`). Earlier these fields *were* raw, unregistered `Sensor*` objects used purely as internal storage; routing every RS-485 read through `Sensor::publish_state()` meant paying for ESPHome's full filter/callback/controller-registry machinery — and its unconditional state-broadcast log line — for values that were never meant to be their own entities, producing constant `'?'`-named noise in the log (unregistered objects have no name). Switching to plain floats removes both costs; the real template sensors are unaffected, since they always ran on their own independent `update_interval`, not through these internal objects.

`connection_status` (`TextSensor*`) and `serial_reset_count` (`Sensor*`) are the two exceptions — kept as real, named ESPHome objects since they publish rarely (only on an actual connection-state change or a self-heal reset, not every cycle) and were never the source of the log noise.

## Polling & Reporting Intervals

- **RS-485 poll (device ↔ inverter):** every 15 seconds — the `interval:` block in `inverter.yaml`.
- **Sensor reporting (device → Home Assistant):** every 30 seconds — the `&default_sensor` anchor used by every sensor's `update_interval`.

Each `update()` cycle does one `ReadState()` call, and — if that succeeds — roughly 30 further `ReadDSPValue`/`ReadCumulatedEnergy` calls, one per telemetry field.

## Serial Communication Protocol

Two named ends of the link: **Esphome** (the ESP32-S3 running this firmware) and **Inverter** (the ABB/Power-One Aurora unit). Every exchange is Esphome-initiated request / Inverter response — the Inverter never speaks first.

This is ABB/Power-One's own proprietary **Aurora Communication Protocol**, not Modbus. It has a family resemblance (RS-485, single-master polling, `[address][command][data][CRC16]` framing) but diverges in specifics: fixed-size frames rather than length-scaling-with-request, command bytes (`50`/`59`/`78`) that don't map to the standard Modbus function-code table, and a CRC16 algorithm that isn't the standard Modbus polynomial shift — notably, it bitwise-complements the final result, which real Modbus CRC16 never does.

Walking through one full 15-second `update()` cycle, from the `interval:` trigger to the next one:

1. **Cycle starts.** Esphome's `interval:` block (in `inverter.yaml`) calls `InverterMonitor::get_instance()->update()` every 15 seconds.

2. **`ReadState()`.** The cycle always opens with one transaction that checks whether the Inverter is responding at all:
   - Esphome sends the Inverter a 10-byte request: the Inverter's RS-485 address (`2`), command byte **50** ("Read State"), six zero-filled parameter bytes, and a 2-byte CRC16 trailer.
   - The Inverter responds with an 8-byte frame: **transmission state**, **global state**, **inverter state**, **channel 1 state**, **channel 2 state**, **alarm state**, and its own 2-byte CRC16.
   - Fails → the cycle ends immediately; no sensor data is read this pass. `consecutive_failures` increments; after 10 in a row (~2.5 min), the self-heal `reset_serial()` fires once per outage.
   - Succeeds → `consecutive_failures` resets to 0 and the cycle proceeds to read one group of sensors.

3. **One group of sensors is read** — not all ~29 at once. Two fixed groups (`group_a_`, `group_b_`) alternate each *successful* cycle (the toggle only advances on success), so a full refresh of any given sensor takes two successful cycles — nominally 30s, matching Home Assistant's own reporting interval. Each entry is read and stored in turn by `publish_entry()` (into the raw `float` cache, not an ESPHome `Sensor` — see Raw value storage above), with a `delay(5)` + `yield()` afterward so the WiFi/API stack gets a chance to drain other outgoing traffic between reads.

4. **Each individual sensor read is its own request/response transaction**, one of two shapes depending on the field:
   - **DSP value fields** (voltages, currents, power, temperatures, frequency, etc.): Esphome sends the Inverter a 10-byte request: address, command byte **59** ("Read DSP Value"), the specific value-type code for that field (e.g. `23` for `V_IN_1`, `21` for `TEMPERATURE_INVERTER`), a "global" parameter byte (`0` = per-module measurement — the only mode this code uses), three more zero-filled bytes, and a 2-byte CRC16. The Inverter responds with an 8-byte frame: transmission state, global state, a 4-byte IEEE-754 float carrying the requested value (sent most-significant-byte-first; Esphome reassembles it by reversing the byte order into a `float`), and a 2-byte CRC16.
   - **Cumulated-energy fields** (today/week/month/year/total generation): same 10-byte request shape, but command byte **78** ("Read Cumulated Energy") and a period code (`0`=today, `1`=week, `3`=month, `4`=year, `5`=total) in place of the DSP type. The Inverter responds with an 8-byte frame: transmission state, global state, a 4-byte unsigned integer (same byte-reversal convention) carrying cumulated watt-hours, and a 2-byte CRC16.

   The RS-485 transceiver is half-duplex, so a single GPIO on the Esphome side — **GPIO21** (`TX_CONTROL_GPIO`) — controls which direction it's facing: driven `HIGH` (`RS485Transmit`) for Esphome to send, driven `LOW` (`RS485Receive`) for Esphome to listen for the Inverter's response. Every transaction flips it twice:
   - Discard any stale bytes still sitting in Esphome's UART RX buffer from a previous exchange.
   - **GPIO21 → HIGH** (TX-enable), then `delay(40)` to let the transceiver settle into transmit mode before Esphome writes anything.
   - Esphome writes the 10-byte request frame, then calls `flush()` (TX-only — waits for the hardware to finish shifting the last byte out; does not touch the RX buffer).
   - **GPIO21 → LOW** (RX-enable) — no settling delay here. A padding delay after this transition was tried at both 2ms and 20ms and made no measurable improvement (if anything, trended slightly worse), so it was removed; Esphome calls `readBytes()` immediately after the pin flips.
   - Esphome calls `readBytes()` for the expected 8-byte response. This isn't one 100ms deadline for the whole frame — internally it loops byte-by-byte (`timedRead()` → `read()`, one byte per call), and each of the 8 iterations gets its own fresh 100ms window to wait in. In practice most iterations cost almost nothing: the ESP32's UART hardware buffers incoming bytes in the background regardless of when Esphome asks for them, so if several bytes have already arrived by the time the loop reaches them, they're returned instantly with no waiting. The 100ms window is only actually spent on a byte that genuinely hasn't shown up yet — which is exactly what happens on a truncated read: the loop collects whatever bytes already arrived almost instantly, then burns the full 100ms waiting on the one that never comes, and gives up with fewer than 8 bytes.
   - Esphome validates the CRC16 it computes over the first 6 response bytes against the CRC the Inverter sent in the last 2.
   - `MaxAttempt = 1` — no internal retry. A single failed exchange is reported as a failed read for that field this cycle; it's simply retried on the field's next scheduled turn.

   So of GPIO21's two transitions per transaction, only the TX-enable side (HIGH) carries a deliberate settling delay (`delay(40)`); the RX-enable side (LOW) currently has none.

5. **After the group loop.** `power_in_total` is recomputed from the two power sensors' latest states, the group toggle flips so the *other* group gets read next cycle, and the status LED turns back off.

**Measured timing:**
- Successful 8/8-byte reads: ~21ms average, up to ~80ms.
- Truncated reads: ~100ms average — i.e. they run out the full per-byte timeout rather than failing fast. The response is consistently a clean prefix of real bytes followed by silence (never scattered mid-frame corruption), which points at the loss happening at the tail end of the *inverter's own* transmission (driver-disable timing or EMI from its switching electronics) rather than anything fixable from this side of the link — see Known Limitations.
- Gap between one field's read finishing and the next field's transaction starting (within the same group loop): GPIO21 flips back to `HIGH` (TX-enable) ~28ms after `readBytes()` returns — that covers returning up through `ReadDSPValue`/`ReadCumulatedEnergy` into `publish_entry()`, the log/CRC-check lines, `sensor->publish_state()`, the explicit `delay(5)`, and `yield()`. Esphome doesn't actually start writing the next request's bytes on the wire until ~72ms after `readBytes()` returns, once the `delay(40)` TX-settle has also elapsed. This gap doesn't apply after the *last* entry in a group — nothing transmits again until the next scheduled `update()` cycle, 15 seconds later.

## Web UI Sensor Groups

The local ESPHome web page (`inverter.local`) organizes sensors into custom groups (`web_server.sorting_groups` in `inverter.yaml`):

1. **General** — connection status, temperatures, cumulated energy, isolation resistance, serial reset count.
2. **DC** — PV string voltage/current/power, bulk voltage fields.
3. **Grid** — grid voltage/current/power/frequency.

The **Diagnostic** entities (IP address, WiFi strength, uptime, ESPHome version, reset reason, reboot button — defined in the shared `common_interface.yaml`) render *before* these three groups rather than after, because any page with custom sorting groups causes ungrouped entities to render first; this is a cosmetic ordering quirk, not a functional issue.

## Reliability Features

- **Overnight handling:** the inverter powers itself down at night. `ReadState()` will fail continuously for hours; this is normal, not a fault. `connection_status` reports `DISCONNECTED` and clears automatically on the next real success at sunrise.
- **UART lockup detection & self-heal:** if `ReadState()` fails **10 cycles in a row** (~2.5 minutes at the current 15s interval), `InverterMonitor` assumes the ESP32's UART driver may be wedged (as opposed to the inverter simply being offline) and calls `ABBAurora::reset_serial()`, which calls `serial->end()`, waits `delay(100)` before reinitializing, then calls `serial->begin()` and restores the 100ms `readBytes()` timeout. This logs a `WARNING` (`INVERTER_MONITOR` and `ABB_AURORA` tags) at each step of the teardown/reinit — so a crash occurring mid-reset can be pinpointed by which step was last logged — and only fires **once per outage**; it does not keep resetting every 2.5 minutes through a multi-hour overnight outage, since the counter re-arms on the next real success.
- **`Serial Reset Count` sensor** — a persistent counter (General group) tracking how many times the self-heal reset has fired since boot.
- **`Reset Reason` sensor** (from `common_interface.yaml`, applies to every device) — reports the ESP32's actual last reset cause (OTA reboot, task watchdog, brownout, power-on, etc.), useful for telling a deliberate reflash apart from a crash.

## Boot-Time Data Handling

Two related fixes prevent bad or misleading data from reaching Home Assistant in the window right after a boot, before `InverterMonitor` has completed any real reads:

- **`NAN` defaults.** Each raw `float` field defaults to `NAN`, not `0`. If a template sensor's lambda evaluates before that field has ever been successfully read, it returns `NAN` rather than a fabricated zero — `TemplateSensor::update()` still calls `publish_state(NAN)` in that case, but Home Assistant's ESPHome integration treats a NaN sensor value as "unknown," not a real `0 V`/`0 W` reading.
- **A 1-minute publish hold-off.** Each of the 28 raw-telemetry lambdas also checks `InverterMonitor::get_instance()->still_booting()` (`millis() < BOOT_SETTLE_MS`, currently 60000) first, and returns `{}` (skip this update) until a minute has passed since boot. Unlike a `NAN` publish, returning `{}` means `publish_state()` is never called at all, so ESPHome reports `missing_state: true` over the API — the more explicit and unambiguous "no data yet" signal, guaranteed regardless of how Home Assistant happens to handle a raw `NaN` float. This is deliberately a stronger guarantee than the `NAN` default alone provides, at the cost of holding back up to a minute of otherwise-legitimate early readings.

`connection_status` and `serial_reset_count` are not gated by either mechanism — their boot-time values (`DISCONNECTED`, `0`) are accurate statements of reality at boot, not placeholder data.

## Debug Logging

Per-read timing and low-level protocol tracing exist but are dormant by default (`logger.level: INFO` in `inverter.yaml`). To re-enable:

```yaml
logger:
  level: DEBUG
  logs:
    INVERTER_MONITOR: DEBUG   # per-read timing: "read type=5 (62 ms), storing value=411.86"
    ABB_AURORA: DEBUG         # raw TX/RX bytes and CRC check results
```

Note: enabling this adds real per-call overhead (string formatting + network log transmission across ~14-15 reads per cycle) and was observed to be enough, on its own, to occasionally push `update()` past its "interval took a long time" warning threshold — that warning's absence isn't necessarily evidence of a fix elsewhere; check whether debug logging is active before drawing conclusions from it.

`WARNING`-level messages (lockup detection/reset) always show regardless of this setting.

## Known Limitations

- Only a fraction of the Aurora protocol's DSP value types are supported by this inverter model — fields specific to 3-phase central inverters (phase R/S/T), fans, and wind-converter models were removed from `InverterMonitor.h`/`inverter.yaml` since the PVI-5000-6000-OUTD-US never responds to them.
- `V_BULK_POSITIVE`/`V_BULK_NEGATIVE` and `Panel Micro Voltage` were removed — they consistently read `0.0` on this model (a valid response, just an unpopulated field, not a communication failure).
- A residual ~10-15% per-read failure rate remains even with proper 120Ω termination in place. Failed reads are consistently truncated (a clean prefix of real bytes followed by silence, running out the full 100ms per-byte timeout), never scattered mid-frame corruption. Several fixes targeting *our* side of the exchange were tried and measured to have no benefit: a settling delay on RX-enable (tested at both 2ms and 20ms), a longer overall `readBytes()` timeout (150ms), and draining stale bytes from the RX buffer before each transmit (confirmed to find nothing to drain). The consistent "good prefix, then silence" shape points at the loss happening during the *inverter's own* transmission — its half-duplex driver-disable timing or EMI from its switching electronics — which isn't addressable from the ESP32 side.
