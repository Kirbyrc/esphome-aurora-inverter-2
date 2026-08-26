#include "esphome.h"
#include <ABBAurora.h>
#include <cmath>

using namespace esphome;
using namespace text_sensor;
using namespace sensor;

#if defined(USE_ESP32)
#define LED 2               //GPIO02, the ESP32 internal led
#define RX 18               //GPIO18
#define TX 17               //GPIO17
#define TX_CONTROL_GPIO 21  //GPIO21
#define INVERTER_MONITOR_SERIAL Serial2

#elif defined(USE_ESP8266)
#define LED LED_BUILTIN
// On esp8266 UART0 *must* use GPIO (1, 3) OR GPIO (15, 13)
// Alternate pins GPIO (15, 13) are not connected to USB-UART
#define RX 13              // GPI13, D7, UART0 alt RX
#define TX 15              // GPI15, D8, UART0 alt TX
#define TX_CONTROL_GPIO 12 // GPIO12, D6 on nodeMCU and d1 mini
#define INVERTER_MONITOR_SERIAL Serial

#else
#error Unsupported device

#endif

#define INVERTER_ADDRESS 2  //Default address; this can be modified on the inverter settings menu


#ifdef USE_ESP8266
// Check UART pins are compatible
#if !((TX == 1 && RX == 3) || (TX == 15 && RX == 13))
#error Unsupported UART pin configuration for esp8266
#endif
#endif


#define CONNECTED "CONNECTED"
#define DISCONNECTED "DISCONNECTED"

#define TAG "INVERTER_MONITOR"

enum class ReadKind : uint8_t { DSP_RAW, CUMULATED_ENERGY };

struct SensorEntry
{
  ReadKind kind;
  uint8_t type_code;  // DSP_VALUE_TYPE or CUMULATED_ENERGY_TYPE, depending on kind
  float *value;  // internal raw-value storage -- not an ESPHome Sensor, so storing
                  // into it doesn't log or hit the API; the real, HA-facing template
                  // sensors in inverter.yaml read this directly via their lambda.
};

class InverterMonitor : public PollingComponent
{

protected:
  InverterMonitor() : PollingComponent(5000) {}
  static InverterMonitor *instance_;

private:
  ABBAurora *inverter = nullptr;
  uint8_t connection = 0;
  uint8_t consecutive_failures = 0;
  bool group_toggle_ = false;  // alternates which half of the sensors gets read/published each cycle
  bool reset_attempted = false;  // only try one serial reset per outage -- avoids
                                  // reset-spamming for hours when the inverter is
                                  // simply powered down overnight (not a lockup)
  static const uint8_t LOCKUP_THRESHOLD = 10;  // ~150s of unbroken failures at the 15s poll interval

public:
  InverterMonitor(InverterMonitor &other) = delete;
  void operator=(const InverterMonitor &) = delete;
  static InverterMonitor *get_instance();

  // Holds HA-facing sensor publishes off for a settling window after boot --
  // otherwise the first lambda evaluations (on their own 30s schedule) could
  // fire before InverterMonitor has completed even one successful read.
  static constexpr uint32_t BOOT_SETTLE_MS = 60000;
  bool still_booting() { return millis() < BOOT_SETTLE_MS; }

  TextSensor *connection_status = new TextSensor();
  Sensor *serial_reset_count = new Sensor();

  // Raw value cache for each telemetry field -- plain floats, not ESPHome
  // Sensor objects. These are never registered with ESPHome and never
  // published themselves; the real, HA-facing template sensors in
  // inverter.yaml read them directly (e.g. "->v_in_1" instead of
  // "->v_in_1->state"). Avoids routing every RS-485 read through the full
  // Sensor::publish_state() machinery (filters, callbacks, controller
  // registry, and its unconditional log line) for values that were never
  // meant to be their own entities.
  float v_in_1 = NAN;
  float v_in_2 = NAN;
  float i_in_1 = NAN;
  float i_in_2 = NAN;
  float power_in_1 = NAN;
  float power_in_2 = NAN;
  float power_in_total = NAN;
  float power_peak_today = NAN;
  float power_peak_max = NAN;
  float temperature_inverter = NAN;
  float temperature_booster = NAN;
  float cumulated_energy_today = NAN;
  float cumulated_energy_week = NAN;
  float cumulated_energy_month = NAN;
  float cumulated_energy_year = NAN;
  float cumulated_energy_total = NAN;
  float grid_voltage = NAN;
  float grid_current = NAN;
  float grid_power = NAN;
  float frequency = NAN;
  float v_bulk = NAN;
  float i_leak_dc_dc = NAN;
  float i_leak_inverter = NAN;
  float dc_dc_grid_voltage = NAN;
  float dc_dc_grid_frequency = NAN;
  float isolation_resistance = NAN;
  float dc_dc_v_bulk = NAN;
  float average_grid_voltage = NAN;
  float v_bulk_mid = NAN;
  float grid_voltage_neutral = NAN;

  // Split across two groups so a full refresh takes two 15s cycles (30s --
  // matching Home Assistant's own reporting interval) instead of reading and
  // publishing all ~29 sensors in one pass. Halves per-cycle RS-485 traffic
  // and publish-burst size.
  const SensorEntry group_a_[15] = {
    {ReadKind::DSP_RAW, (uint8_t)V_IN_1, &v_in_1},
    {ReadKind::DSP_RAW, (uint8_t)V_IN_2, &v_in_2},
    {ReadKind::DSP_RAW, (uint8_t)I_IN_1, &i_in_1},
    {ReadKind::DSP_RAW, (uint8_t)I_IN_2, &i_in_2},
    {ReadKind::DSP_RAW, (uint8_t)POWER_IN_1, &power_in_1},
    {ReadKind::DSP_RAW, (uint8_t)POWER_IN_2, &power_in_2},
    {ReadKind::DSP_RAW, (uint8_t)POWER_PEAK_TODAY, &power_peak_today},
    {ReadKind::DSP_RAW, (uint8_t)POWER_PEAK, &power_peak_max},
    {ReadKind::DSP_RAW, (uint8_t)TEMPERATURE_INVERTER, &temperature_inverter},
    {ReadKind::DSP_RAW, (uint8_t)TEMPERATURE_BOOSTER, &temperature_booster},
    {ReadKind::CUMULATED_ENERGY, (uint8_t)CURRENT_DAY, &cumulated_energy_today},
    {ReadKind::CUMULATED_ENERGY, (uint8_t)CURRENT_WEEK, &cumulated_energy_week},
    {ReadKind::CUMULATED_ENERGY, (uint8_t)CURRENT_MONTH, &cumulated_energy_month},
    {ReadKind::CUMULATED_ENERGY, (uint8_t)CURRENT_YEAR, &cumulated_energy_year},
    {ReadKind::CUMULATED_ENERGY, (uint8_t)TOTAL, &cumulated_energy_total},
  };

  const SensorEntry group_b_[14] = {
    {ReadKind::DSP_RAW, (uint8_t)GRID_VOLTAGE, &grid_voltage},
    {ReadKind::DSP_RAW, (uint8_t)GRID_CURRENT, &grid_current},
    {ReadKind::DSP_RAW, (uint8_t)GRID_POWER, &grid_power},
    {ReadKind::DSP_RAW, (uint8_t)FREQUENCY, &frequency},
    {ReadKind::DSP_RAW, (uint8_t)V_BULK, &v_bulk},
    {ReadKind::DSP_RAW, (uint8_t)I_LEAK_DC_DC, &i_leak_dc_dc},
    {ReadKind::DSP_RAW, (uint8_t)I_LEAK_INVERTER, &i_leak_inverter},
    {ReadKind::DSP_RAW, (uint8_t)DC_DC_GRID_VOLTAGE, &dc_dc_grid_voltage},
    {ReadKind::DSP_RAW, (uint8_t)DC_DC_GRID_FREQUENCY, &dc_dc_grid_frequency},
    {ReadKind::DSP_RAW, (uint8_t)ISOLATION_RESISTANCE, &isolation_resistance},
    {ReadKind::DSP_RAW, (uint8_t)DC_DC_V_BULK, &dc_dc_v_bulk},
    {ReadKind::DSP_RAW, (uint8_t)AVERAGE_GRID_VOLTAGE, &average_grid_voltage},
    {ReadKind::DSP_RAW, (uint8_t)V_BULK_MID, &v_bulk_mid},
    {ReadKind::DSP_RAW, (uint8_t)GRID_VOLTAGE_NEUTRAL, &grid_voltage_neutral},
  };

  void setup() override
  {
    ESP_LOGD(TAG, "Setting up");
    pinMode(LED, OUTPUT);
    ABBAurora::setup(INVERTER_MONITOR_SERIAL, RX, TX, TX_CONTROL_GPIO);
    inverter = new ABBAurora(INVERTER_ADDRESS);
    connection_status->publish_state(DISCONNECTED);
    serial_reset_count->publish_state(0);
  }

  // A real delay() (not just yield()) between each publish gives the
  // WiFi/network stack's background task actual processing time to drain
  // the API's outgoing send queue between publishes.
  void publish_entry(const SensorEntry &entry)
  {
    uint32_t start = millis();
    bool ok;
    float value = 0;
    ESP_LOGD(TAG, "publish_entry: about to read type=%d kind=%d", entry.type_code, (int)entry.kind);
    if (entry.kind == ReadKind::CUMULATED_ENERGY) {
      ok = inverter->ReadCumulatedEnergy((CUMULATED_ENERGY_TYPE)entry.type_code);
      if (ok) value = inverter->CumulatedEnergy.Energy;
    } else {
      ok = inverter->ReadDSPValue((DSP_VALUE_TYPE)entry.type_code, MODULE_MEASUREMENT);
      if (ok) {
        value = inverter->DSP.Value;
      }
    }
    uint32_t elapsed = millis() - start;
    if (ok) {
      ESP_LOGD(TAG, "read type=%d (%u ms), storing value=%f", entry.type_code, elapsed, value);
      *entry.value = value;
    } else {
      ESP_LOGD(TAG, "read type=%d failed (%u ms)", entry.type_code, elapsed);
    }
    delay(5);
    yield();
  }

  void update() override
  {
    ESP_LOGD(TAG, "update() begin (millis=%u)", millis());
    if (inverter == nullptr) {
      ESP_LOGW(TAG, "update() called before setup() has run -- skipping this cycle");
      return;
    }
    uint32_t read_state_start = millis();
    ESP_LOGD(TAG, "update: calling ReadState()");
    bool read_state_ok = inverter->ReadState();
    uint32_t read_state_elapsed = millis() - read_state_start;
    ESP_LOGD(TAG, "ReadState %s (%u ms)", read_state_ok ? "ok" : "failed", read_state_elapsed);

    //If inverter is connected
    if (read_state_ok)
    {
      consecutive_failures = 0;
      reset_attempted = false;
      if (!connection)
      {
        connection = 1;
        connection_status->publish_state(CONNECTED);
      }
      turn_led_on();

      // Only read/publish one of the two groups per cycle -- full refresh of
      // any given sensor takes two 15s cycles (30s) instead of one, matching
      // Home Assistant's own reporting interval. Halves per-cycle RS-485
      // traffic and publish-burst size vs. doing all ~29 in one pass.
      const SensorEntry *group = group_toggle_ ? group_b_ : group_a_;
      size_t count = group_toggle_ ? (sizeof(group_b_) / sizeof(group_b_[0]))
                                    : (sizeof(group_a_) / sizeof(group_a_[0]));
      ESP_LOGD(TAG, "Reading group %s (%u sensors)", group_toggle_ ? "B" : "A", (unsigned)count);
      for (size_t i = 0; i < count; i++) {
        ESP_LOGD(TAG, "update: group entry %u/%u", (unsigned)(i + 1), (unsigned)count);
        publish_entry(group[i]);
      }
      ESP_LOGD(TAG, "update: group loop done, storing power_in_total");
      power_in_total = power_in_1 + power_in_2;
      group_toggle_ = !group_toggle_;

      turn_led_off();
      ESP_LOGD(TAG, "update() end (success path)");
    }
    else
    {
      ESP_LOGD(TAG, "update: ReadState failed, consecutive_failures now %d", consecutive_failures + 1);
      if (connection)
      {
        connection = 0;
        connection_status->publish_state(DISCONNECTED);
      }

      if (consecutive_failures < 255)
        consecutive_failures++;

      // Only reset once per outage -- if the inverter is simply powered down
      // (e.g. overnight), no amount of resetting our own serial port will
      // help, so don't spend hours resetting every 50s. One attempt covers
      // the case where it's actually a wedged UART driver; if the outage
      // continues past that, we just wait quietly for the next real success.
      if (consecutive_failures >= LOCKUP_THRESHOLD && !reset_attempted)
      {
        ESP_LOGW(TAG, "%d consecutive failed reads -- possible UART lockup, resetting serial", consecutive_failures);
        ESP_LOGW(TAG, "update: about to call ABBAurora::reset_serial()");
        ABBAurora::reset_serial();
        ESP_LOGW(TAG, "update: reset_serial() returned");
        reset_attempted = true;
        serial_reset_count->publish_state(serial_reset_count->get_state() + 1);
      }
      ESP_LOGD(TAG, "update() end (failure path)");
    }
  }

  void turn_led_on()
  {
    digitalWrite(LED, HIGH);
  }

  void turn_led_off()
  {
    digitalWrite(LED, LOW);
  }
};

InverterMonitor *InverterMonitor::instance_ = nullptr;

InverterMonitor *InverterMonitor::get_instance()
{
  if (instance_ == nullptr)
    instance_ = new InverterMonitor();
  return instance_;
}
