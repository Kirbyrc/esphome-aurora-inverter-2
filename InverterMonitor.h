#include "esphome.h"
#include <ABBAurora.h>

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

class InverterMonitor : public PollingComponent
{

protected:
  InverterMonitor() : PollingComponent(5000) {}
  static InverterMonitor *instance_;

private:
  ABBAurora *inverter;
  uint8_t connection = 0;
  uint8_t consecutive_failures = 0;
  bool reset_attempted = false;  // only try one serial reset per outage -- avoids
                                  // reset-spamming for hours when the inverter is
                                  // simply powered down overnight (not a lockup)
  static const uint8_t LOCKUP_THRESHOLD = 10;  // ~150s of unbroken failures at the 15s poll interval

public:
  InverterMonitor(InverterMonitor &other) = delete;
  void operator=(const InverterMonitor &) = delete;
  static InverterMonitor *get_instance();

  TextSensor *connection_status = new TextSensor();
  Sensor *serial_reset_count = new Sensor();
  Sensor *v_in_1 = new Sensor();
  Sensor *v_in_2 = new Sensor();
  Sensor *i_in_1 = new Sensor();
  Sensor *i_in_2 = new Sensor();
  Sensor *power_in_1 = new Sensor();
  Sensor *power_in_2 = new Sensor();
  Sensor *power_in_total = new Sensor();
  Sensor *power_peak_today = new Sensor();
  Sensor *power_peak_max = new Sensor();
  Sensor *temperature_inverter = new Sensor();
  Sensor *temperature_booster = new Sensor();
  Sensor *cumulated_energy_today = new Sensor();
  Sensor *cumulated_energy_week = new Sensor();
  Sensor *cumulated_energy_month = new Sensor();
  Sensor *cumulated_energy_year = new Sensor();
  Sensor *cumulated_energy_total = new Sensor();
  Sensor *grid_voltage = new Sensor();
  Sensor *grid_current = new Sensor();
  Sensor *grid_power = new Sensor();
  Sensor *frequency = new Sensor();
  Sensor *v_bulk = new Sensor();
  Sensor *i_leak_dc_dc = new Sensor();
  Sensor *i_leak_inverter = new Sensor();
  Sensor *dc_dc_grid_voltage = new Sensor();
  Sensor *dc_dc_grid_frequency = new Sensor();
  Sensor *isolation_resistance = new Sensor();
  Sensor *dc_dc_v_bulk = new Sensor();
  Sensor *average_grid_voltage = new Sensor();
  Sensor *v_bulk_mid = new Sensor();
  Sensor *grid_voltage_neutral = new Sensor();

  void setup() override
  {
    ESP_LOGD(TAG, "Setting up");
    pinMode(LED, OUTPUT);
    ABBAurora::setup(INVERTER_MONITOR_SERIAL, RX, TX, TX_CONTROL_GPIO);
    inverter = new ABBAurora(INVERTER_ADDRESS);
    connection_status->publish_state(DISCONNECTED);
    serial_reset_count->publish_state(0);
  }

  void publish_dsp_value(DSP_VALUE_TYPE type, Sensor *sensor)
  {
    uint32_t start = millis();
    bool ok = inverter->ReadDSPValue(type, MODULE_MEASUREMENT);
    uint32_t elapsed = millis() - start;
    if (ok) {
      ESP_LOGD(TAG, "ReadDSPValue %d %f (%u ms)", type, inverter->DSP.Value, elapsed);
      sensor->publish_state(inverter->DSP.Value);
    } else {
      ESP_LOGD(TAG, "ReadDSPValue %d failed (%u ms)", type, elapsed);
    }
    yield();
  }

  void publish_temperature(DSP_VALUE_TYPE type, Sensor *sensor)
  {
    uint32_t start = millis();
    bool ok = inverter->ReadDSPValue(type, MODULE_MEASUREMENT);
    uint32_t elapsed = millis() - start;
    if (ok) {
      float temp = inverter->DSP.Value / 10;
      ESP_LOGD(TAG, "ReadDSPValue %d %f (%u ms)", type, temp, elapsed);
      sensor->publish_state(temp);
    } else {
      ESP_LOGD(TAG, "ReadDSPValue %d failed (%u ms)", type, elapsed);
    }
    yield();
  }

  void publish_cumulated_energy(CUMULATED_ENERGY_TYPE type, Sensor *sensor)
  {
    uint32_t start = millis();
    bool ok = inverter->ReadCumulatedEnergy(type);
    uint32_t elapsed = millis() - start;
    if (ok) {
      ESP_LOGD(TAG, "ReadCumulatedEnergy %d %f.0 (%u ms)", type, inverter->CumulatedEnergy.Energy, elapsed);
      sensor->publish_state(inverter->CumulatedEnergy.Energy);
    } else {
      ESP_LOGD(TAG, "ReadCumulatedEnergy %d failed (%u ms)", type, elapsed);
    }
    yield();
  }

  void update() override
  {
    uint32_t read_state_start = millis();
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

      // A real delay() (not just yield()) between each publish gives the
      // WiFi/network stack's background task actual processing time to
      // drain the API's outgoing send queue, instead of every publish_state()
      // in this whole pass getting batched into one send when update()
      // finally returns to the main loop.
      publish_dsp_value(V_IN_1, v_in_1);
      delay(5);
      publish_dsp_value(V_IN_2, v_in_2);
      delay(5);
      publish_dsp_value(I_IN_1, i_in_1);
      delay(5);
      publish_dsp_value(I_IN_2, i_in_2);
      delay(5);
      publish_dsp_value(POWER_IN_1, power_in_1);
      delay(5);
      publish_dsp_value(POWER_IN_2, power_in_2);
      delay(5);
      power_in_total->publish_state(power_in_1->get_state() + power_in_2->get_state());
      delay(5);
      publish_dsp_value(POWER_PEAK_TODAY, power_peak_today);
      delay(5);
      publish_dsp_value(POWER_PEAK, power_peak_max);
      delay(5);
      publish_dsp_value(TEMPERATURE_INVERTER, temperature_inverter);
      delay(5);
      publish_dsp_value(TEMPERATURE_BOOSTER, temperature_booster);
      delay(5);
      publish_cumulated_energy(CURRENT_DAY, cumulated_energy_today);
      delay(5);
      publish_cumulated_energy(CURRENT_WEEK, cumulated_energy_week);
      delay(5);
      publish_cumulated_energy(CURRENT_MONTH, cumulated_energy_month);
      delay(5);
      publish_cumulated_energy(CURRENT_YEAR, cumulated_energy_year);
      delay(5);
      publish_cumulated_energy(TOTAL, cumulated_energy_total);
      delay(5);
      publish_dsp_value(GRID_VOLTAGE, grid_voltage);
      delay(5);
      publish_dsp_value(GRID_CURRENT, grid_current);
      delay(5);
      publish_dsp_value(GRID_POWER, grid_power);
      delay(5);
      publish_dsp_value(FREQUENCY, frequency);
      delay(5);
      publish_dsp_value(V_BULK, v_bulk);
      delay(5);
      publish_dsp_value(I_LEAK_DC_DC, i_leak_dc_dc);
      delay(5);
      publish_dsp_value(I_LEAK_INVERTER, i_leak_inverter);
      delay(5);
      publish_dsp_value(DC_DC_GRID_VOLTAGE, dc_dc_grid_voltage);
      delay(5);
      publish_dsp_value(DC_DC_GRID_FREQUENCY, dc_dc_grid_frequency);
      delay(5);
      publish_dsp_value(ISOLATION_RESISTANCE, isolation_resistance);
      delay(5);
      publish_dsp_value(DC_DC_V_BULK, dc_dc_v_bulk);
      delay(5);
      publish_dsp_value(AVERAGE_GRID_VOLTAGE, average_grid_voltage);
      delay(5);
      publish_dsp_value(V_BULK_MID, v_bulk_mid);
      delay(5);
      publish_dsp_value(GRID_VOLTAGE_NEUTRAL, grid_voltage_neutral);

      turn_led_off();
    }
    else
    {
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
        ABBAurora::reset_serial();
        reset_attempted = true;
        serial_reset_count->publish_state(serial_reset_count->get_state() + 1);
      }
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
