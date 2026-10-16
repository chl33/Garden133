// Copyright (c) 2026 Chris Lee and contributors.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <SPI.h>
#include <og3/adc_voltage.h>
#include <og3/blink_led.h>
#include <og3/config_module.h>
#include <og3/constants.h>
#include <og3/din.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/kernel_filter.h>
#include <og3/lora.h>
#include <og3/mapped_analog_sensor.h>
#include <og3/packet_writer.h>
#include <og3/satellite.h>
#include <og3/shtc3.h>
#include <og3/tasks.h>
#include <og3/units.h>
#include <og3/web_server.h>
#include <pb_encode.h>

#include <memory>
#include <vector>

#include "svelteesp32async.h"

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define STR(X) #X
#define MAKE_VERSION(MAJOR, MINOR, PATCH) STR(MAJOR) "." STR(MINOR) "." STR(PATCH)
#define VERSION MAKE_VERSION(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH)

#if HARDWARE_VERSION_MAJOR > 5
#define HAVE_STANDBY_INPUT
#endif
#if HARDWARE_VERSION_MAJOR >= 7
#define HAVE_MOISTURE_POWER_CTL
#endif
#if HARDWARE_VERSION_MAJOR >= 8
#define HAVE_AI_THINKER_LORA
#endif

// TODO
// - Use interrupt on tx-finished to sleep (didn't work so far).
// - LoRa.setTxPower(int level); (??)

namespace og3 {

static const char kManufacturer[] = "Chris Lee";
static const char kModel[] = "Garden133";
static const char kSoftware[] = "Garden133 v" VERSION;

constexpr App::LogType kLogType = OG3_LOG_METHOD;
constexpr uint32_t kCleeOrg = 0xc133;
constexpr uint16_t kDevicePktType = 0xde1c;

#ifdef HAVE_MOISTURE_POWER_CTL
constexpr uint32_t kMoistureSettleMs = 800;
#endif

constexpr size_t kLoraPacketSize = 255;

uint16_t s_board_id = 0xFF;

HAApp s_app(
    HAApp::Options(kManufacturer, kModel,
                   WifiApp::Options()
                       .withSoftwareName(kSoftware)
                       .withDefaultDeviceName(kModel)
#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
                       .withUdpLogHost(IPAddress(LOG_UDP_ADDRESS))

#endif
                       .withOta(OtaManager::Options(OTA_PASSWORD))
                       .withApp(App::Options().withLogType(kLogType).withReserveTasks(64))));

VariableGroup s_vg("garden");
VariableGroup s_cvg("garden_config");

static const char kTemperature[] = "temperature";
static const char kHumidity[] = "humidity";

// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
#ifdef HAVE_AI_THINKER_LORA
constexpr int kLoraDio1 = 2;
constexpr int kLoraBusy = 17;
#else
constexpr int kLoraDio0 = 2;
#endif

constexpr int kFiveVPin = 32;      // ADC1_CH4
constexpr int kBatteryPin = 33;    // ADC1_CH5
constexpr int kMoisturePin = 34;   // ADC1_CH6
constexpr int kSolarPlusPin = 35;  // ADC1_CH7
#ifdef HAVE_MOISTURE_POWER_CTL
// Gate control for AO3401A P-FET moisture sensor power switch
constexpr int kMoisturePowerPin = 25;
#endif
constexpr int kDebugSwitchPin = 04;
inline bool read_debug_switch() {
#if HARDWARE_VERSION_MAJOR >= 8
  // Board v8+: Switch 2 connects GPIO4 to GND with external pull-up Rv5.
  // Switch ON (closed) = GND (LOW) -> Debug Mode.
  return digitalRead(kDebugSwitchPin) == LOW;
#else
  // Board < v8: Push switch connects GPIO4 to 3.3V when activated.
  // HIGH -> Debug Mode.
  return digitalRead(kDebugSwitchPin) == HIGH;
#endif
}
constexpr int kLedPin = 16;
#if defined(HAVE_STANDBY_INPUT)
// NOTE: it looks from the circuit diagram that these pins should be reversed, but
//  these are the pin assignments that work in testing.
constexpr int kChrgPin = 27;
constexpr int kStbyPin = 26;
#endif

// Default sleep for 5 minute between readings (for now).
constexpr unsigned kDefaultSleepMin = 5;

// Number of samples to store for the moisture filter.
constexpr unsigned kFilterSize = 32;

namespace Status {
constexpr unsigned kDebugMode = 0x1;
constexpr unsigned kPowerOn = 0x2;
constexpr unsigned kDeepSleep = 0x4;
constexpr unsigned kFilterRestoreFailure = 0x8;
constexpr unsigned kFilterSaveFailure = 0x10;
}  // namespace Status

constexpr unsigned kBlinkMsec = 100;

// This data should persist during deep sleep.
// DO NOT SET INITIALIZERS ON THESE VALUES!
RTC_DATA_ATTR struct {
  unsigned bootCount;
  byte mac[6];
  KernelFilter::State<kFilterSize> filter_state;
  unsigned code;
  satellite::PacketSender::Rtc packet_sender;
  unsigned last_wake_secs;
  unsigned expected_wake_secs;
} s_rtc;

bool s_boot = false;
bool s_lora_ok = false;
bool s_is_debug_mode = false;

BlinkLed s_led("mode_led", kLedPin, &s_app, kBlinkMsec, false);

Shtc3 s_shtc3(kTemperature, kHumidity, &s_app.module_system(), "temperature", s_vg);

// To get to 1% radio utilization, should transmit every 4 minutes maximum.
// Assuming a 86 byte packet.
// 86 bytes / (288 bits/s / 8 bits/byte) = 2.4 seconds to transmit
// So cannot transmit more often than everty (100 * 2.4 seconds) / (60 sec / min) = 4 minutes

auto s_lora_options = []() -> LoRaModule::Options {
  LoRaModule::Options opts;
  opts.gpio_ss = kLoraSS;
  opts.gpio_rst = kLoraRst;
#ifdef HAVE_AI_THINKER_LORA
  opts.module_type = LoRaModule::Options::ModuleType::kSX1262;
  opts.gpio_dio1 = kLoraDio1;
  opts.gpio_busy = kLoraBusy;
  opts.tcxo_voltage = 0.0f;  // Ai-Thinker Ra-01SH uses standard crystal oscillator (XTAL)
#else
  opts.gpio_dio0 = kLoraDio0;
#endif
  opts.sync_word = 0xF0;
  opts.enable_crc = true;
  // opts.on_transmit_done = []() { s_led.off(); };

  opts.frequency = lora::Frequency::k915MHz;
  opts.spreading_factor = lora::SpreadingFactor::kSF8;
  opts.signal_bandwidth = lora::SignalBandwidth::k125kHz;

  const auto opt_select = static_cast<LoRaModule::OptionSelect>(LoRaModule::kOptionSyncWord |
                                                                LoRaModule::kOptionSpreadingFactor |
                                                                LoRaModule::kOptionSignalBandwidth);
  opts.config_options = opt_select;
  opts.settable_options = opt_select;

  return opts;
};

VariableGroup s_lora_vg("lora");
LoRaModule s_lora(s_lora_options(), &s_app, s_lora_vg, nullptr);

Variable<unsigned> s_sleep_min("sleepMin", kDefaultSleepMin, units::kMinutes, "sleep min",
                               VariableBase::kSettable | VariableBase::kConfig, s_lora_vg);

template <typename T>
void copy(T& dest, const T& src) {
  memcpy(&dest, &src, sizeof(dest));
}

void add_html_button(String* body, const char* title, const char* url) {
  *body += F("<p><form action='");
  *body += url;
  *body += F("' method='get'><button>");
  *body += title;
  *body += F("</button></form></p>\n");
}

// Global variable for html, so asyncwebserver can send data in the background (single client)
String s_html;

class MoistureSensor : public ConfigModule {
 public:
  MoistureSensor()
      : ConfigModule("moisture", &s_app),
        m_mapped_adc(
            {
                .name = "moisture",
                .pin = kMoisturePin,
                .units = units::kPercentage,
                .raw_description = "soil moisture raw value",
                .description = "soil moisture",
                .raw_var_flags = 0,
                .mapped_var_flags = 0,
                .config_flags = kCfgSet,
                .default_in_min = kNoMoistureCounts,
                .default_in_max = kFullMoistureCounts,
                .default_out_min = 0.0f,
                .default_out_max = 100.0f,
                .config_decimals = 0,
                .decimals = 1,
                .valid_in_min = 50,
                .valid_in_max = 1 << 12,
            },
            &s_app.module_system(), m_cvg, s_vg) {}
  float read() { return m_mapped_adc.read(); }
  float value() const { return m_mapped_adc.value(); }
  int raw_counts() const { return m_mapped_adc.raw_counts(); }
  Variable<unsigned>& countsVar() { return m_mapped_adc.raw_value(); }
  VariableGroup& configVarGroup() { return m_cvg; }

 private:
  // Multiple constants by 2/3 compared to Plant133 due to voltage divider.
  // Default ADC reading at which to consider soil moisture to be 100%.
  static constexpr unsigned kFullMoistureCounts = 1100;
  // Default ADC reading at which to consider soil moisture to be 0%.
  static constexpr unsigned kNoMoistureCounts = 3300;

  static constexpr unsigned kCfgSet = VariableBase::Flags::kConfig | VariableBase::Flags::kSettable;

  MappedAnalogSensor m_mapped_adc;
};

constexpr float kSoilSensorSigmaSecs = 30 * kSecInMin;

MoistureSensor s_moisture;
KernelFilter s_moisture_filter(
    {
        .name = "soil moisture",
        .units = units::kPercentage,
        .description = nullptr,
        .var_flags = 0,
        .sigma = kSoilSensorSigmaSecs,
        .decimals = 1,
        .size = kFilterSize,
    },
    &s_app.module_system(), s_vg);

int64_t total_usecs() {
  const int64_t wake_sleep_usecs = static_cast<int64_t>(s_rtc.expected_wake_secs) * kUsecInSec;
  return esp_timer_get_time() + wake_sleep_usecs;
}

AdcVoltage s_five_v_sensor("fivevVoltage", &s_app, kFiveVPin, "fivev raw value",
                           "voltage from battery or solar", 3.3 * 2, s_vg, s_cvg);

AdcVoltage s_battery_sensor("batteryVoltage", &s_app, kBatteryPin, "battery raw value", nullptr,
                            3.3 * 32.0 / 22.0, s_vg, s_cvg);

AdcVoltage s_solar_sensor("solarVoltage", &s_app, kSolarPlusPin, "solar raw value", nullptr,
                          3.3 * 2, s_vg, s_cvg);

Variable<unsigned> s_status_var("status", 0, nullptr, "status flags", 0, s_vg);
Variable<unsigned> s_board_id_var("board id", 0, nullptr, nullptr, 0, s_vg);
Variable<unsigned> s_wake_secs("wake_secs", s_rtc.expected_wake_secs, units::kSeconds, nullptr, 0,
                               s_vg);
Variable<unsigned> s_last_wake_secs("last_wake_secs", s_rtc.last_wake_secs, units::kSeconds,
                                    nullptr, 0, s_vg);
#if defined(HAVE_STANDBY_INPUT)
constexpr bool kDinPublish = true;
constexpr bool kDinInvert = false;
DIn s_din_chrg("chrg", &s_app.module_system(), kChrgPin, nullptr, s_vg, kDinPublish, kDinInvert);
DIn s_din_stby("stby", &s_app.module_system(), kStbyPin, nullptr, s_vg, kDinPublish, kDinInvert);
Variable<unsigned> s_var_charge("charge", 0, nullptr, nullptr, VariableBase::Flags::kNoPublish,
                                s_vg);
Variable<unsigned> s_var_standby("standby", 0, nullptr, nullptr, VariableBase::Flags::kNoPublish,
                                 s_vg);
#endif

#define SETSTR(X, VAL) strncpy(X, (VAL), sizeof(X) - 1)

class PacketTemperatureReading : public satellite::PacketFloatReading {
 public:
  PacketTemperatureReading(unsigned sensor_id, Shtc3& shtc3)
      : PacketFloatReading(sensor_id, og3_Sensor_Type_TYPE_TEMPERATURE, shtc3.temperatureVar()),
        m_shtc3(shtc3) {}

  bool read() final {
    m_shtc3.read();
    return m_shtc3.ok();
  }
  bool write(og3_Packet& packet) final { return PacketFloatReading::write(packet); }

 private:
  Shtc3& m_shtc3;
};

class PacketHumidityReading : public satellite::PacketFloatReading {
 public:
  PacketHumidityReading(unsigned sensor_id, Shtc3& shtc3)
      : PacketFloatReading(sensor_id, og3_Sensor_Type_TYPE_HUMIDITY, shtc3.humidityVar()),
        m_shtc3(shtc3) {}

  bool read() final {
    // Assume read() happened in PacketTemperatureReading
    return m_shtc3.ok();
  }
  bool write(og3_Packet& packet) final {
    if (!m_shtc3.ok()) {
      return false;
    }
    return PacketFloatReading::write(packet);
  }

 private:
  Shtc3& m_shtc3;
};

class PacketMoistureReading : public satellite::PacketFloatReading {
 public:
  static constexpr float kMinVForValidReading = 3.2f;

  PacketMoistureReading(unsigned sensor_id, MoistureSensor& moisture, KernelFilter& moisture_filter)
      : PacketFloatReading(sensor_id, og3_Sensor_Type_TYPE_MOISTURE,
                           moisture_filter.valueVariable(),
                           og3_Sensor_StateClass_STATE_CLASS_MEASUREMENT),
        m_moisture(moisture),
        m_moisture_filter(moisture_filter) {}

  bool read() {
    const int64_t now_usecs = total_usecs();
    const float reading = m_moisture.read();
    // When the power supply is < 3.2V, it seems moisture sensor readings are not trustworthy.
    m_is_ok = s_five_v_sensor.read() > kMinVForValidReading;
    if (m_is_ok) {
      m_moisture_filter.addSample(now_usecs * 1e-6, reading);
    }
    return true;
  }
  bool write(og3_Packet& packet) final {
    if (!m_is_ok) {
      return false;
    }
    return PacketFloatReading::write(packet);
  }

 private:
  MoistureSensor& m_moisture;
  KernelFilter& m_moisture_filter;
  bool m_is_ok = false;
};

class LoraPacketSender : public satellite::PacketSender {
 public:
  LoraPacketSender(og3_Device* device, og3::App* app, og3::satellite::PacketSender::Rtc* rtc)
      : satellite::PacketSender(device, app, rtc), m_mutable_device(device) {
    auto idx = [this]() { return m_readings.size(); };
    m_readings.reserve(10);
    m_readings.emplace_back(new PacketTemperatureReading(idx(), s_shtc3));
    m_readings.emplace_back(new PacketHumidityReading(idx(), s_shtc3));
    m_readings.emplace_back(new PacketMoistureReading(idx(), s_moisture, s_moisture_filter));
    m_readings.emplace_back(new satellite::PacketVoltageReading(idx(), s_five_v_sensor));
    m_readings.emplace_back(new satellite::PacketVoltageReading(idx(), s_battery_sensor));
    m_readings.emplace_back(new satellite::PacketVoltageReading(idx(), s_solar_sensor));
    m_readings.emplace_back(
        new satellite::PacketIntReading(idx(), "soil ADC counts", s_moisture.countsVar()));
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "status", s_status_var));
#if defined(HAVE_STANDBY_INPUT)
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "charge", s_var_charge));
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "standby", s_var_standby));
#else
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "wake time", s_wake_secs));
    m_readings.emplace_back(
        new satellite::PacketIntReading(idx(), "last wake time", s_last_wake_secs));
#endif
  }

  void update() {
    SETSTR(m_mutable_device->name, s_app.board_cname());
    const int64_t now_usecs = total_usecs();
    const int64_t now_secs = now_usecs / kUsecInSec;
    const int64_t secs_since_info_sent = now_secs - m_rtc->secs_device_sent;
    s_app.log().debugf("now_secs:%llu secs_since_info_sent:%llu", now_secs, secs_since_info_sent);

    const bool need_to_send_info = s_boot || (secs_since_info_sent >= 3600);
    if (need_to_send_info) {
      m_rtc->secs_device_sent = now_secs;
      m_rtc->sensor_descriptions_sent = 0;
    }
    s_status_var = s_rtc.code;
    s_board_id_var = s_board_id;
#if defined(HAVE_STANDBY_INPUT)
    s_var_charge = s_din_chrg.read() ? 1 : 0;
    s_var_standby = s_din_stby.read() ? 1 : 0;
#endif

    if (s_boot) {
      // At boot, for each sensor send one packet describing the sensor.
      const int max_packet = s_lora.usa_max_payload();
      constexpr int packet_overhead = pkt::kHeaderSize + pkt::kMsgHeaderSize + sizeof(uint32_t);
      const int max_payload = max_packet - packet_overhead;
      if (max_payload < 8) {
        s_app.log().logf("Max packet %d - overhead %d is too small", max_packet, packet_overhead);
        return;
      }
      send_desc(max_payload);
    } else {
      send_all_readings();
    }
  }

 protected:
  void send_packet(og3_Packet& packet) override {
    uint8_t msg_buffer[kLoraPacketSize];
    pb_ostream_t ostream = pb_ostream_from_buffer(msg_buffer, sizeof(msg_buffer));
    if (!pb_encode(&ostream, &og3_Packet_msg, &packet)) {
      s_app.log().log("Failed to pb_encode packet.");
      return;
    }

    uint8_t pkt_buffer[og3_Packet_size + pkt::kHeaderSize + pkt::kMsgHeaderSize];
    pkt::PacketWriter writer(pkt_buffer, sizeof(pkt_buffer), m_rtc->seq_id++);
    if (!writer.add_message(kDevicePktType, msg_buffer, ostream.bytes_written)) {
      s_app.log().log("Failed to add device msg to packet.");
      return;
    }
    if (!writer.add_crc()) {
      s_app.log().log("Failed to add CRC to packet.");
      return;
    }

    // s_led.on();
    s_lora.send_packet(pkt_buffer, writer.packet_size());
    s_app.log().debugf("Sent LoRa packet (seq_id:%u, %zu bytes).", m_rtc->seq_id - 1,
                       ostream.bytes_written);
  }

  // Non-const pointer so we can update the name.
  og3_Device* m_mutable_device;
};

og3_Device* s_device() {
  static og3_Device ret;

  ret.id = s_board_id;  // may not be set yet
  ret.manufacturer = kCleeOrg;
  SETSTR(ret.name, s_app.board_cname());
  SETSTR(ret.device_type, kModel);
  ret.hardware_version.major = HARDWARE_VERSION_MAJOR;
  ret.hardware_version.minor = HARDWARE_VERSION_MINOR;
  ret.software_version.major = VERSION_MAJOR;
  ret.software_version.minor = VERSION_MINOR;
  ret.software_version.patch = VERSION_PATCH;
  ret.timeout_secs = s_sleep_min.value() * 60 * 3 + 10;
  return &ret;
}

LoraPacketSender s_packet_sender(s_device(), &s_app, &s_rtc.packet_sender);

std::unique_ptr<PeriodicTaskScheduler> s_hourly_lora;

bool print_wakeup_reason() {
  const esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      s_app.log().log("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      s_app.log().log("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      s_app.log().log("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      s_app.log().log("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      s_app.log().log("Wakeup caused by ULP program");
      break;
    default:
      s_app.log().logf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      return true;
  }
  return false;
}

WebButton s_button_wifi_config = s_app.createWifiConfigButton();
WebButton s_button_mqtt_config = s_app.createMqttConfigButton();
WebButton s_button_app_status = s_app.createAppStatusButton();
WebButton s_button_restart = s_app.createRestartButton();

NetHandlerStatus handleWebRoot(NetRequest* request, NetResponse* response) {
  const int64_t now_usecs = total_usecs();

  // Read the sensors.
  s_shtc3.read();
  s_moisture_filter.addSample(now_usecs * 1e-6, s_moisture.read());
  s_five_v_sensor.read();
  s_battery_sensor.read();
  s_solar_sensor.read();
#if defined(HAVE_STANDBY_INPUT)
  s_var_charge = s_din_chrg.read() ? 1 : 0;
  s_var_standby = s_din_stby.read() ? 1 : 0;
#endif

  // Write the page.
  s_html.clear();
  html::writeTableInto(&s_html, s_vg);
  html::writeTableInto(&s_html, s_app.wifi_manager().variables());
  html::writeTableInto(&s_html, s_app.mqtt_manager().variables());

  s_moisture.add_html_button(&s_html);
  s_five_v_sensor.add_html_button(&s_html);
  s_battery_sensor.add_html_button(&s_html);
  s_solar_sensor.add_html_button(&s_html);

  s_html += HTML_BUTTON("/lora", "LoRa");
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, response, s_app.board_cname(), kSoftware, s_html.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus handleLoraConfig(NetRequest* request, NetResponse* response) {
  read(*request, s_lora_vg);
  s_html.clear();
  html::writeFormTableInto(&s_html, s_lora_vg);
  s_html += HTML_BUTTON("/", "Back");
  sendWrappedHTML(request, response, s_app.board_cname(), kSoftware, s_html.c_str());
  s_app.config().write_config(s_lora_vg);
  NET_REPLY(request, ESP_OK);
}

void start_sleep() {
  s_app.log().debug("Sleeping");
  s_rtc.code &= ~Status::kDebugMode;

#ifdef HAVE_MOISTURE_POWER_CTL
  digitalWrite(kMoisturePowerPin, HIGH);  // Ensure sensor power is OFF during sleep
#endif

  // Save filter state to memory that is preserved when sleeping.
  if (!s_moisture_filter.saveState(s_rtc.filter_state)) {
    s_rtc.code |= Status::kFilterSaveFailure;
  }

  og3::s_lora.sleep();

  // Set a timer which will wakeup the board, then put the board into deep sleep.
  s_rtc.last_wake_secs = s_rtc.expected_wake_secs;
  s_rtc.expected_wake_secs = s_rtc.last_wake_secs + kSecInMin * s_sleep_min.value();

  esp_sleep_enable_timer_wakeup((kSecInMin * s_sleep_min.value()) * kUsecInSec -
                                esp_timer_get_time());
  esp_deep_sleep_start();
}

static String s_body;

NetHandlerStatus apiGetWifi(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_app.wifi_manager().variables().toJson(json, VariableBase::kConfig);
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putWifiConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_app.wifi_manager().variables().updateFromJson(obj);
  s_app.config().write_config(s_app.wifi_manager().variables());
  response->send(200, "text/plain", "ok");
  s_app.tasks().runIn(1000, []() { ESP.restart(); });
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetMqtt(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_app.mqtt_manager().variables().toJson(json, VariableBase::kConfig);
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putMqttConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_app.mqtt_manager().variables().updateFromJson(obj);
  s_app.config().write_config(s_app.mqtt_manager().variables());
  if (s_app.mqtt_manager().isEnabled() && !s_app.mqtt_manager().isConnected()) {
    s_app.mqtt_manager().connect();
  } else if (!s_app.mqtt_manager().isEnabled() && s_app.mqtt_manager().isConnected()) {
    s_app.mqtt_manager().disconnect();
  }
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetStatus(NetRequest* request, NetResponse* response) {
  const int64_t now_usecs = total_usecs();
  s_shtc3.read();
  s_moisture_filter.addSample(now_usecs * 1e-6, s_moisture.read());
  s_five_v_sensor.read();
  s_battery_sensor.read();
  s_solar_sensor.read();

  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  json["mqttConnected"] = s_app.mqtt_manager().isConnected();
  json["software"] = VERSION;
  json["hardware"] = "Garden133";
  json["bootCount"] = s_rtc.bootCount;
  json["uptime"] = millis() / 1000;

  JsonObject status = json["status"].to<JsonObject>();
  status["temperature"] = s_shtc3.temperature();
  status["humidity"] = s_shtc3.humidity();
  status["moisture"] = s_moisture.value();
  status["moistureFilt"] = s_moisture_filter.value();
  status["moistureRaw"] = s_moisture.raw_counts();
  status["fivevVoltage"] = s_five_v_sensor.value();
  status["batteryVoltage"] = s_battery_sensor.value();
  status["solarVoltage"] = s_solar_sensor.value();
#if defined(HAVE_STANDBY_INPUT)
  status["charging"] = s_din_chrg.read();
  status["standby"] = s_din_stby.read();
#endif

  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetConfig(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_cvg.toJson(json, VariableBase::kConfig);
  s_lora_vg.toJson(json, VariableBase::kConfig);
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus apiGetMoistureConfig(NetRequest* request, NetResponse* response) {
  JsonDocument jsondoc;
  JsonObject json = jsondoc.to<JsonObject>();
  s_moisture.configVarGroup().toJson(json, VariableBase::kConfig);
  s_body.clear();
  serializeJson(jsondoc, s_body);
  response->send(200, "application/json", s_body.c_str());
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putConfig(NetRequest* request, NetResponse* response, JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_cvg.updateFromJson(obj);
  s_app.config().write_config(s_cvg);
  s_lora_vg.updateFromJson(obj);
  s_app.config().write_config(s_lora_vg);
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

NetHandlerStatus putMoistureConfig(NetRequest* request, NetResponse* response,
                                   JsonVariant& jsonIn) {
  if (!jsonIn.is<JsonObject>()) {
    response->send(500, "text/plain", "not a json object");
    NET_REPLY(request, ESP_FAIL);
  }
  JsonObject obj = jsonIn.as<JsonObject>();
  s_moisture.configVarGroup().updateFromJson(obj);
  s_app.config().write_config(s_moisture.configVarGroup());
  response->send(200, "text/plain", "ok");
  NET_REPLY(request, ESP_OK);
}

}  // namespace og3

// #define RUN_LORA_PIN_TEST 1

#ifdef RUN_LORA_PIN_TEST
static uint8_t spi_bb_transfer(uint8_t out_byte, int delay_us = 10) {
  uint8_t in_byte = 0;
  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(23, (out_byte & (1 << bit)) ? HIGH : LOW);  // MOSI
    delayMicroseconds(delay_us);
    digitalWrite(18, HIGH);                                  // SCK rising edge
    delayMicroseconds(delay_us);
    if (digitalRead(19)) {                                   // Sample MISO
      in_byte |= (1 << bit);
    }
    digitalWrite(18, LOW);  // SCK falling edge
    delayMicroseconds(delay_us);
  }
  return in_byte;
}

void run_lora_diagnostics() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\n==================================================");
  Serial.println("     SX1262 / Ra-01SH DEEP HARDWARE DIAGNOSTIC    ");
  Serial.println("==================================================");

  while (true) {
    // --- Test 1: BUSY pin pull-up / pull-down float check ---
    Serial.println("\n--- [Test 1] BUSY Pin (Pin 10 / GPIO17) Connection Check ---");
    pinMode(17, INPUT_PULLUP);
    delay(2);
    int busy_pu = digitalRead(17);
    pinMode(17, INPUT_PULLDOWN);
    delay(2);
    int busy_pd = digitalRead(17);
    pinMode(17, INPUT);
    Serial.printf("BUSY read with internal PULLUP: %d | with PULLDOWN: %d\n", busy_pu, busy_pd);
    if (busy_pu == 1 && busy_pd == 0) {
      Serial.println(
          "  -> ALERT: BUSY pin is FLOATING! (Pin 10 is not connected to module, cold solder "
          "joint)");
    } else if (busy_pu == 0 && busy_pd == 0) {
      Serial.println("  -> OK: BUSY is firmly held LOW (0V) by the SX1262 or GND.");
    } else {
      Serial.println("  -> BUSY is firmly held HIGH (3.3V).");
    }

    // --- Test 2: RESET Pulse & BUSY Boot Activity ---
    Serial.println("\n--- [Test 2] RESET Pulse & SX1262 Boot Monitoring ---");
    pinMode(14, OUTPUT);
    digitalWrite(14, LOW);  // Assert RESET
    delay(10);
    int busy_reset = digitalRead(17);
    Serial.printf("BUSY while RESET is LOW: %d\n", busy_reset);

    // Release RESET and monitor BUSY in microsecond resolution
    uint32_t t0 = micros();
    digitalWrite(14, HIGH);  // Release RESET
    int busy_went_high = 0;
    uint32_t high_start_us = 0;
    uint32_t high_duration_us = 0;
    while (micros() - t0 < 35000) {  // Monitor for 35 ms
      int b = digitalRead(17);
      if (b == 1 && !busy_went_high) {
        busy_went_high = 1;
        high_start_us = micros() - t0;
      } else if (b == 0 && busy_went_high && high_duration_us == 0) {
        high_duration_us = (micros() - t0) - high_start_us;
      }
    }
    if (busy_went_high) {
      Serial.printf(
          "  -> SUCCESS: BUSY went HIGH at +%u us, stayed HIGH for %u us, then dropped LOW!\n",
          high_start_us, high_duration_us);
      Serial.println("     THE SX1262 CHIP IS ALIVE AND EXECUTING ITS BOOTLOADER!");
    } else {
      Serial.println("  -> WARNING: BUSY NEVER went HIGH after reset! (Stayed 0V the entire 35ms)");
      Serial.println(
          "     Possible causes: Module has no power (Pin 3), Pin 10 shorted to Pin 9 (GND), or "
          "module dead.");
    }

    // --- Test 3: Bit-Bang SPI (50 kHz slow clock) ---
    Serial.println("\n--- [Test 3] Bit-Bang SPI Communication (50 kHz) ---");
    pinMode(5, OUTPUT);   // NSS
    digitalWrite(5, HIGH);
    pinMode(18, OUTPUT);  // SCK
    digitalWrite(18, LOW);
    pinMode(23, OUTPUT);  // MOSI
    digitalWrite(23, LOW);
    pinMode(19, INPUT);   // MISO
    delay(5);

    int miso_idle = digitalRead(19);
    Serial.printf("MISO with NSS HIGH (should be 1 via Ru2 pull-up): %d\n", miso_idle);

    digitalWrite(5, LOW);  // NSS LOW
    delayMicroseconds(10);
    int miso_nss_low = digitalRead(19);
    Serial.printf("MISO immediately after NSS LOW (SX1262 status bit 7, should be 0): %d\n",
                  miso_nss_low);
    Serial.println("  [Probe Window] Holding NSS LOW for 3 seconds:");
    for (int s = 1; s <= 3; s++) {
      Serial.printf("    t=%ds: NSS pin is driven LOW (0V). MISO reads: %d\n", s, digitalRead(19));
      delay(1000);
    }

    // Send GetStatus (0xC0 0x00)
    uint8_t bb_status = spi_bb_transfer(0xC0);
    uint8_t bb_dummy = spi_bb_transfer(0x00);
    digitalWrite(5, HIGH);  // NSS HIGH
    delayMicroseconds(20);

    Serial.printf("Bit-Bang GetStatus (0xC0 0x00) -> Status: 0x%02X, Dummy: 0x%02X\n", bb_status,
                  bb_dummy);
    if (bb_status != 0xFF) {
      Serial.printf("  -> SUCCESS! Chip responded over SPI: Status = 0x%02X\n", bb_status);
    } else {
      Serial.println("  -> Failed: Status returned 0xFF (no response on MISO).");
    }

    // Try reading register 0x0320 (version string)
    digitalWrite(5, LOW);
    delayMicroseconds(10);
    spi_bb_transfer(0x1D);  // ReadRegister
    spi_bb_transfer(0x03);  // Reg MSB
    spi_bb_transfer(0x20);  // Reg LSB
    spi_bb_transfer(0x00);  // Status dummy
    char ver_bb[8] = {0};
    for (int i = 0; i < 7; i++) {
      ver_bb[i] = (char)spi_bb_transfer(0x00);
    }
    digitalWrite(5, HIGH);
    Serial.printf("Bit-Bang Read Reg 0x0320: \"%s\" (Hex: %02X %02X %02X %02X %02X %02X %02X)\n",
                  ver_bb, (uint8_t)ver_bb[0], (uint8_t)ver_bb[1], (uint8_t)ver_bb[2],
                  (uint8_t)ver_bb[3], (uint8_t)ver_bb[4], (uint8_t)ver_bb[5], (uint8_t)ver_bb[6]);

    // --- Test 4: Hardware SPI (1 MHz) ---
    Serial.println("\n--- [Test 4] Hardware SPI Communication (1 MHz) ---");
    SPI.begin(18, 19, 23, -1);
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    delay(5);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(5, LOW);
    delayMicroseconds(10);
    uint8_t hw_status = SPI.transfer(0xC0);
    uint8_t hw_dummy = SPI.transfer(0x00);
    digitalWrite(5, HIGH);
    SPI.endTransaction();
    Serial.printf("Hardware SPI GetStatus -> Status: 0x%02X, Dummy: 0x%02X\n", hw_status, hw_dummy);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(5, LOW);
    delayMicroseconds(10);
    SPI.transfer(0x1D);  // ReadRegister
    SPI.transfer(0x03);  // MSB
    SPI.transfer(0x20);  // LSB
    SPI.transfer(0x00);  // Status
    char ver_hw[8] = {0};
    for (int i = 0; i < 7; i++) {
      ver_hw[i] = (char)SPI.transfer(0x00);
    }
    digitalWrite(5, HIGH);
    SPI.endTransaction();
    SPI.end();
    Serial.printf(
        "Hardware SPI Read Reg 0x0320: \"%s\" (Hex: %02X %02X %02X %02X %02X %02X %02X)\n", ver_hw,
        (uint8_t)ver_hw[0], (uint8_t)ver_hw[1], (uint8_t)ver_hw[2], (uint8_t)ver_hw[3],
        (uint8_t)ver_hw[4], (uint8_t)ver_hw[5], (uint8_t)ver_hw[6]);

    Serial.println("\n==================================================");
    Serial.println("  Cycle complete. Retesting in 5 seconds...       ");
    Serial.println("==================================================");
    delay(5000);
  }
}
#endif

void setup() {
#ifdef RUN_LORA_PIN_TEST
  run_lora_diagnostics();
#endif

#if HARDWARE_VERSION_MAJOR >= 8
  pinMode(og3::kDebugSwitchPin, INPUT_PULLUP);
#else
  pinMode(og3::kDebugSwitchPin, INPUT);
#endif
  og3::s_is_debug_mode = og3::read_debug_switch();

  if (og3::s_is_debug_mode) {
    og3::s_rtc.code |= og3::Status::kDebugMode;
  } else {
    og3::s_rtc.code = 0;
  }

  if (!og3::s_is_debug_mode) {
    // This is the normal operation: wake from sleep, send a packet with LoRa, go back to sleep.
    og3::s_app.wifi_manager().set_enable(false);
    if (og3::print_wakeup_reason()) {
      // Normal operation, but first-time boot, not waking from deep sleep.
      og3::s_boot = true;
      memset(&og3::s_rtc, 0, sizeof(og3::s_rtc));
      WiFi.macAddress(og3::s_rtc.mac);
      og3::s_rtc.code |= og3::Status::kPowerOn;
      og3::s_app.log().debug("First wake, pausing sleep");
      og3::s_packet_sender.set_is_sending(true);
    } else {
      // Normal operation, woke from deep sleep.
      og3::s_boot = false;
      if (!og3::s_moisture_filter.restoreState(og3::s_rtc.filter_state)) {
        og3::s_rtc.code |= og3::Status::kFilterRestoreFailure;
      }
      og3::s_rtc.bootCount += 1;
      og3::s_rtc.code |= og3::Status::kDeepSleep;
      og3::s_app.log().debugf("bootCount %u: code:0x%X", og3::s_rtc.bootCount, og3::s_rtc.code);
    }
    // build a board-id from the mac address.
  } else {
    // This is debug mode.
    og3::s_boot = true;
    constexpr auto kInitialWait = 10 * og3::kMsecInSec;
    constexpr auto kUpdatePeriod = og3::kMsecInMin;
    WiFi.macAddress(og3::s_rtc.mac);
    og3::s_hourly_lora.reset(new og3::PeriodicTaskScheduler(
        kInitialWait, kUpdatePeriod, []() { og3::s_packet_sender.update(); }, &og3::s_app.tasks()));
    // Blink twice on boot if debug mode.
    og3::s_led.blink(2);
  }

#ifdef HAVE_MOISTURE_POWER_CTL
  pinMode(og3::kMoisturePowerPin, OUTPUT);
  if (og3::s_is_debug_mode || !og3::s_boot) {
    // - In Debug Mode: keep power continuously ON.
    // - When waking from deep sleep: turn power ON immediately at boot so settling overlaps with
    //    setup.
    digitalWrite(og3::kMoisturePowerPin, LOW);
  } else {
    // - On cold boot in Normal Mode: keep power OFF while sending sensor descriptions.
    digitalWrite(og3::kMoisturePowerPin, HIGH);
  }
#endif
  // Constuct a board ID from the MAC address.
  og3::s_board_id = (og3::s_rtc.mac[3] << 8) | (og3::s_rtc.mac[4] ^ og3::s_rtc.mac[5]);
  og3::s_packet_sender.set_board_id(og3::s_board_id);

  initSvelteStaticFiles(&og3::s_app.web_server_module().native_server());
  og3::s_app.web_server_module().on("/api/wifi", HTTP_GET, og3::apiGetWifi);
  og3::s_app.web_server_module().on("/api/mqtt", HTTP_GET, og3::apiGetMqtt);
  og3::s_app.web_server_module().on("/api/status", HTTP_GET, og3::apiGetStatus);
  og3::s_app.web_server_module().on("/api/config", HTTP_GET, og3::apiGetConfig);
  og3::s_app.web_server_module().on("/api/moisture/config", HTTP_GET, og3::apiGetMoistureConfig);

  og3::s_app.web_server_module().onJson("/api/wifi", HTTP_PUT, og3::putWifiConfig);
  og3::s_app.web_server_module().onJson("/api/mqtt", HTTP_PUT, og3::putMqttConfig);
  og3::s_app.web_server_module().onJson("/api/config", HTTP_PUT, og3::putConfig);
  og3::s_app.web_server_module().onJson("/api/moisture/config", HTTP_PUT, og3::putMoistureConfig);

  og3::s_app.web_server_module().on("/api/restart", HTTP_POST,
                                    [](og3::NetRequest* request, og3::NetResponse* response) {
                                      response->send(200, "text/plain", "restarting");
                                      og3::s_app.tasks().runIn(1000, []() { ESP.restart(); });
                                      NET_REPLY(request, ESP_OK);
                                    });

  og3::s_app.web_server_module().on("/old", HTTP_GET, og3::handleWebRoot);
  og3::s_app.web_server_module().on("/old", HTTP_POST, og3::handleWebRoot);
  og3::s_app.web_server_module().on("/old_lora", HTTP_GET, og3::handleLoraConfig);
  og3::s_app.web_server_module().on("/old_lora", HTTP_POST, og3::handleLoraConfig);
  og3::s_app.setup();
}

void loop() {
  og3::s_app.loop();

  // Check the debug-mode toggle button again, so we can switch away from debug mode
  //  while the board is running.
  og3::s_is_debug_mode = og3::read_debug_switch();

  if (og3::s_is_debug_mode) {
#ifdef HAVE_MOISTURE_POWER_CTL
    // Ensure moisture sensor remains powered ON in debug mode
    digitalWrite(og3::kMoisturePowerPin, LOW);
#endif
    return;  // In debug mode, all the work happens in og3::s_app.loop().
  }

  // --- Normal Mode (wake from sleep, send LoRa packet, deep sleep) ---

  // Safety timeout: If LoRa fails to initialize within 10 seconds, go to sleep.
  if (!og3::s_lora.is_ok()) {
    if (millis() < og3::kMsecInSec * 10) {
      return;
    }
    og3::s_app.log().log("LoRa startup timed out, sleeping.");
#ifdef HAVE_MOISTURE_POWER_CTL
    digitalWrite(og3::kMoisturePowerPin, HIGH);
#endif
    og3::start_sleep();
    return;
  }

  // State machine for normal mode
  enum class NormalState {
    kInitBootDescriptions,
    kWaitingForDescriptions,
    kWaitingBetweenDescAndReadings,
    kWaitingToSampleAndSend,
    kWaitingForSleep,
  };
  static NormalState s_normal_state =
      og3::s_boot ? NormalState::kInitBootDescriptions : NormalState::kWaitingToSampleAndSend;
  static uint32_t s_desc_finish_ms = 0;

  switch (s_normal_state) {
    case NormalState::kInitBootDescriptions:
      // Start sending description packets (every 15s)
      og3::s_packet_sender.update();
      s_normal_state = NormalState::kWaitingForDescriptions;
      break;

    case NormalState::kWaitingForDescriptions:
      // Wait while description packets are still being sent (every 15s at boot)
      if (og3::s_packet_sender.is_sending()) {
        return;
      }
      // All description packets have finished sending.
      // Record timestamp to wait 15 seconds before the first sensor reading.
      s_desc_finish_ms = millis();
      s_normal_state = NormalState::kWaitingBetweenDescAndReadings;
      break;

    case NormalState::kWaitingBetweenDescAndReadings: {
      constexpr uint32_t kDescToReadingWaitMs = 15 * og3::kMsecInSec;
      const uint32_t elapsed = millis() - s_desc_finish_ms;
#ifdef HAVE_MOISTURE_POWER_CTL
      if (elapsed >= kDescToReadingWaitMs - og3::kMoistureSettleMs) {
        // Power ON moisture sensor 500ms before reading so it has settled
        digitalWrite(og3::kMoisturePowerPin, LOW);
      }
#endif
      if (elapsed < kDescToReadingWaitMs) {
        return;
      }
      // 15 seconds have elapsed and moisture sensor is settled; take and send readings
      og3::s_boot = false;
      og3::s_packet_sender.update();
#ifdef HAVE_MOISTURE_POWER_CTL
      digitalWrite(og3::kMoisturePowerPin, HIGH);  // Power OFF moisture sensor
#endif
      s_normal_state = NormalState::kWaitingForSleep;
      og3::s_app.tasks().runIn(og3::kMsecInSec, og3::start_sleep);
      og3::s_led.blink(1);
      break;
    }

    case NormalState::kWaitingToSampleAndSend:
#ifdef HAVE_MOISTURE_POWER_CTL
      if (millis() < og3::kMoistureSettleMs) {
        return;  // Still within the settling window (from boot wake)
      }
#endif
      // Sample all sensors and send readings packet
      og3::s_packet_sender.update();
#ifdef HAVE_MOISTURE_POWER_CTL
      digitalWrite(og3::kMoisturePowerPin, HIGH);  // Power OFF moisture sensor
#endif
      // Since RFM95W doesn't provide a reliable tx-done signal,
      //  schedule deep sleep in 1 second using og3 tasks.
      s_normal_state = NormalState::kWaitingForSleep;
      og3::s_app.tasks().runIn(og3::kMsecInSec, og3::start_sleep);
      og3::s_led.blink(1);
      break;

    case NormalState::kWaitingForSleep:
      // Waiting for the 1-second task to trigger start_sleep().
      // og3::s_app.loop() continues executing in the background.
      break;
  }
}
