// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <og3/constants.h>
#include <og3/dependencies.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/kernel_filter.h>
#include <og3/lora.h>
#include <og3/mapped_analog_sensor.h>
#include <og3/packet_writer.h>
#include <og3/shtc3.h>
#include <og3/tasks.h>
#include <og3/units.h>
#include <pb_encode.h>

#include <memory>

#include "device.pb.h"

#define VERSION "0.3.0"

namespace og3 {

static const char kManufacturer[] = "Chris Lee";
static const char kModel[] = "Garden133";
static const char kSoftware[] = "Garden133 v" VERSION;

#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
constexpr App::LogType kLogType = App::LogType::kUdp;
#else
// constexpr App::LogType kLogType = App::LogType::kNone;  // kSerial
constexpr App::LogType kLogType = App::LogType::kSerial;
#endif

constexpr uint32_t kCleeOrg = 0xc133;
constexpr uint16_t kDevicePktType = 0xde1c;

uint16_t s_board_id = 0xFF;

HAApp s_app(HAApp::Options(kManufacturer, kModel,
                           WifiApp::Options()
                               .withSoftwareName(kSoftware)
                               .withDefaultDeviceName("rooml33")
#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
                               .withUdpLogHost(IPAddress(LOG_UDP_ADDRESS))

#endif
                               .withOta(OtaManager::Options(OTA_PASSWORD))
                               .withApp(App::Options().withLogType(kLogType))));

VariableGroup s_vg("garden");

static const char kTemperature[] = "temperature";
static const char kHumidity[] = "humidity";

// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
constexpr int kLoraDio0 = 2;

constexpr int kFiveVPin = 32;      // ADC1_CH4
constexpr int kBatteryPin = 33;    // ADC1_CH5
constexpr int kMoisturePin = 34;   // ADC1_CH6
constexpr int kSolarPlusPin = 35;  // ADC1_CH7
constexpr int kDebugSwitchPin = 04;

// Sleep for 1 minute between readings (for now).
constexpr unsigned kSleepSecs = kSecInMin;

// Number of samples to store for the moisture filter.
constexpr unsigned kFilterSize = 32;

namespace Status {
constexpr unsigned kDebugMode = 0x1;
constexpr unsigned kPowerOn = 0x2;
constexpr unsigned kDeepSleep = 0x4;
constexpr unsigned kFilterRestoreFailure = 0x8;
constexpr unsigned kFilterSaveFailure = 0x10;
}  // namespace Status

// This data should persist during deep sleep.
RTC_DATA_ATTR struct {
  uint16_t bootCount = 0;
  byte mac[6];
  KernelFilter::State<kFilterSize> filter_state;
  unsigned code = 0;
} s_rtc;

bool s_lora_ok = false;
bool s_is_debug_mode = false;

Shtc3 s_shtc3(kTemperature, kHumidity, &s_app.module_system(), "temperature", s_vg);

void _on_lora_initialized() {
  LoRa.setSpreadingFactor(12);
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  LoRa.enableCrc();
}

LoRaModule s_lora("lora", LoRaModule::Options(), &s_app, s_vg, _on_lora_initialized);

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

class ConfigModule : public Module {
 public:
  explicit ConfigModule(const char* name)
      : Module(name, &s_app.module_system()),
        m_deps(ConfigInterface::kName),
        m_cvg(name),
        m_cfg_url(String("/config/") + name) {
    setDependencies(&m_deps);
    add_link_fn([this](og3::NameToModule& name_to_module) -> bool {
      m_config = ConfigInterface::get(name_to_module);
      return true;
    });
    add_init_fn([this]() {
      if (m_config) {
        m_config->read_config(m_cvg);
      }
      s_app.web_server().on(cfg_url(), [this](AsyncWebServerRequest* request) {
        this->handleConfigRequest(request);
      });
    });
  }

  const char* cfg_url() const { return m_cfg_url.c_str(); }
  void add_html_button(String* body) const { Module::add_html_button(body, name(), cfg_url()); }

 protected:
  SingleDependency m_deps;
  VariableGroup m_cvg;
  ConfigInterface* m_config = nullptr;
  String m_cfg_url;

 private:
  void handleConfigRequest(AsyncWebServerRequest* request) {
    ::og3::read(*request, m_cvg);
    s_html.clear();
    html::writeFormTableInto(&s_html, m_cvg);
    Module::add_html_button(&s_html, "Back", "/");
    sendWrappedHTML(request, s_app.board_cname(), name(), s_html.c_str());
    s_app.config().write_config(m_cvg);
  }
};

class MoistureSensor : public ConfigModule {
 public:
  MoistureSensor()
      : ConfigModule("moisture"),
        m_mapped_adc(
            {
                .name = "soil moisture",
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

 private:
  // Multiple constants by 2/3 compared to Plant133 due to voltage divider.
  // Default ADC reading at which to consider soil moisture to be 100%.
  static constexpr unsigned kFullMoistureCounts = 355;  // 2*1000/3;
  // Default ADC reading at which to consider soil moisture to be 0%.
  static constexpr unsigned kNoMoistureCounts = 780;  // 2*2800/2;

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
  const int64_t total_sleep_usecs = s_rtc.bootCount * kSleepSecs * kUsecInSec;
  return esp_timer_get_time() + total_sleep_usecs;
}

class ADV20Mez10 : public ConfigModule {
  // voltage divider with 10k, 20k(mez) R.
 public:
  ADV20Mez10(const char* name, uint8_t pin, const char* raw_desc, const char* desc, float out_max)
      : ConfigModule(name),
        m_mapped_adc(
            {
                .name = name,
                .pin = pin,
                .units = units::kVolt,
                .raw_description = raw_desc,
                .description = desc,
                .raw_var_flags = 0,
                .mapped_var_flags = 0,
                .config_flags = kCfgSet,
                .default_in_min = 0,
                .default_in_max = 1 << 12,
                .default_out_min = 0.0f,
                .default_out_max = out_max,
                .config_decimals = 2,
                .decimals = 2,
                .valid_in_min = 50,
                .valid_in_max = 1 << 12,
            },
            &s_app.module_system(), m_cvg, s_vg) {}

  float read() { return m_mapped_adc.read(); }
  float value() const { return m_mapped_adc.value(); }
  const FloatVariable& valueVariable() const { return m_mapped_adc.mapped_value(); }
  int raw_counts() const { return m_mapped_adc.raw_counts(); }

 private:
  static constexpr unsigned kCfgSet = VariableBase::Flags::kConfig | VariableBase::Flags::kSettable;

  MappedAnalogSensor m_mapped_adc;
};

ADV20Mez10 s_five_v_sensor("fivev voltage", kFiveVPin, "fivev raw value",
                           "voltage from battery or solar", 3.3 * 2);

ADV20Mez10 s_battery_sensor("battery voltage", kBatteryPin, "battery raw value", nullptr,
                            3.3 * 32.0 / 22.0);

ADV20Mez10 s_solar_sensor("solar voltage", kSolarPlusPin, "solar raw value", nullptr,
                          3.3 * 32.0 / 22.0);

Variable<unsigned> s_status_var("status", 0, nullptr, "status flags", 0, s_vg);
Variable<unsigned> s_board_id_var("board id", 0, nullptr, nullptr, 0, s_vg);

#define SETSTR(X, VAL) strncpy(X, (VAL), sizeof(X) - 1)

class PacketSender {
 public:
  PacketSender() {}
  void update() {
    const int64_t now_usecs = total_usecs();
    const int64_t now_secs = now_usecs / kUsecInSec;

    // Read the temp/humdity sensor and the moisture sensor.
    s_shtc3.read();
    s_moisture_filter.addSample(now_usecs * 1e-6, s_moisture.read());

    s_five_v_sensor.read();
    s_battery_sensor.read();
    s_solar_sensor.read();

    s_status_var = og3::s_rtc.code;
    s_board_id_var = og3::s_board_id;

    if (s_is_debug_mode) {
      s_app.mqttSend(s_vg);
    }

    s_app.log().debug("PacketSender::update() preparing packet.");
    // Update-device logic should work when not in debug mode.
    // was: m_secs_device_sent == 0 || (now_secs - m_secs_device_sent) > kSecInHour;
    const bool update_device = true;
    const bool update_sensors = true;
    og3_Packet packet og3_Packet_init_zero;
    packet.device_id = s_board_id;
    packet.has_device = update_device;
    if (update_device) {
      packet.device.id = s_board_id;
      packet.device.manufacturer = kCleeOrg;
      SETSTR(packet.device.name, "Garden133");
      packet.device.hardware_version.major = 3;
      packet.device.hardware_version.minor = 0;
      packet.device.software_version.major = 0;
      packet.device.software_version.minor = 1;
    }

    unsigned sensor_id = 0;

    auto set = [&sensor_id, &packet, update_sensors](const FloatVariable& var,
                                                     og3_Sensor_Type type) {
      auto& reading = packet.reading[packet.reading_count];
      reading.sensor_id = sensor_id;
      reading.has_sensor = update_sensors;
      reading.value = var.value();
      if (update_sensors) {
        reading.sensor.id = sensor_id;
        SETSTR(reading.sensor.name, var.name());
        SETSTR(reading.sensor.units, var.units());
        reading.sensor.type = type;
      }
      packet.reading_count += 1;
      sensor_id += 1;
    };

    auto i_set = [&sensor_id, &packet, update_sensors](uint32_t val, const char* name,
                                                       const char* units, og3_Sensor_Type type) {
      auto& reading = packet.i_reading[packet.i_reading_count];
      reading.sensor_id = sensor_id;
      reading.has_sensor = update_sensors;
      reading.value = val;
      if (update_sensors) {
        reading.sensor.id = sensor_id;
        SETSTR(reading.sensor.name, name);
        SETSTR(reading.sensor.units, units);
        reading.sensor.type = type;
      }
      packet.i_reading_count += 1;
      sensor_id += 1;
    };

    if (s_shtc3.ok()) {
      set(s_shtc3.temperatureVar(), og3_Sensor_Type_TYPE_TEMPERATURE);
      set(s_shtc3.humidityVar(), og3_Sensor_Type_TYPE_HUMIDITY);
    }

    set(s_moisture_filter.valueVariable(), og3_Sensor_Type_TYPE_MOISTURE);
    set(s_five_v_sensor.valueVariable(), og3_Sensor_Type_TYPE_VOLTAGE);
    set(s_battery_sensor.valueVariable(), og3_Sensor_Type_TYPE_VOLTAGE);
    // set(s_solar_sensor.valueVariable(), og3_Sensor_Type_TYPE_VOLTAGE);

    i_set(s_moisture.raw_counts(), "soil ADC counts", "", og3_Sensor_Type_TYPE_INT_NUMBER);
    i_set(s_rtc.code, "status", "", og3_Sensor_Type_TYPE_INT_NUMBER);

    uint8_t msg_buffer[og3_Packet_size];
    pb_ostream_t ostream = pb_ostream_from_buffer(msg_buffer, sizeof(msg_buffer));
    if (!pb_encode(&ostream, &og3_Packet_msg, &packet)) {
      s_app.log().log("Failed to pb_encode packet.");
      return;
    }

    uint8_t pkt_buffer[og3_Packet_size + pkt::kHeaderSize + pkt::kMsgHeaderSize];
    const uint16_t seq_id = s_is_debug_mode ? m_seq_id : s_rtc.bootCount;
    pkt::PacketWriter writer(pkt_buffer, sizeof(pkt_buffer), seq_id);
    if (!writer.add_message(kDevicePktType, msg_buffer, ostream.bytes_written)) {
      s_app.log().log("Failed to add device msg to packet.");
      return;
    }

    // Disable on new board until different MAC is detected.
    LoRa.beginPacket();
    LoRa.write(pkt_buffer, writer.packet_size());
    LoRa.endPacket();

    s_app.log().debugf("Sent LoRa packet (seq_id:%u, %zu bytes).", m_seq_id, ostream.bytes_written);
    m_seq_id += 1;
    if (update_device) {
      m_secs_device_sent = now_secs;
    }
  }

 private:
  uint16_t m_seq_id = 0;
  uint64_t m_secs_device_sent = 0;
};

PacketSender s_packet_sender;

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

void handleWebRoot(AsyncWebServerRequest* request) {
  const int64_t now_usecs = total_usecs();
  s_moisture_filter.addSample(now_usecs * 1e-6, s_moisture.read());
  s_shtc3.read();
  s_html.clear();
  html::writeTableInto(&s_html, s_vg);
  html::writeTableInto(&s_html, s_app.wifi_manager().variables());
  html::writeTableInto(&s_html, s_app.mqtt_manager().variables());

  s_moisture.add_html_button(&s_html);
  s_five_v_sensor.add_html_button(&s_html);
  s_battery_sensor.add_html_button(&s_html);
  s_solar_sensor.add_html_button(&s_html);

  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(og3::kDebugSwitchPin, INPUT);
  og3::s_is_debug_mode = digitalRead(og3::kDebugSwitchPin);
  if (og3::s_is_debug_mode) {
    og3::s_rtc.code |= og3::Status::kDebugMode;
  } else {
    og3::s_rtc.code = og3::s_is_debug_mode ? og3::Status::kDebugMode : 0;
  }

  if (!og3::s_is_debug_mode) {
    // This is the normal operation: wake from sleep, send a packet with LoRa, go back to sleep.
    og3::s_app.wifi_manager().set_enable(false);
    if (og3::print_wakeup_reason()) {
      // Normal operation, but first-time boot, not waking from deep sleep.
      og3::s_rtc.bootCount = 0;
      WiFi.macAddress(og3::s_rtc.mac);
      og3::s_rtc.code |= og3::Status::kPowerOn;
    } else {
      // Normal operation, woke from deep sleep.
      if (!og3::s_moisture_filter.restoreState(og3::s_rtc.filter_state)) {
        og3::s_rtc.code |= og3::Status::kFilterRestoreFailure;
      }
      og3::s_rtc.bootCount += 1;
      og3::s_rtc.code |= og3::Status::kDeepSleep;
    }
    // build a board-id from the mac address.
  } else {
    constexpr auto kInitialWait = 10 * og3::kMsecInSec;
    constexpr auto kUpdatePeriod = og3::kMsecInMin;
    WiFi.macAddress(og3::s_rtc.mac);
    og3::s_hourly_lora.reset(new og3::PeriodicTaskScheduler(
        kInitialWait, kUpdatePeriod, []() { og3::s_packet_sender.update(); }, &og3::s_app.tasks()));
  }
  // Constuct a board ID from the MAC address.
  og3::s_board_id = (og3::s_rtc.mac[3] << 8) | (og3::s_rtc.mac[4] ^ og3::s_rtc.mac[5]);

  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.setup();
}

void loop() {
  og3::s_app.loop();

  if (og3::s_is_debug_mode) {
    return;  // In debug mode, we don't run the stuff below which puts the board into deep sleep.
  }

  // This is normal operation: send a LoRa packet and then do deep sleep.
  if (og3::s_lora.is_ok()) {
    // Read the sensors, send a LoRa packet.
    og3::s_packet_sender.update();
  } else if (millis() < og3::kMsecInSec * 10) {
    return;  // Wait for up to 10 seconds for LoRa module to initialize.
  }

  // Save filter state to memory that is preserved when sleeping.
  if (!og3::s_moisture_filter.saveState(og3::s_rtc.filter_state)) {
    og3::s_rtc.code |= og3::Status::kFilterSaveFailure;
  }

  // After we go to sleep below, set a timer which will wakeup the board in 1 minute.
  esp_sleep_enable_timer_wakeup(og3::kSleepSecs * og3::kUsecInSec);

  // Shut down peripherals here.
  esp_deep_sleep_start();
}
