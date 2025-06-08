// Copyright (c) 2025 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <og3/adc_voltage.h>
#include <og3/blink_led.h>
#include <og3/config_module.h>
#include <og3/constants.h>
#include <og3/dependencies.h>
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
#include <pb_encode.h>

#include <memory>
#include <vector>

#define VERSION_MAJOR 0
#define VERSION_MINOR 5
#define VERSION_PATCH 0
#define STR(X) #X
#define MAKE_VERSION(MAJOR, MINOR, PATCH) STR(MAJOR) "." STR(MINOR) "." STR(PATCH)
#define VERSION MAKE_VERSION(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH)

#define HARDWARE_VERSION_MAJOR 6
#define HARDWARE_VERSION_MINOR 0

// TODO
//
// Use interrupt on tx-finished to sleep, etc...
// Sleep from wake time, to keep wake time more predictable.
//
// I think right now my packets are around 80 bytes long, which with my selected
//  spreading factor leads to 2.4 sec transmittion.
// The FCC says "dwell time" on a single frequency should be 400msec, so I
//  need to break things up.
// "Packets can contain up to 255 bytes."

// LoRa stuff to set:
//  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
// x void setFrequency(long frequency);
// x void setSpreadingFactor(int sf);
// x void setSignalBandwidth(long sbw);
//  Should probably be calling this after transmitting to save power:
//    LoRa.sleep();

namespace og3 {

static const char kManufacturer[] = "Chris Lee";
static const char kModel[] = "Garden133";
static const char kSoftware[] = "Garden133 v" VERSION;

constexpr App::LogType kLogType = LOG_METHOD;
constexpr uint32_t kCleeOrg = 0xc133;
constexpr uint16_t kDevicePktType = 0xde1c;

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
constexpr int kLoraDio0 = 2;

constexpr int kFiveVPin = 32;      // ADC1_CH4
constexpr int kBatteryPin = 33;    // ADC1_CH5
constexpr int kMoisturePin = 34;   // ADC1_CH6
constexpr int kSolarPlusPin = 35;  // ADC1_CH7
constexpr int kDebugSwitchPin = 04;
constexpr int kLedPin = 16;

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

constexpr unsigned kBlinkMsec = 100;

// This data should persist during deep sleep.
// DO NOT SET INITIALIZERS ON THESE VALUES!
RTC_DATA_ATTR struct {
  unsigned bootCount;
  byte mac[6];
  KernelFilter::State<kFilterSize> filter_state;
  unsigned code;
  satellite::PacketSender::Rtc packet_sender;
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
  Variable<unsigned>& countsVar() { return m_mapped_adc.raw_value(); }

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
  const int64_t total_sleep_usecs = static_cast<int64_t>(s_rtc.bootCount) * kSleepSecs * kUsecInSec;
  return esp_timer_get_time() + total_sleep_usecs;
}

AdcVoltage s_five_v_sensor("fivev voltage", &s_app, kFiveVPin, "fivev raw value",
                           "voltage from battery or solar", 3.3 * 2, s_vg, s_cvg);

AdcVoltage s_battery_sensor("battery voltage", &s_app, kBatteryPin, "battery raw value", nullptr,
                            3.3 * 32.0 / 22.0, s_vg, s_cvg);

AdcVoltage s_solar_sensor("solar voltage", &s_app, kSolarPlusPin, "solar raw value", nullptr,
                          3.3 * 2, s_vg, s_cvg);

Variable<unsigned> s_status_var("status", 0, nullptr, "status flags", 0, s_vg);
Variable<unsigned> s_board_id_var("board id", 0, nullptr, nullptr, 0, s_vg);
Variable<unsigned> s_wake_secs("wake_secs", 0, units::kSeconds, nullptr, 0, s_vg);
Variable<unsigned> s_secs_device_sent("secs device sent", 0, units::kSeconds, nullptr, 0, s_vg);

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
                           moisture_filter.valueVariable()),
        m_moisture(moisture),
        m_moisture_filter(moisture_filter) {}

  bool read() {
    // When the power supply is < 3.2V, it seems moisture sensor readings are not trustworthy.
    m_is_ok = s_five_v_sensor.read() > kMinVForValidReading;
    if (!m_is_ok) {
      return false;
    }
    const int64_t now_usecs = total_usecs();
    m_moisture_filter.addSample(now_usecs * 1e-6, m_moisture.read());
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
  LoraPacketSender(const og3_Device* device, og3::App* app, og3::satellite::PacketSender::Rtc* rtc)
      : satellite::PacketSender(device, app, rtc) {
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
#if 0
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "wake time", s_wake_secs));
    m_readings.emplace_back(new satellite::PacketIntReading(idx(), "time device sent", s_secs_device_sent));
#endif
  }

  void update() {
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
    s_wake_secs = now_secs;
    s_secs_device_sent = m_rtc->secs_device_sent;

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
};

og3_Device* s_device() {
  static og3_Device ret;

  ret.id = s_board_id;  // may not be set yet
  ret.manufacturer = kCleeOrg;
  SETSTR(ret.name, kModel);
  ret.hardware_version.major = HARDWARE_VERSION_MAJOR;
  ret.hardware_version.minor = HARDWARE_VERSION_MINOR;
  ret.software_version.major = VERSION_MAJOR;
  ret.software_version.minor = VERSION_MINOR;
  ret.software_version.patch = VERSION_PATCH;
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

  s_html += HTML_BUTTON("/lora", "LoRa");
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

void handleLoraConfig(AsyncWebServerRequest* request) {
  ::og3::read(*request, s_lora_vg);
  s_html.clear();
  html::writeFormTableInto(&s_html, s_lora_vg);
  s_html += HTML_BUTTON("/", "Back");
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
  s_app.config().write_config(s_lora_vg);
}

void start_sleep() {
  s_app.log().debug("Sleeping");
  s_rtc.code &= ~Status::kDebugMode;

  // Save filter state to memory that is preserved when sleeping.
  if (!s_moisture_filter.saveState(s_rtc.filter_state)) {
    s_rtc.code |= Status::kFilterSaveFailure;
  }

  // Put the radio to sleep to save power.
  LoRa.sleep();

  // After we go to sleep below, set a timer which will wakeup the board in 1 minute.
  esp_sleep_enable_timer_wakeup(kSleepSecs * kUsecInSec);

  // Shut down peripherals here.
  esp_deep_sleep_start();
}

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(og3::kDebugSwitchPin, INPUT);
  og3::s_is_debug_mode = digitalRead(og3::kDebugSwitchPin);
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
  // Constuct a board ID from the MAC address.
  og3::s_board_id = (og3::s_rtc.mac[3] << 8) | (og3::s_rtc.mac[4] ^ og3::s_rtc.mac[5]);
  og3::s_packet_sender.set_board_id(og3::s_board_id);

  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/lora", og3::handleLoraConfig);
  og3::s_app.setup();
}

void loop() {
  og3::s_app.loop();

  // Check debug-mode again, so we can switch away from debug mode while the board is running.
  og3::s_is_debug_mode = digitalRead(og3::kDebugSwitchPin);

  if (og3::s_is_debug_mode) {
    return;  // In debug mode, all the work happens in og3::s_app.loop().
  }

  // Here, we are in "normal mode", where we normally take readings, send them via LoRa,
  //  then go into deep sleep for a while to preserve power.

  // Wait up to 10 seconds for LoRa to start-up at boot.
  // After that, if LoRa isn't running, just sleep.
  if (!og3::s_lora.is_ok()) {
    if (millis() < og3::kMsecInSec * 10) {
      return;
    }
    og3::start_sleep();
  }

  // Here, are are in "normal mode" and the LoRa radio is running.

  // Just once, read the sensors, and send a LoRa packet.
  static bool s_sent = false;
  if (!s_sent) {
    og3::s_packet_sender.update();
    s_sent = true;
  }

  // Here, we have sent a packet and may send further ones before sleeping.
  // Wait for is_sending() to be unset, meaning that all packets have been sent,
  //  then wait for 1 second and then go to sleep.

  if (og3::s_packet_sender.is_sending()) {
    return;
  }

  // Here, all packets have been sent.
  // Schedule a sleep in 1 second if not yet started.
  static bool s_sleep_started = false;
  if (!s_sleep_started) {
    s_sleep_started = true;
    og3::s_app.tasks().runIn(og3::kMsecInSec, og3::start_sleep);
    og3::s_led.blink(1);
  }
}
