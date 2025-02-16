// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <og3/constants.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/lora.h>
#include <og3/mapped_analog_sensor.h>
#include <og3/packet_writer.h>
#include <og3/shtc3.h>
#include <og3/tasks.h>
#include <og3/units.h>
#include <pb_encode.h>

#include <memory>

#include "device.pb.h"

#define VERSION "0.1.0"

// - Deep sleep
// - Packet indicates debug mode

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

HAApp s_app(HAApp::Options(kManufacturer, kModel,
                           WifiApp::Options()
                               .withSoftwareName(kSoftware)
                               .withDefaultDeviceName("rooml33")
#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
                               .withUdpLogHost(IPAddress(LOG_UDP_ADDRESS))

#endif
                               .withOta(OtaManager::Options(OTA_PASSWORD))
                               .withApp(App::Options().withLogType(kLogType))));

VariableGroup s_cvg("garden_cfg");
VariableGroup s_vg("garden");

static const char kTemperature[] = "temperature";
static const char kHumidity[] = "humidity";

// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
constexpr int kLoraDio0 = 2;
constexpr int kDebugSwitchPin = 33;

constexpr int kMoisturePin = 32;

bool s_lora_ok = false;
bool s_is_debug_mode = false;

Shtc3 s_shtc3(kTemperature, kHumidity, &s_app.module_system(), "temperature", s_vg);

void _on_lora_initialized() {
  LoRa.setSpreadingFactor(12);
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
}

#define SETSTR(X, VAL) strncpy(X, (VAL), sizeof(X) - 1)

LoRaModule s_lora("lora", LoRaModule::Options(), &s_app, s_vg, _on_lora_initialized);

template <typename T>
void copy(T& dest, const T& src) {
  memcpy(&dest, &src, sizeof(dest));
}

class MoistureSensor {
 public:
  MoistureSensor()
      : m_mapped_adc(
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
            &s_app.module_system(), s_cvg, s_vg) {}

  float read() { return m_mapped_adc.read(); }
  float value() const { return m_mapped_adc.value(); }

 private:
  // Multiple constants by 2/3 compared to Plant133 due to voltage divider.
  // Default ADC reading at which to consider soil moisture to be 100%.
  static constexpr unsigned kFullMoistureCounts = 100;  // 2*1000/3;
  // Default ADC reading at which to consider soil moisture to be 0%.
  static constexpr unsigned kNoMoistureCounts = 450;  // 2*2800/2;

  static constexpr unsigned kCfgSet = VariableBase::Flags::kConfig | VariableBase::Flags::kSettable;

  MappedAnalogSensor m_mapped_adc;
};

MoistureSensor s_moisture;

class PacketSender {
 public:
  PacketSender() {}
  void update() {
    // TODO: Update logic for deep sleep.
    const int64_t now_secs = esp_timer_get_time() / kUsecInSec;
    if ((now_secs - m_secs_sensor_sent) < kSecInMin) {
      s_app.log().log("PacketSender::update() not time yet.");
      return;
    }

    // Read the temp/humdity sensor and the moisture sensor.
    s_shtc3.read();
    s_moisture.read();

    if (s_is_debug_mode) {
      s_app.mqttSend(s_vg);
    }

    s_app.log().log("PacketSender::update() preparing packet.");
    const bool update_device =
        m_secs_device_sent == 0 || (now_secs - m_secs_device_sent) > kSecInHour;
    og3_Packet packet og3_Packet_init_zero;
    packet.has_device = update_device;
    packet.has_device = m_secs_device_sent == 0 || (now_secs - m_secs_device_sent) > kSecInHour;
    if (update_device) {
      packet.device.id = 0x2;
      packet.device.manufacturer = kCleeOrg;
      SETSTR(packet.device.name, "Garden133");
      packet.device.hardware_version.major = 3;
      packet.device.hardware_version.minor = 0;
      packet.device.software_version.major = 0;
      packet.device.software_version.minor = 1;
    }

    auto set = [](og3_FloatSensorReading& reading, uint32_t id, float val, bool set_sensor,
                  const char* name, const char* units, og3_Sensor_Type type) {
      reading.sensor_id = id;
      reading.has_sensor = set_sensor;
      reading.value = val;
      if (set_sensor) {
        reading.sensor.id = id;
        SETSTR(reading.sensor.name, name);
        SETSTR(reading.sensor.units, units);
        reading.sensor.type = type;
      }
    };

    unsigned count = 0;
    constexpr bool kSetSensor = true;
    if (s_shtc3.ok()) {
      set(packet.reading[count++], 0x1, s_shtc3.temperature(), kSetSensor, "temperature", "C",
          og3_Sensor_Type_TYPE_TEMPERATURE);
      set(packet.reading[count++], 0x2, s_shtc3.humidity(), kSetSensor, "humidity", "%",
          og3_Sensor_Type_TYPE_HUMIDITY);
    }

    set(packet.reading[count++], 0x3, s_moisture.value(), kSetSensor, "soil moisture", "%",
        og3_Sensor_Type_TYPE_MOISTURE);
    packet.reading_count = count;

    uint8_t msg_buffer[og3_Packet_size];
    pb_ostream_t ostream = pb_ostream_from_buffer(msg_buffer, sizeof(msg_buffer));
    if (!pb_encode(&ostream, &og3_Packet_msg, &packet)) {
      s_app.log().log("Failed to pb_encode packet.");
      return;
    }

    uint8_t pkt_buffer[og3_Packet_size + pkt::kHeaderSize + pkt::kMsgHeaderSize];
    pkt::PacketWriter writer(pkt_buffer, sizeof(pkt_buffer), m_seq_id);
    if (!writer.add_message(kDevicePktType, msg_buffer, ostream.bytes_written)) {
      s_app.log().log("Failed to add device msg to packet.");
      return;
    }
    LoRa.beginPacket();
    LoRa.write(pkt_buffer, writer.packet_size());
    LoRa.endPacket();
    s_app.log().debugf("Sent LoRa packet (seq_id:%u, %zu bytes).", m_seq_id, ostream.bytes_written);
    m_seq_id += 1;
    m_secs_device_sent = now_secs;
  }

 private:
  uint16_t m_seq_id = 0;
  uint64_t m_secs_device_sent = 0;
  // TODO(chrishl): device_sent_wakeups in storage surviving deep sleep.
  uint64_t m_secs_sensor_sent = 0;
};

PacketSender s_packet_sender;

std::unique_ptr<PeriodicTaskScheduler> s_hourly_lora;

// Global variable for html, so asyncwebserver can send data in the background (single client)
String s_html;

WebButton s_button_wifi_config = s_app.createWifiConfigButton();
WebButton s_button_mqtt_config = s_app.createMqttConfigButton();
WebButton s_button_app_status = s_app.createAppStatusButton();
WebButton s_button_restart = s_app.createRestartButton();

void handleWebRoot(AsyncWebServerRequest* request) {
  s_moisture.read();
  s_shtc3.read();
  s_html.clear();
  html::writeTableInto(&s_html, s_vg);
  html::writeTableInto(&s_html, s_app.wifi_manager().variables());
  html::writeTableInto(&s_html, s_app.mqtt_manager().variables());
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  pinMode(og3::kDebugSwitchPin, INPUT);
  og3::s_is_debug_mode = digitalRead(og3::kDebugSwitchPin);
  // Serial.printf("debug mode? %s\n", og3::s_is_debug_mode ? "DEBUG" : "PROD");

  if (!og3::s_is_debug_mode) {
    og3::s_app.wifi_manager().set_enable(false);
  } else {
    constexpr auto kInitialWait = 10 * og3::kMsecInSec;
    constexpr auto kUpdatePeriod = og3::kMsecInMin;
    og3::s_hourly_lora.reset(new og3::PeriodicTaskScheduler(
        kInitialWait, kUpdatePeriod, []() { og3::s_packet_sender.update(); }, &og3::s_app.tasks()));
  }

  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/config", [](AsyncWebServerRequest* request) {});
  og3::s_app.setup();
}

void loop() { og3::s_app.loop(); }
