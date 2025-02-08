// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <og3/constants.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/lora.h>
#include <og3/shtc3.h>
#include <pb_encode.h>

#include "device.pb.h"

#define VERSION "0.1.0"

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

HAApp s_app(HAApp::Options(kManufacturer, kModel,
                           WifiApp::Options()
                               .withSoftwareName(kSoftware)
                               .withDefaultDeviceName("rooml33")
#if defined(LOG_UDP) && defined(LOG_UDP_ADDRESS)
                               .withUdpLogHost(IPAddress(LOG_UDP_ADDRESS))

#endif
                               .withOta(OtaManager::Options(OTA_PASSWORD))
                               .withApp(App::Options().withLogType(kLogType))));

VariableGroup s_cvg("garage_cfg");
VariableGroup s_vg("garage");

static const char kTemperature[] = "temperature";
static const char kHumidity[] = "humidity";

// Hardware config
// Define the pins used by the transceiver module.
constexpr int kLoraSS = 5;
constexpr int kLoraRst = 14;
constexpr int kLoraDio0 = 2;
constexpr int kDebugSwitchPin = 33;

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

const og3_Sensor s_sensor_temp{0x1 /*TODO: id*/, "temp", "C"};
const og3_Sensor s_sensor_humidity{0x2 /*TODO: id*/, "humidity", "%"};
const og3_Sensor s_sensor_moisture{0x3 /*TODO: id*/, "soil moisture", "%"};

LoRaModule s_lora("lora", LoRaModule::Options(), &s_app, s_vg, _on_lora_initialized);

template <typename T>
void copy(T& dest, const T& src){memcpy(&dest, &src, sizeof(dest));}

class PacketSender {
 public:
  PacketSender() {}
  void update() {
    const int64_t now_secs = esp_timer_get_time() / kUsecInSec;
    if ((now_secs - m_secs_sensor_sent) < kSecInMin) {
      return;
    }
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
    packet.component_count = 3;
    packet.component[0].which_kind = og3_TemperatureSensor_sensor_tag;
    packet.component[1].which_kind = og3_HumiditySensor_sensor_tag;
    packet.component[2].which_kind = og3_MoistureSensor_sensor_tag;
    auto& temp = packet.component[0].kind.temperature;
    auto& humid = packet.component[1].kind.humidity;
    auto& moist = packet.component[2].kind.moisture;
    if (update_device) {
      temp.has_sensor = 1;
      humid.has_sensor = 1;
      moist.has_sensor = 1;
      copy(temp.sensor, s_sensor_temp);
      copy(humid.sensor, s_sensor_humidity);
      copy(moist.sensor, s_sensor_moisture);
    }
    temp.value = 25;
    humid.value = 50;
    moist.percent = 40;

    char buffer[og3_Packet_size];
    
  }

 private:
  uint64_t m_secs_device_sent = 0;
  // TODO(chrishl): device_sent_wakeups in storage surviving deep sleep.
  uint64_t m_secs_sensor_sent = 0;
};

// Global variable for html, so asyncwebserver can send data in the background (single client)
String s_html;

WebButton s_button_wifi_config = s_app.createWifiConfigButton();
WebButton s_button_mqtt_config = s_app.createMqttConfigButton();
WebButton s_button_app_status = s_app.createAppStatusButton();
WebButton s_button_restart = s_app.createRestartButton();

void handleWebRoot(AsyncWebServerRequest* request) {
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
  pinMode(og3::kDebugSwitchPin, INPUT);
  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/config", [](AsyncWebServerRequest* request) {});
  og3::s_app.setup();
}

void loop() { og3::s_app.loop(); }
