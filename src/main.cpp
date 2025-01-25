// Copyright (c) 2024 Chris Lee and contibuters.
// Licensed under the MIT license. See LICENSE file in the project root for details.

#include <Arduino.h>
#include <LittleFS.h>
#include <og3/ha_app.h>
#include <og3/html_table.h>
#include <og3/shtc3.h>
#include <LoRa.h>
#include <pb_encode.h>

#include "moisture_packet.pb.h"

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


class RFM95Module : public Module {
public:
  static constexpr unsigned kMaxInitTries = 10;
  
 RFM95Module(const char* name, HAApp* app, VariableGroup& vg)
   : Module(name, &app->module_system()), m_app(app) {
    add_init_fn([this]() {
      // setup LoRa transceiver module
      LoRa.setPins(og3::kLoraSS, og3::kLoraRst, og3::kLoraDio0);
      LoRa.setSpreadingFactor(12);
    });
    add_start_fn([this]() { this->setup_module(); });
 }

 private:
  void setup_module() {
    m_init_tries += 1;
    if (!LoRa.begin(915e6)) {
      og3::s_app.log().logf("Failed to setup LoRa: %u/%u tries.", m_init_tries, kMaxInitTries);
      if (m_init_tries < kMaxInitTries) {
        m_app->tasks().runIn(500, [this]() { setup_module(); });
      }
      return;
    }
    m_is_ok = true;
    og3::s_app.log().logf("Setup LoRa in %u tries.", m_init_tries);
  }
    
  App* m_app;
  unsigned m_init_tries = 0;
  bool m_is_ok = false;
};

RFM95Module s_rfm95("rfm95", &s_app, s_vg);
  

// Global variable for html, so asyncwebserver can send data in the background (single client)
String s_html;

WebButton s_button_wifi_config = s_app.createWifiConfigButton();
WebButton s_button_mqtt_config = s_app.createMqttConfigButton();
WebButton s_button_app_status = s_app.createAppStatusButton();
WebButton s_button_restart = s_app.createRestartButton();

void handleWebRoot(AsyncWebServerRequest* request) {
  html::writeTableInto(&s_html, s_vg);
  html::writeTableInto(&s_html, s_app.wifi_manager().variables());
  html::writeTableInto(&s_html, s_app.mqtt_manager().variables());
#if 0
  s_button_wifi_config.add_button(&s_html);
  s_button_mqtt_config.add_button(&s_html);
  s_button_app_status.add_button(&s_html);
  s_button_restart.add_button(&s_html);
#endif
  sendWrappedHTML(request, s_app.board_cname(), kSoftware, s_html.c_str());
}

}  // namespace og3

////////////////////////////////////////////////////////////////////////////////

void setup() {
  og3::s_app.web_server().on("/", og3::handleWebRoot);
  og3::s_app.web_server().on("/config", [](AsyncWebServerRequest* request) {});
  og3::s_app.setup();
}

void loop() { og3::s_app.loop(); }
