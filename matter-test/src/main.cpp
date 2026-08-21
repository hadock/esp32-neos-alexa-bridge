// Standalone feasibility test for the Arduino-ESP32 Matter stack on our
// exact board (ESP32-S3, no PSRAM, 8MB flash). Adapted from Espressif's
// official MatterContactSensor example. The goal here is narrow: does this
// compile within the huge_app partition, and does it run without crashing
// (i.e. does the no-PSRAM RAM budget hold up)? Not wired to the real
// bridge/dongle at all yet -- that's the next step if this works.

#include <Arduino.h>
#include <Matter.h>
#include <WiFi.h>
#include "secrets.h"

MatterContactSensor ContactSensor;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("Matter feasibility test");
  Serial.printf("Free heap before WiFi: %u bytes\n", ESP.getFreeHeap());

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Free heap after WiFi: %u bytes\n", ESP.getFreeHeap());

  ContactSensor.begin();
  Serial.printf("Free heap after ContactSensor.begin(): %u bytes\n", ESP.getFreeHeap());

  Matter.begin();
  Serial.printf("Free heap after Matter.begin(): %u bytes\n", ESP.getFreeHeap());

  if (!Matter.isDeviceCommissioned()) {
    Serial.println();
    Serial.println("Matter Node is not commissioned yet.");
    Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  } else {
    Serial.println("Matter Node already commissioned.");
  }
}

void loop() {
  static uint32_t lastToggle = 0;
  static uint32_t lastHeapPrint = 0;
  static uint32_t lastPairingPrint = 0;
  uint32_t now = millis();

  // Re-print the pairing code periodically instead of only once at boot --
  // a fresh reset's early output has been unreliable to capture over
  // serial (looks like a real USB-CDC re-enumeration glitch right at
  // reset), so this lets it be read at any convenient moment instead.
  if (!Matter.isDeviceCommissioned() && now - lastPairingPrint > 10000) {
    lastPairingPrint = now;
    Serial.printf("[%lu] Not commissioned. Manual pairing code: %s | QR code URL: %s\n", now,
                  Matter.getManualPairingCode().c_str(), Matter.getOnboardingQRCodeUrl().c_str());
  }

  if (now - lastToggle > 20000) {
    lastToggle = now;
    ContactSensor.setContact(!ContactSensor.getContact());
    Serial.printf("[%lu] Toggled contact to %s. Commissioned=%d Free heap=%u\n", now,
                  ContactSensor.getContact() ? "closed" : "open", Matter.isDeviceCommissioned(),
                  ESP.getFreeHeap());
  }

  if (now - lastHeapPrint > 5000) {
    lastHeapPrint = now;
    Serial.printf("[%lu] heartbeat, free heap=%u, min free heap=%u\n", now, ESP.getFreeHeap(),
                  ESP.getMinFreeHeap());
  }

  delay(50);
}
