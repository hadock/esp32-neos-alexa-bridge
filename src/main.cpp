// NEOS/WyzeSense USB bridge reader, running the real protocol handshake,
// decoding live sensor events on-device, and exposing each paired sensor
// to Alexa as a real Matter Contact Sensor / Occupancy Sensor endpoint --
// unlike the earlier fauxmoESP/Hue-light approach, these have genuine
// sensor semantics, so Alexa Routines can trigger off them directly
// ("when this happens" -> the sensor -> opens/closes, or detects motion).
//
// Matter endpoints must exist before Matter.begin() runs -- there's no
// dynamic "add an endpoint later" in this API -- so a fixed pool of
// endpoint slots is declared up front (sized to this exact hardware: 2
// contact + 1 motion sensor from the box). Which physical sensor MAC
// lands on which slot is decided at runtime (from the pairing scan's
// type field, or from a sensor's first live event for ones rediscovered
// via GetSensorList at boot) and persisted in flash (Preferences) so the
// same physical sensor always reoccupies the same Matter endpoint across
// reboots -- otherwise Alexa would see it as a different device each time.
//
// Matter doesn't let the device push a friendly display name (that's
// controller-side, set in the Alexa app after commissioning), so unlike
// the fauxmoESP version there's no name registry or `name <MAC> <name>`
// command here -- nothing to push it to.

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Matter.h>
#include <EspUsbHost.h>
#include "wyzesense_dongle.h"
#include "secrets.h"

static EspUsbHost usb;
static wyzesense::Dongle *g_dongle = nullptr;
static uint8_t g_bridgeAddress = 0;
static bool g_bridgeConnected = false;
static bool g_handshakePending = false;
static uint32_t g_connectedAtMs = 0;
static constexpr uint32_t kHandshakeSettleMs = 300;

static Preferences prefs;

// Separate namespace (own key space) for each contact sensor's last-known
// physical state. WyzeSense sensors only ever report *transitions*, never
// "what's your current state right now" -- so a sensor that's been sitting
// closed since before a reboot never generates an event to tell the fresh
// boot about it, and the Matter endpoint would otherwise sit at whatever
// default the library initializes it to (regardless of physical reality)
// until the next real transition. Persist every real event here and
// restore it the moment a MAC gets assigned to a slot, so a reboot doesn't
// make Alexa show a stale/wrong state for an unchanged sensor.
static Preferences prefsContactState;

// GPIO0 is the board's BOOT button -- only meaningful to the ROM bootloader
// very early in boot (for entering flash-download mode); once the app is
// running it's a free input pin. Hold it down while the board powers up to
// enter pairing mode automatically once the handshake completes.
static constexpr int kBootButtonPin = 0;
static constexpr uint32_t kScanTimeoutMs = 60000;
static constexpr uint16_t kMotionHoldTimeSeconds = 8;

// --- Matter endpoint pool: fixed-size, sized to this exact box's hardware.
// Bump these and reflash if you add more sensors. ---
static constexpr size_t kMaxContact = 2;
static constexpr size_t kMaxMotion = 1;

static MatterContactSensor gContact[kMaxContact];
static MatterOccupancySensor gMotion[kMaxMotion];

struct SlotAssignment {
  char mac[9] = {};
  bool inUse = false;
};
static SlotAssignment gContactSlot[kMaxContact];
static SlotAssignment gMotionSlot[kMaxMotion];

static int FindContactSlotByMac(const char *mac) {
  for (size_t i = 0; i < kMaxContact; i++) {
    if (gContactSlot[i].inUse && strcmp(gContactSlot[i].mac, mac) == 0) return int(i);
  }
  return -1;
}

static int FindMotionSlotByMac(const char *mac) {
  for (size_t i = 0; i < kMaxMotion; i++) {
    if (gMotionSlot[i].inUse && strcmp(gMotionSlot[i].mac, mac) == 0) return int(i);
  }
  return -1;
}

// Assigns `mac` to a Contact slot: reuses a previously-persisted mapping
// for this MAC if there is one, else claims the next free slot and
// persists the choice. Returns -1 if every slot is already taken by a
// different sensor.
static int AssignContactSlot(const char *mac) {
  int existing = FindContactSlotByMac(mac);
  if (existing >= 0) return existing;

  int idx = -1;
  String stored = prefs.getString(mac, "");
  if (stored.length() > 1 && stored[0] == 'C') {
    int candidate = stored.substring(1).toInt();
    if (candidate >= 0 && size_t(candidate) < kMaxContact && !gContactSlot[candidate].inUse) {
      idx = candidate;
    }
  }
  if (idx < 0) {
    for (size_t i = 0; i < kMaxContact; i++) {
      if (!gContactSlot[i].inUse) {
        idx = int(i);
        break;
      }
    }
  }
  if (idx < 0) return -1;

  strncpy(gContactSlot[idx].mac, mac, 8);
  gContactSlot[idx].mac[8] = '\0';
  gContactSlot[idx].inUse = true;
  char key[4];
  snprintf(key, sizeof(key), "C%d", idx);
  prefs.putString(mac, key);
  Serial.printf("[matter] %s -> Contact slot %d\n", mac, idx);

  if (prefsContactState.isKey(mac)) {
    bool lastState = prefsContactState.getBool(mac);
    gContact[idx].setContact(!lastState);  // same inversion as the live-event path
    Serial.printf("[matter] %s restored last-known state=%s\n", mac, lastState ? "open" : "closed");
  }
  return idx;
}

static int AssignMotionSlot(const char *mac) {
  int existing = FindMotionSlotByMac(mac);
  if (existing >= 0) return existing;

  int idx = -1;
  String stored = prefs.getString(mac, "");
  if (stored.length() > 1 && stored[0] == 'M') {
    int candidate = stored.substring(1).toInt();
    if (candidate >= 0 && size_t(candidate) < kMaxMotion && !gMotionSlot[candidate].inUse) {
      idx = candidate;
    }
  }
  if (idx < 0) {
    for (size_t i = 0; i < kMaxMotion; i++) {
      if (!gMotionSlot[i].inUse) {
        idx = int(i);
        break;
      }
    }
  }
  if (idx < 0) return -1;

  strncpy(gMotionSlot[idx].mac, mac, 8);
  gMotionSlot[idx].mac[8] = '\0';
  gMotionSlot[idx].inUse = true;
  char key[4];
  snprintf(key, sizeof(key), "M%d", idx);
  prefs.putString(mac, key);
  Serial.printf("[matter] %s -> Motion slot %d\n", mac, idx);
  return idx;
}

static void onScanResult(bool found, const char *mac, uint8_t type, uint8_t version) {
  if (!found) {
    Serial.println("=== Pairing: no sensor found (timed out) ===");
    return;
  }
  Serial.printf("=== Pairing: sensor found! mac=%s type=%u version=%u ===\n", mac, type, version);
  // Protocol sensor type: 1=switch (contact), 2=motion, 3=leak. Motion gets
  // its own pool; everything else is boolean-open/closed-shaped enough to
  // share the Contact pool.
  if (type == 2) {
    if (AssignMotionSlot(mac) < 0) Serial.println("[matter] no free Motion slot");
  } else {
    if (AssignContactSlot(mac) < 0) Serial.println("[matter] no free Contact slot");
  }
}

static void onDeviceConnected(const EspUsbHostDeviceInfo &device) {
  Serial.printf("=== USB device connected: addr=%u vid:pid=%04x:%04x ===\n",
                device.address, device.vid, device.pid);

  if (device.vid == 0x1a86 && device.pid == 0xe024) {
    Serial.println("  -> matches NEOS/WyzeSense bridge");
    g_bridgeAddress = device.address;
    g_bridgeConnected = true;
    // Don't submit a control transfer from inside this callback -- it runs
    // on EspUsbHost's own event task while enumeration may still be
    // settling. Defer the actual handshake start to loop().
    g_connectedAtMs = millis();
    g_handshakePending = true;
  }
}

static void onDeviceDisconnected(const EspUsbHostDeviceInfo &device) {
  Serial.printf("=== USB device disconnected: addr=%u ===\n", device.address);
  if (device.address == g_bridgeAddress) {
    g_bridgeConnected = false;
  }
}

static void onHIDInput(const EspUsbHostHIDInput &input) {
  if (!g_dongle || input.address != g_bridgeAddress) return;
  if (!input.data || input.length < 1) return;

  // The bridge's HID descriptor has no Report ID field, which means
  // EspUsbHost's report-ID-based vendor/mouse/keyboard/gamepad sniffing
  // (which all key off byte[0] matching a specific ID) never recognizes it
  // as anything in particular, so it never reaches onHIDVendorInput -- only
  // the generic onHIDInput fires unconditionally for any HID endpoint.
  //
  // Byte[0] here isn't a report ID at all: it's a length prefix the real
  // WyzeSense protocol embeds inside each 64-byte interrupt IN report (see
  // gateway.py's _ReadRawHID: `length = s[0]; return s[1:1+length]`). Strip
  // it the same way before handing bytes to the protocol layer.
  uint8_t length = input.data[0];
  if (length > 0x3F) length = 0x3F;
  if (size_t(length) + 1 > input.length) return;  // malformed, drop it

  g_dongle->FeedBytes(input.data + 1, length, millis());
}

static const char *SensorTypeName(wyzesense::SensorType t) {
  switch (t) {
    case wyzesense::SensorType::kSwitch: return "switch";
    case wyzesense::SensorType::kMotion: return "motion";
    case wyzesense::SensorType::kLeak: return "leak";
    case wyzesense::SensorType::kLeakTemperature: return "leak:temperature";
    default: return "unknown";
  }
}

static void connectWiFi() {
  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi connect FAILED (will keep retrying in background) -- Alexa integration won't work until connected.");
  }
}

// Handles one line of serial input: "p" to pair a new sensor, "factory-reset"
// to wipe Matter commissioning/subscription state.
static void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "p" || line == "P") {
    if (g_dongle && g_dongle->IsReady()) {
      Serial.println("=== Pairing mode: trigger the new sensor's pairing broadcast now (60s window) ===");
      g_dongle->StartScan(millis(), kScanTimeoutMs, onScanResult);
    } else {
      Serial.println("Dongle not ready yet, can't start pairing.");
    }
    return;
  }

  if (line == "factory-reset") {
    // Wipes the Matter fabric/ACL/subscription-resumption table so the node
    // can be recommissioned clean. Useful after moving the device to a new
    // Alexa account/location, or (as discovered during development) after
    // accumulating enough stale persisted subscriptions from repeated
    // recommissioning that Matter.begin() can no longer resume any of them
    // without exhausting heap.
    Serial.println("=== Factory-resetting Matter state and rebooting... ===");
    delay(100);
    Matter.decommission();
    delay(500);
    ESP.restart();
    return;
  }

  Serial.println("Unknown command. Commands: p (pair new sensor), factory-reset (wipe Matter state)");
}

void setup() {
  pinMode(kBootButtonPin, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("NEOS bridge USB host -- WyzeSense protocol handshake + Matter/Alexa bridge");
  Serial.printf("[psram] size=%u free=%u\n", ESP.getPsramSize(), ESP.getFreePsram());

  prefs.begin("wyzesense", /*readOnly=*/false);
  prefsContactState.begin("wz_ct_state", /*readOnly=*/false);

  connectWiFi();
  Serial.printf("[heap] after WiFi connect: free=%u min_free=%u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // All Matter endpoints must be created before Matter.begin() is called.
  for (size_t i = 0; i < kMaxContact; i++) gContact[i].begin();
  for (size_t i = 0; i < kMaxMotion; i++) gMotion[i].begin();

  Matter.begin();
  Serial.printf("[heap] after Matter.begin(): free=%u min_free=%u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  for (size_t i = 0; i < kMaxMotion; i++) gMotion[i].setHoldTime(kMotionHoldTimeSeconds);

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("Matter Node not commissioned yet. Commission via the Alexa app (Add Device -> Matter).");
  } else {
    Serial.println("Matter Node already commissioned.");
  }

  static wyzesense::Dongle dongle([](const uint8_t *data, size_t len) {
    if (!g_bridgeConnected) return;
    // The bridge has only an interrupt IN endpoint (confirmed via lsusb -v
    // back on Linux and via this board's own enumeration log) -- no
    // interrupt OUT endpoint at all. sendHIDVendorOutput() requires one and
    // will always fail here; the real protocol's writes go out as a
    // control-transfer SET_REPORT instead (which is what a plain
    // os.write() to /dev/hidraw0 falls back to on Linux when there's no
    // OUT endpoint), so use the generic control-transfer path directly.
    usb.sendHIDReport(/*interfaceNumber=*/0, ESP_USB_HOST_HID_REPORT_TYPE_OUTPUT,
                       /*reportId=*/0, data, len, g_bridgeAddress);
  });
  g_dongle = &dongle;

  dongle.OnLog([](const char *msg) { Serial.printf("[dongle] %s\n", msg); });

  dongle.OnReady([]() {
    Serial.println("=== Dongle ready ===");
    Serial.printf("  MAC:     %s\n", g_dongle->MAC());
    Serial.printf("  Version: %s\n", g_dongle->Version());
    Serial.println("Commands: p (pair new sensor), factory-reset (wipe Matter state)");

    if (digitalRead(kBootButtonPin) == LOW) {
      Serial.println("BOOT button held at startup -- entering pairing mode now.");
      Serial.println("Trigger the new sensor's pairing broadcast (battery tab/reset pinhole).");
      g_dongle->StartScan(millis(), kScanTimeoutMs, onScanResult);
    }
  });

  dongle.OnSensorList([](const char (*macs)[9], size_t count) {
    Serial.printf("=== %u sensor(s) paired ===\n", (unsigned)count);
    for (size_t i = 0; i < count; i++) {
      Serial.printf("  - %s\n", macs[i]);
      // If this MAC already has a persisted slot from a previous boot,
      // reclaim it now so it's live immediately rather than waiting for a
      // fresh event. A MAC with no prior mapping stays unassigned until
      // its type is revealed, either by pairing (which carries the type
      // directly) or its first live event.
      String stored = prefs.getString(macs[i], "");
      if (stored.length() > 1) {
        if (stored[0] == 'C') AssignContactSlot(macs[i]);
        else if (stored[0] == 'M') AssignMotionSlot(macs[i]);
      }
    }
  });

  dongle.OnSensorEvent([](const wyzesense::SensorAlarm &e) {
    Serial.printf("=== SENSOR EVENT: mac=%s type=%s ", e.mac, SensorTypeName(e.type));
    if (e.type == wyzesense::SensorType::kLeakTemperature) {
      Serial.printf("temp=%d.%d", e.tempWhole, e.tempFrac);
    } else {
      Serial.printf("state=%s", e.boolState ? "open/active/wet" : "closed/inactive/dry");
    }
    Serial.printf(" battery=%u signal=%u ===\n", e.battery, e.signal);

    if (e.type == wyzesense::SensorType::kMotion) {
      int idx = AssignMotionSlot(e.mac);
      if (idx < 0) {
        Serial.println("[matter] no free Motion slot for this sensor");
        return;
      }
      // setOccupancy(true) auto-reverts to false after holdTime seconds
      // (native Matter OccupancySensing HoldTime feature) -- the sensor's
      // own alarm stream doesn't send an explicit "motion stopped" event,
      // so there's nothing to wire an explicit false transition to anyway.
      if (e.boolState) gMotion[idx].setOccupancy(true);
    } else {
      int idx = AssignContactSlot(e.mac);
      if (idx < 0) {
        Serial.println("[matter] no free Contact slot for this sensor");
        return;
      }
      // Polarity has flip-flopped once already during testing (see git
      // history/README) -- the direct-passthrough version looked right
      // from one post-reboot observation, but a cleaner, controlled test
      // (verified error-free send, app force-killed and reopened to rule
      // out UI caching) showed Alexa displaying "open" for a sensor we'd
      // just sent "closed" for, on both sensors independently. Inverting
      // to match MatterContactSensor.cpp's own documented convention
      // (setContact(true) = Closed) after all.
      gContact[idx].setContact(!e.boolState);
      prefsContactState.putBool(e.mac, e.boolState);
    }
  });

  usb.onDeviceConnected(onDeviceConnected);
  usb.onDeviceDisconnected(onDeviceDisconnected);
  usb.onHIDInput(onHIDInput);

  if (!usb.begin()) {
    Serial.println("usb.begin() FAILED");
  } else {
    Serial.println("USB host ready, waiting for bridge...");
  }
}

void loop() {
  uint32_t now = millis();

  if (g_handshakePending && (now - g_connectedAtMs) >= kHandshakeSettleMs) {
    g_handshakePending = false;
    Serial.println("  -> starting handshake");
    g_dongle->Start(now);
  }

  if (g_bridgeConnected && g_dongle) {
    g_dongle->Update(now);
  }

  static uint32_t lastPairingPrint = 0;
  if (!Matter.isDeviceCommissioned() && now - lastPairingPrint > 10000) {
    lastPairingPrint = now;
    Serial.printf("Not commissioned. Manual pairing code: %s | QR code URL: %s\n",
                  Matter.getManualPairingCode().c_str(), Matter.getOnboardingQRCodeUrl().c_str());
  }

  // Heap heartbeat: correlating this against the CASE-resumption/UDP-send
  // failures in the Matter/CHIP logs is what will confirm or rule out heap
  // pressure (this board has no PSRAM) as their root cause.
  static uint32_t lastHeapPrint = 0;
  if (now - lastHeapPrint > 2000) {
    lastHeapPrint = now;
    Serial.printf("[heap] t=%lu free=%u min_free=%u\n", (unsigned long)now, ESP.getFreeHeap(), ESP.getMinFreeHeap());
  }

  static String lineBuf;
  while (Serial.available() > 0) {
    char c = char(Serial.read());
    if (c == '\n') {
      handleSerialLine(lineBuf);
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
    }
  }

  delay(20);
}
