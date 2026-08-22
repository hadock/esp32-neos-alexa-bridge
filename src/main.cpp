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
// controller-side, set in the Alexa app after commissioning) -- there's no
// name registry here, nothing to push it to.
//
// WiFi credentials aren't hardcoded either: on first boot (or after a
// WiFi reset -- hold BOOT for 5s during normal operation), the device
// starts an open access point and setup portal (see RunWifiSetupPortal())
// instead of connecting anywhere. secrets.h, if present, is only a
// one-time seed for people who'd rather set it at build time.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Matter.h>
#include <EspUsbHost.h>
#include "wyzesense_dongle.h"

// secrets.h is now fully optional: it only serves as a one-time seed for
// people who prefer setting WiFi credentials at build time. Everyone else
// configures WiFi at runtime via the AP + setup-portal flow in
// connectWiFi() -- see RunWifiSetupPortal() below.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

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

// WiFi credentials, entered once through the setup portal (or seeded from
// secrets.h) and persisted here from then on.
static Preferences wifiPrefs;

// GPIO0 is the board's BOOT button -- only meaningful to the ROM bootloader
// very early in boot (for entering flash-download mode); once the app is
// running it's a free input pin. Held at startup: enters pairing mode once
// the dongle handshake completes. Held for kWifiResetHoldMs at any point
// during normal operation: wipes the stored WiFi credentials and reboots
// into the setup portal, for reconfiguring a device that's already
// mounted somewhere the USB port isn't reachable.
static constexpr int kBootButtonPin = 0;
static constexpr uint32_t kScanTimeoutMs = 60000;
static constexpr uint16_t kMotionHoldTimeSeconds = 8;
static constexpr uint32_t kWifiResetHoldMs = 5000;

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

static bool LoadStoredWifiCreds(String &ssid, String &pass) {
  if (!wifiPrefs.isKey("ssid")) return false;
  ssid = wifiPrefs.getString("ssid", "");
  pass = wifiPrefs.getString("pass", "");
  return ssid.length() > 0;
}

static void SaveWifiCreds(const String &ssid, const String &pass) {
  wifiPrefs.putString("ssid", ssid);
  wifiPrefs.putString("pass", pass);
}

static void ClearWifiCreds() {
  wifiPrefs.remove("ssid");
  wifiPrefs.remove("pass");
}

struct ScannedNetwork {
  String ssid;
  int32_t rssi = 0;
};
static constexpr size_t kMaxScannedNetworks = 20;
static ScannedNetwork gScannedNetworks[kMaxScannedNetworks];
static size_t gScannedCount = 0;

static void ScanNetworks() {
  Serial.println("Scanning nearby WiFi networks...");
  int n = WiFi.scanNetworks();
  gScannedCount = 0;
  for (int i = 0; i < n && gScannedCount < kMaxScannedNetworks; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    bool dup = false;
    for (size_t j = 0; j < gScannedCount; j++) {
      if (gScannedNetworks[j].ssid == ssid) {
        dup = true;
        if (WiFi.RSSI(i) > gScannedNetworks[j].rssi) gScannedNetworks[j].rssi = WiFi.RSSI(i);
        break;
      }
    }
    if (!dup) {
      gScannedNetworks[gScannedCount].ssid = ssid;
      gScannedNetworks[gScannedCount].rssi = WiFi.RSSI(i);
      gScannedCount++;
    }
  }
  // Strongest signal first (small N -- a plain insertion sort is plenty).
  for (size_t i = 1; i < gScannedCount; i++) {
    ScannedNetwork key = gScannedNetworks[i];
    size_t j = i;
    while (j > 0 && gScannedNetworks[j - 1].rssi < key.rssi) {
      gScannedNetworks[j] = gScannedNetworks[j - 1];
      j--;
    }
    gScannedNetworks[j] = key;
  }
  Serial.printf("Found %u network(s)\n", (unsigned)gScannedCount);
}

static constexpr const char *kApSsid = "esp_neos_bridge";
static const IPAddress kApIP(192, 168, 4, 1);

static String BuildPortalPage() {
  String html = F(
      "<!DOCTYPE html><html><head><title>NEOS Bridge Setup</title>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<style>body{font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em}"
      "select,input{width:100%;padding:.5em;margin:.3em 0 1em;font-size:1em;box-sizing:border-box}"
      "button{background:#2563eb;color:#fff;border:0;padding:.7em 1.2em;border-radius:6px;"
      "font-size:1em;width:100%}</style></head><body>"
      "<h2>NEOS Bridge -- WiFi Setup</h2>"
      "<form method='POST' action='/configure'>"
      "<label>Network</label><select name='ssid'>");
  for (size_t i = 0; i < gScannedCount; i++) {
    html += "<option value=\"" + gScannedNetworks[i].ssid + "\">" + gScannedNetworks[i].ssid +
            " (" + String(gScannedNetworks[i].rssi) + " dBm)</option>";
  }
  html += F(
      "</select>"
      "<label>Password (leave blank for open networks)</label>"
      "<input type='password' name='password' autocomplete='off'>"
      "<button type='submit'>Connect</button>"
      "</form></body></html>");
  return html;
}

// Blocking: runs until a network is configured through the portal, then
// reboots -- there's nothing else useful for setup() to do without WiFi
// credentials anyway. Starts an open (no password) access point at a fixed
// IP so the whole flow works with nothing but a browser: connect to
// "esp_neos_bridge", visit http://192.168.4.1/, pick a scanned network,
// enter its password, submit.
//
// The DNS server answers every query (any hostname at all) with our own
// IP, and the web server redirects any unrecognized path to "/". Together
// these are what make a phone/laptop's own "is there internet?" probe get
// intercepted and auto-pop its captive-portal sign-in view, the same way
// it would for hotel/coffee-shop WiFi -- no manual browser navigation
// needed on most devices.
static void RunWifiSetupPortal() {
  ScanNetworks();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(kApIP, kApIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(kApSsid);
  Serial.printf("WiFi not configured. Connect to \"%s\" (open network) and visit http://%s/\n",
                kApSsid, kApIP.toString().c_str());

  static DNSServer dnsServer;
  dnsServer.start(53, "*", kApIP);

  static WebServer portalServer(80);
  portalServer.on("/", HTTP_GET, [&]() {
    portalServer.send(200, "text/html", BuildPortalPage());
  });
  portalServer.on("/configure", HTTP_POST, [&]() {
    String ssid = portalServer.arg("ssid");
    String password = portalServer.arg("password");
    if (ssid.length() == 0) {
      portalServer.send(400, "text/html", "<p>No network selected. <a href='/'>Back</a></p>");
      return;
    }
    SaveWifiCreds(ssid, password);
    portalServer.send(200, "text/html",
                       "<!DOCTYPE html><html><body><p>Saved. Rebooting and connecting to \"" + ssid +
                           "\"...</p></body></html>");
    delay(500);
    ESP.restart();
  });
  portalServer.onNotFound([&]() {
    portalServer.sendHeader("Location", String("http://") + kApIP.toString() + "/");
    portalServer.send(302);
  });
  portalServer.begin();

  for (;;) {
    dnsServer.processNextRequest();
    portalServer.handleClient();
    delay(5);
  }
}

static void connectWiFi() {
  String ssid, pass;
  if (!LoadStoredWifiCreds(ssid, pass)) {
    // No stored credentials yet. secrets.h (if present) seeds them once, for
    // people who prefer setting WiFi at build time over the setup portal.
    if (strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "your-wifi-ssid") != 0) {
      ssid = WIFI_SSID;
      pass = WIFI_PASSWORD;
      SaveWifiCreds(ssid, pass);
      Serial.println("Seeded WiFi credentials from secrets.h into flash.");
    } else {
      RunWifiSetupPortal();  // never returns -- reboots once configured
    }
  }

  Serial.printf("Connecting to WiFi \"%s\"", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    // Deliberately not falling back to the setup portal here: a stored SSID
    // that's merely temporarily unreachable (router reboot, etc.) shouldn't
    // wipe good credentials. If they're genuinely wrong, hold BOOT for 5s
    // once the device is reachable via USB again to clear them.
    Serial.println("WiFi connect FAILED (will keep retrying in background) -- Alexa integration won't work until connected.");
  }
}

// Wipes the Matter fabric/ACL/subscription-resumption table so the node can
// be recommissioned clean. Useful after moving the device to a new Alexa
// account/location, after accumulating enough stale persisted subscriptions
// that Matter.begin() can no longer resume any of them without exhausting
// heap, or after the kind of subscription staleness a full power-cycle can
// trigger (see README's "Recovery" section) -- reachable both from the
// serial console and, once the device is mounted somewhere the USB port
// isn't reachable, from the web status page on port 8080.
static void PerformFactoryReset() {
  Serial.println("=== Factory-resetting Matter state and rebooting... ===");
  delay(100);
  Matter.decommission();
  delay(500);
  ESP.restart();
}

// Shared by the serial "p" command and the web UI's pairing button.
static void StartPairing() {
  if (g_dongle && g_dongle->IsReady()) {
    Serial.println("=== Pairing mode: trigger the new sensor's pairing broadcast now (60s window) ===");
    g_dongle->StartScan(millis(), kScanTimeoutMs, onScanResult);
  } else {
    Serial.println("Dongle not ready yet, can't start pairing.");
  }
}

// Ends pairing mode early. Harmless no-op if no scan is in progress.
static void StopPairing() {
  if (g_dongle) g_dongle->CancelScan(millis());
}

// Handles one line of serial input: "p" to pair a new sensor, "factory-reset"
// to wipe Matter commissioning/subscription state.
static void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "p" || line == "P") {
    StartPairing();
    return;
  }

  if (line == "factory-reset") {
    PerformFactoryReset();
    return;
  }

  Serial.println("Unknown command. Commands: p (pair new sensor), factory-reset (wipe Matter state)");
}

static WebServer webServer(8080);

// The page walks through setup in order rather than showing everything at
// once: wait for the USB bridge -> wait for the dongle handshake -> pair at
// least one sensor -> link with Alexa. Each later section only appears once
// the one before it is actually done, matching the real order that things
// need to happen in.
static String BuildStatusPage() {
  bool usbOk = g_bridgeConnected;
  bool dongleOk = g_dongle && g_dongle->IsReady();
  bool scanning = g_dongle && g_dongle->IsScanning();
  bool commissioned = Matter.isDeviceCommissioned();

  size_t pairedCount = 0;
  for (size_t i = 0; i < kMaxContact; i++) {
    if (gContactSlot[i].inUse) pairedCount++;
  }
  for (size_t i = 0; i < kMaxMotion; i++) {
    if (gMotionSlot[i].inUse) pairedCount++;
  }

  bool waitingOnHardware = !usbOk || !dongleOk;

  String html = "<!DOCTYPE html><html><head><title>NEOS Bridge</title>"
                "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  if (waitingOnHardware) {
    html += "<meta http-equiv='refresh' content='3'>";  // auto-refresh while just waiting
  }
  html += F(
      "<style>body{font-family:sans-serif;max-width:480px;margin:2em auto;padding:0 1em}"
      "table{width:100%;border-collapse:collapse}td,th{padding:.4em;border-bottom:1px solid #ddd;text-align:left}"
      "button{border:0;padding:.7em 1.2em;border-radius:6px;font-size:1em;margin-right:.5em}"
      ".danger{background:#b91c1c;color:#fff}.action{background:#2563eb;color:#fff}"
      ".dot{display:inline-block;width:.7em;height:.7em;border-radius:50%;margin-right:.4em}"
      ".dot-ok{background:#16a34a}.dot-bad{background:#dc2626}</style>"
      "</head><body><h2>NEOS/WyzeSense Bridge</h2>");

  if (!usbOk) {
    html += F(
        "<p><span class='dot dot-bad'></span><b>USB bridge:</b> not connected</p>"
        "<p>Connect a NEOS/WyzeSense USB bridge dongle to the board's native USB port to "
        "continue. This page refreshes automatically.</p></body></html>");
    return html;
  }

  if (!dongleOk) {
    html += F(
        "<p><span class='dot dot-ok'></span><b>USB bridge:</b> connected</p>"
        "<p><span class='dot dot-bad'></span><b>Dongle handshake:</b> not ready</p>"
        "<p>Talking to the bridge... this page refreshes automatically.</p></body></html>");
    return html;
  }

  html += F(
      "<p><span class='dot dot-ok'></span><b>USB bridge:</b> connected</p>"
      "<p><span class='dot dot-ok'></span><b>Dongle handshake:</b> ready</p>");
  html += "<p><b>WiFi:</b> " + WiFi.localIP().toString() + "</p>";
  html += "<p><b>Free heap:</b> " + String(ESP.getFreeHeap()) + " bytes (min " + String(ESP.getMinFreeHeap()) + ")</p>";

  if (pairedCount == 0) {
    html += F("<h3>Pair your first sensor</h3><p>No sensors paired yet.</p>");
  } else {
    html += F("<h3>Sensors</h3><table><tr><th>MAC</th><th>Type</th><th>Slot</th><th>Last known state</th></tr>");
    for (size_t i = 0; i < kMaxContact; i++) {
      if (!gContactSlot[i].inUse) continue;
      bool lastOpen = prefsContactState.isKey(gContactSlot[i].mac) && prefsContactState.getBool(gContactSlot[i].mac);
      html += "<tr><td>" + String(gContactSlot[i].mac) + "</td><td>contact</td><td>" + String(i) +
              "</td><td>" + (lastOpen ? "open" : "closed") + "</td></tr>";
    }
    for (size_t i = 0; i < kMaxMotion; i++) {
      if (!gMotionSlot[i].inUse) continue;
      html += "<tr><td>" + String(gMotionSlot[i].mac) + "</td><td>motion</td><td>" + String(i) + "</td><td>--</td></tr>";
    }
    html += F("</table>");
  }

  html += "<p><span class='dot ";
  html += scanning ? "dot-ok" : "dot-bad";
  html += "'></span><b>Pairing mode:</b> ";
  html += scanning ? "ACTIVE -- trigger the new sensor's pairing action now" : "inactive";
  html += "</p>";
  html += "<form style='display:inline' method='POST' action='/pair-start'><button type='submit' class='action'";
  if (scanning) html += " disabled";
  html += ">Start Pairing</button></form>";
  html += "<form style='display:inline' method='POST' action='/pair-stop'><button type='submit' class='action'";
  if (!scanning) html += " disabled";
  html += ">Stop Pairing</button></form>";

  if (commissioned) {
    html += F("<p><b>Matter:</b> commissioned</p>");
  } else if (pairedCount > 0) {
    html += "<h3>Link with Alexa</h3><p><b>Pairing code:</b> " + Matter.getManualPairingCode() + "<br>";
    html += "<a href=\"" + Matter.getOnboardingQRCodeUrl() + "\" target=\"_blank\">QR code link</a></p>";
  }

  html += F(
      "<h3>Recovery</h3>"
      "<p>Wipes Matter commissioning/subscription state and reboots. "
      "You'll need to remove the device from Alexa and re-add it afterward.</p>"
      "<form method='POST' action='/factory-reset' "
      "onsubmit=\"return confirm('Factory-reset the Matter node? "
      "You will need to recommission it in Alexa afterward.');\">"
      "<button type='submit' class='danger'>Factory Reset</button></form>"
      "</body></html>");
  return html;
}

// No authentication on this endpoint -- it trusts the local WiFi network as
// the security boundary (same trust level as the serial console it mirrors).
// Fine for a home network; don't port-forward this to the internet.
static void SetupWebServer() {
  webServer.on("/", HTTP_GET, []() {
    webServer.send(200, "text/html", BuildStatusPage());
  });

  webServer.on("/factory-reset", HTTP_POST, []() {
    webServer.send(200, "text/html", "<!DOCTYPE html><html><body><p>Factory-resetting and rebooting now...</p></body></html>");
    delay(200);  // let the response actually flush before the reboot cuts the connection
    PerformFactoryReset();
  });

  webServer.on("/pair-start", HTTP_POST, []() {
    StartPairing();
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/pair-stop", HTTP_POST, []() {
    StopPairing();
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.begin();
  Serial.println("Web status/recovery server listening on port 8080");
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
  wifiPrefs.begin("wz_wifi", /*readOnly=*/false);

  connectWiFi();
  Serial.printf("[heap] after WiFi connect: free=%u min_free=%u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  SetupWebServer();

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

  webServer.handleClient();

  // Hold BOOT for kWifiResetHoldMs at any point during normal operation to
  // wipe stored WiFi credentials and reboot into the setup portal -- for
  // reconfiguring a device that's already mounted somewhere the USB port
  // isn't reachable. Separate from the "BOOT held at startup" check in
  // OnReady(), which is a one-shot look at the pin, not a hold duration.
  static uint32_t bootHeldSinceMs = 0;
  static bool wifiResetTriggered = false;
  if (digitalRead(kBootButtonPin) == LOW) {
    if (bootHeldSinceMs == 0) bootHeldSinceMs = now;
    if (!wifiResetTriggered && (now - bootHeldSinceMs) >= kWifiResetHoldMs) {
      wifiResetTriggered = true;
      Serial.println("BOOT held 5s -- clearing WiFi credentials and rebooting into setup mode...");
      ClearWifiCreds();
      delay(200);
      ESP.restart();
    }
  } else {
    bootHeldSinceMs = 0;
    wifiResetTriggered = false;
  }

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
