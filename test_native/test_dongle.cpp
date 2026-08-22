// Native test for the Dongle state machine: drives a full startup
// handshake (Inquiry -> GetEnr -> GetMAC -> GetVersion -> FinishAuth ->
// GetSensorCount -> GetSensorList) with synthetic response frames, then
// feeds a real sensor-alarm frame whose checksum was independently
// computed by the actual Python `wyzesense` library (see
// test_protocol.cpp / the session transcript for how it was generated) so
// that part of the test isn't just checking our own code against itself.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "../lib/wyzesense_protocol/wyzesense_protocol.h"
#include "../lib/wyzesense_dongle/wyzesense_dongle.h"

using namespace wyzesense;

static int g_failures = 0;

static void Check(const char *name, bool cond) {
  if (cond) {
    printf("PASS  %s\n", name);
  } else {
    printf("FAIL  %s\n", name);
    g_failures++;
  }
}

// Builds a raw response frame for `cmd` with the given payload, exactly as
// Dongle::FeedBytes expects to receive it (sync marker order doesn't
// matter -- ParsePacket accepts either).
static std::vector<uint8_t> BuildResponse(uint16_t cmd, const uint8_t *payload, size_t len) {
  Packet p;
  p.cmd = cmd;
  p.SetPayload(payload, len);
  uint8_t buf[80];
  size_t n = p.Encode(buf, sizeof(buf));
  return std::vector<uint8_t>(buf, buf + n);
}

int main() {
  std::vector<std::vector<uint8_t>> sent;
  Dongle dongle([&](const uint8_t *d, size_t n) {
    sent.emplace_back(d, d + n);
  });

  bool ready = false;
  char readyMac[9] = {};
  char readyVersion[32] = {};
  std::vector<std::string> listedMacs;
  std::vector<SensorAlarm> events;

  dongle.OnReady([&] {
    ready = true;
    strncpy(readyMac, dongle.MAC(), 8);
    strncpy(readyVersion, dongle.Version(), 31);
  });
  dongle.OnSensorList([&](const char (*macs)[9], size_t count) {
    for (size_t i = 0; i < count; i++) listedMacs.emplace_back(macs[i]);
  });
  dongle.OnSensorEvent([&](const SensorAlarm &e) { events.push_back(e); });
  dongle.OnLog([](const char *msg) { printf("  log: %s\n", msg); });

  uint32_t t = 0;
  dongle.Start(t);
  Check("Start() sends Inquiry request", sent.size() == 1);

  auto feed = [&](std::vector<uint8_t> frame) { dongle.FeedBytes(frame.data(), frame.size(), t); };

  // Inquiry response: 1 byte, value 1 == success.
  uint8_t inquiryOk = 1;
  feed(BuildResponse(CMD_INQUIRY + 1, &inquiryOk, 1));
  Check("advances to GetEnr after Inquiry", sent.size() == 2);

  // GetEnr response: 16 bytes (arbitrary, just needs to be 16).
  uint8_t enr[16];
  memset(enr, 0xAB, 16);
  feed(BuildResponse(CMD_GET_ENR + 1, enr, 16));
  Check("advances to GetMAC after GetEnr", sent.size() == 3);

  // GetMAC response: 8 ascii chars -- reuse the real gateway MAC we saw live.
  const uint8_t mac[8] = {'E', 'F', '0', '4', '4', '3', '0', '8'};
  feed(BuildResponse(CMD_GET_MAC + 1, mac, 8));
  Check("advances to GetVersion after GetMAC", sent.size() == 4);

  // GetVersion response -- reuse the real version string we saw live.
  const char *versionStr = "2.0.0.22 V1.4 Dongle UD3A";
  feed(BuildResponse(CMD_GET_DONGLE_VERSION + 1, reinterpret_cast<const uint8_t *>(versionStr), strlen(versionStr)));
  // GetVersion's response cmd is in the TYPE_ASYNC space, so it also
  // triggers an ASYNC_ACK (matching gateway.py's universal ack-on-any-async
  // rule) in addition to the FinishAuth request: +1 ack, +1 request.
  Check("advances to FinishAuth after GetVersion", sent.size() == 6);

  // FinishAuth response: empty payload.
  feed(BuildResponse(CMD_FINISH_AUTH + 1, nullptr, 0));
  // Same deal: FinishAuth's response is TYPE_ASYNC too. +1 ack, +1 request.
  Check("advances to GetSensorCount after FinishAuth", sent.size() == 8);

  // GetSensorCount response: 2 sensors paired.
  uint8_t count = 2;
  feed(BuildResponse(CMD_GET_SENSOR_COUNT + 1, &count, 1));
  // Same deal again: GetSensorCount's response is TYPE_ASYNC. +1 ack, +1 request.
  Check("advances to GetSensorList after GetSensorCount", sent.size() == 10);
  Check("not ready yet (waiting on sensor list)", !ready);

  // GetSensorList responses: one packet per sensor, same cmd each time --
  // reuse the two real contact-sensor MACs paired earlier in this project.
  const uint8_t sensorA[8] = {'E', 'F', '0', '2', 'F', 'F', '4', '9'};
  const uint8_t sensorB[8] = {'E', 'F', '0', '2', 'F', 'F', 'D', '5'};
  feed(BuildResponse(CMD_GET_SENSOR_LIST + 1, sensorA, 8));
  Check("still not ready after 1 of 2 sensors", !ready);
  feed(BuildResponse(CMD_GET_SENSOR_LIST + 1, sensorB, 8));

  Check("ready after full handshake", ready);
  Check("IsReady() true", dongle.IsReady());
  Check("MAC matches", strcmp(readyMac, "EF044308") == 0);
  Check("Version matches", strcmp(readyVersion, "2.0.0.22 V1.4 Dongle UD3A") == 0);
  Check("sensor list has 2 entries", listedMacs.size() == 2);
  Check("sensor list contains EF02FF49", listedMacs.size() > 0 && listedMacs[0] == "EF02FF49");
  Check("sensor list contains EF02FFD5", listedMacs.size() > 1 && listedMacs[1] == "EF02FFD5");

  // --- Live sensor alarm, using the frame independently checksummed by
  // the real Python library (see test_protocol.cpp generation process). ---
  size_t sentBeforeAlarm = sent.size();
  const char *alarmHex = "55aa531d19000001989e26ce00a2454630324646343901005a00000100002406bb";
  std::vector<uint8_t> alarmFrame;
  for (size_t i = 0; i + 1 < strlen(alarmHex); i += 2) {
    char byteStr[3] = {alarmHex[i], alarmHex[i + 1], 0};
    alarmFrame.push_back(uint8_t(strtol(byteStr, nullptr, 16)));
  }
  feed(alarmFrame);

  Check("sensor event decoded", events.size() == 1);
  if (!events.empty()) {
    const SensorAlarm &e = events[0];
    Check("alarm MAC matches", strcmp(e.mac, "EF02FF49") == 0);
    Check("alarm type is switch", e.type == SensorType::kSwitch);
    Check("alarm state is open (true)", e.boolState == true);
    Check("alarm battery == 90", e.battery == 90);
    Check("alarm signal == 36", e.signal == 36);
  }
  Check("alarm triggered an ASYNC_ACK send", sent.size() == sentBeforeAlarm + 1);

  // --- StartScan(): sensor found path ---
  bool scanCbCalled = false;
  bool scanFound = false;
  std::string scanMac;
  uint8_t scanType = 0, scanVersion = 0;

  dongle.StartScan(t, 5000, [&](bool found, const char *mac, uint8_t type, uint8_t version) {
    scanCbCalled = true;
    scanFound = found;
    scanMac = mac;
    scanType = type;
    scanVersion = version;
  });

  uint8_t scanEnableOk = 1;
  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));
  Check("scan not yet resolved after EnableScan ack", !scanCbCalled);

  // NOTIFY_SENSOR_SCAN payload: [seq byte][8-byte mac][type][version]
  {
    uint8_t payload[11] = {0, 'E', 'F', '0', '3', '9', '9', '9', '9', 1, 16};
    feed(BuildResponse(NOTIFY_SENSOR_SCAN, payload, sizeof(payload)));
  }
  Check("scan not yet resolved after sensor announce", !scanCbCalled);

  uint8_t r1resp[16];
  memset(r1resp, 0xCD, 16);
  feed(BuildResponse(CMD_GET_SENSOR_R1 + 1, r1resp, 16));
  Check("scan not yet resolved after GetSensorR1", !scanCbCalled);

  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));  // DisableScan response
  Check("scan not yet resolved after DisableScan", !scanCbCalled);

  uint8_t verifyResp[9] = {'E', 'F', '0', '3', '9', '9', '9', '9', 0xFF};
  feed(BuildResponse(CMD_VERIFY_SENSOR + 1, verifyResp, 9));

  Check("scan callback fired (found path)", scanCbCalled);
  Check("scan found a sensor", scanFound);
  Check("scan found MAC matches", scanMac == "EF039999");
  Check("scan found type matches", scanType == 1);
  Check("scan found version matches", scanVersion == 16);
  Check("dongle ready again after scan", dongle.IsReady());

  // --- StartScan(): timeout / not-found path ---
  bool scan2CbCalled = false;
  bool scan2Found = true;  // start true so a missed callback would show up as a failure below

  dongle.StartScan(t, 100, [&](bool found, const char *, uint8_t, uint8_t) {
    scan2CbCalled = true;
    scan2Found = found;
  });
  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));  // EnableScan response
  Check("scan2 not yet resolved after EnableScan ack", !scan2CbCalled);

  dongle.Update(t + 250);  // past the 100ms scan window -> should time out
  Check("scan2 not yet resolved right at timeout (DisableScan still pending)", !scan2CbCalled);

  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));  // DisableScan response
  Check("scan2 callback fired (timeout path)", scan2CbCalled);
  Check("scan2 found nothing", !scan2Found);
  Check("dongle ready again after scan2 timeout", dongle.IsReady());

  // --- CancelScan(): early cancel, well before the timeout ---
  bool scan3CbCalled = false;
  bool scan3Found = true;  // start true so a missed callback would show up as a failure below

  dongle.StartScan(t, 60000, [&](bool found, const char *, uint8_t, uint8_t) {
    scan3CbCalled = true;
    scan3Found = found;
  });
  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));  // EnableScan response
  Check("IsScanning() true once waiting for a sensor", dongle.IsScanning());
  Check("scan3 not yet resolved after EnableScan ack", !scan3CbCalled);

  dongle.CancelScan(t + 5);  // cancel almost immediately, nowhere near the 60s timeout
  Check("IsScanning() false once cancelled", !dongle.IsScanning());
  Check("scan3 not yet resolved right after CancelScan (DisableScan still pending)", !scan3CbCalled);

  feed(BuildResponse(CMD_START_STOP_SCAN + 1, &scanEnableOk, 1));  // DisableScan response
  Check("scan3 callback fired (cancel path)", scan3CbCalled);
  Check("scan3 found nothing", !scan3Found);
  Check("dongle ready again after scan3 cancel", dongle.IsReady());

  // CancelScan() outside of an active scan should be a harmless no-op.
  dongle.CancelScan(t + 10);
  Check("dongle still ready after a no-op CancelScan()", dongle.IsReady());

  if (g_failures == 0) {
    printf("\nAll dongle tests passed.\n");
  } else {
    printf("\n%d dongle test(s) FAILED.\n", g_failures);
  }
  return g_failures == 0 ? 0 : 1;
}
