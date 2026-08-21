// Port of the WyzeSense USB bridge wire protocol (framing, checksums,
// pairing handshake, sensor-event decoding), reverse-engineered by hclxing
// and implemented in the `wyzesense` Python package (gateway.py). This
// header is plain C++ with no Arduino/ESP-IDF dependency so it can be
// unit-tested natively on a desktop compiler before ever touching hardware
// -- see test_native/.
//
// Wire framing (verified byte-for-byte against the real Python library --
// see test_native/test_protocol.cpp):
//   Host -> dongle frames start with bytes [0xAA, 0x55].
//   Dongle -> host frames start with bytes [0x55, 0xAA] (reverse order).
//   Parsing accepts either order defensively, matching the reference impl.
//
//   [sync0][sync1][cmd_type][b2][cmd_id][...payload...][cksum_hi][cksum_lo]
//
// For a normal (non-ACK) packet: b2 = payloadLen + 3, cmd_id = cmd & 0xFF.
// For an ASYNC_ACK "packet": b2 = ackedCmd & 0xFF, cmd_id = 0xFF (ASYNC_ACK's
// own low byte). The checksum is a plain sum of all preceding bytes & 0xFFFF
// (not a CRC), stored big-endian.
//
// Response convention: the reply to a request with command X arrives as
// command X+1 (see gateway.py's _DoCommand, which registers a handler for
// pkt.Cmd + 1).

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace wyzesense {

constexpr uint8_t TYPE_SYNC = 0x43;
constexpr uint8_t TYPE_ASYNC = 0x53;

constexpr uint16_t MakeCmd(uint8_t type, uint8_t cmd) {
  return (uint16_t(type) << 8) | cmd;
}

// Sync packets -- commands initiated from the host side.
constexpr uint16_t CMD_GET_ENR = MakeCmd(TYPE_SYNC, 0x02);
constexpr uint16_t CMD_GET_MAC = MakeCmd(TYPE_SYNC, 0x04);
constexpr uint16_t CMD_GET_KEY = MakeCmd(TYPE_SYNC, 0x06);
constexpr uint16_t CMD_INQUIRY = MakeCmd(TYPE_SYNC, 0x27);
constexpr uint16_t CMD_UPDATE_CC1310 = MakeCmd(TYPE_SYNC, 0x12);
constexpr uint16_t CMD_SET_CH554_UPGRADE = MakeCmd(TYPE_SYNC, 0x0E);

// Async packets.
constexpr uint16_t ASYNC_ACK = MakeCmd(TYPE_ASYNC, 0xFF);

// Commands initiated from the dongle side (host still sends the request).
constexpr uint16_t CMD_FINISH_AUTH = MakeCmd(TYPE_ASYNC, 0x14);
constexpr uint16_t CMD_GET_DONGLE_VERSION = MakeCmd(TYPE_ASYNC, 0x16);
constexpr uint16_t CMD_START_STOP_SCAN = MakeCmd(TYPE_ASYNC, 0x1C);
constexpr uint16_t CMD_GET_SENSOR_R1 = MakeCmd(TYPE_ASYNC, 0x21);
constexpr uint16_t CMD_VERIFY_SENSOR = MakeCmd(TYPE_ASYNC, 0x23);
constexpr uint16_t CMD_DEL_SENSOR = MakeCmd(TYPE_ASYNC, 0x25);
constexpr uint16_t CMD_GET_SENSOR_COUNT = MakeCmd(TYPE_ASYNC, 0x2E);
constexpr uint16_t CMD_GET_SENSOR_LIST = MakeCmd(TYPE_ASYNC, 0x30);

// Notifications initiated from the dongle side (unsolicited).
constexpr uint16_t NOTIFY_SENSOR_ALARM = MakeCmd(TYPE_ASYNC, 0x19);
constexpr uint16_t NOTIFY_SENSOR_SCAN = MakeCmd(TYPE_ASYNC, 0x20);
constexpr uint16_t NOTIFY_SYNC_TIME = MakeCmd(TYPE_ASYNC, 0x32);
constexpr uint16_t NOTIFY_EVENT_LOG = MakeCmd(TYPE_ASYNC, 0x35);

constexpr size_t MAX_PAYLOAD = 64;  // real payloads are well under this

uint16_t Checksum(const uint8_t *data, size_t len);

struct Packet {
  uint16_t cmd = 0;
  uint8_t payload[MAX_PAYLOAD] = {};
  size_t payloadLen = 0;

  // ASYNC_ACK packets carry an acked-command int instead of a byte payload
  // in the Python model; mirrored here as a separate field.
  bool isAck = false;
  uint16_t ackedCmd = 0;

  void SetPayload(const uint8_t *data, size_t len) {
    payloadLen = (len > MAX_PAYLOAD) ? MAX_PAYLOAD : len;
    if (data && payloadLen) memcpy(payload, data, payloadLen);
  }

  // Encodes this packet into `out` (capacity outCap). Returns the number of
  // bytes written, or 0 if it wouldn't fit.
  size_t Encode(uint8_t *out, size_t outCap) const;
};

// --- Builders, mirroring the Packet.classmethod constructors in gateway.py ---
Packet MakeInquiry();
Packet MakeGetMAC();
Packet MakeGetVersion();
Packet MakeGetKey();
Packet MakeEnableScan();
Packet MakeDisableScan();
Packet MakeFinishAuth();
Packet MakeGetEnr(const uint8_t r[16]);
Packet MakeGetSensorR1(const char mac[8], const uint8_t r[16]);
Packet MakeVerifySensor(const char mac[8]);
Packet MakeDelSensor(const char mac[8]);
Packet MakeGetSensorCount();
Packet MakeGetSensorList(uint8_t count);
Packet MakeAsyncAck(uint16_t ackedCmd);
Packet MakeSyncTimeAck(uint64_t nowMs);

// Parses one packet starting at buf[0].
//   > 0  -- success; return value is the number of bytes consumed, `out` is
//           filled in.
//   == 0 -- not enough data yet (buf doesn't contain a full frame); caller
//           should wait for more bytes and retry with the same buf[0].
//   == -1 -- invalid sync marker or checksum mismatch at buf[0]; caller
//            should skip forward and retry (this deliberately skips only 1
//            byte at a time for more thorough resync than the reference
//            implementation's 2-byte skip).
int ParsePacket(const uint8_t *buf, size_t len, Packet &out);

// --- Sensor alarm (state event) decoding, mirroring Dongle._OnSensorAlarm ---

enum class SensorType { kSwitch, kMotion, kLeak, kLeakTemperature, kUnknown };

struct SensorAlarm {
  char mac[9] = {};         // 8 chars + NUL
  uint64_t timestampMs = 0;
  SensorType type = SensorType::kUnknown;
  bool boolState = false;      // for switch/motion/leak: open|active|wet == true
  int tempWhole = 0;           // for leak:temperature
  int tempFrac = 0;
  uint8_t battery = 0;
  uint8_t signal = 0;
  uint8_t rawEventType = 0;    // set when type == kUnknown, for logging
  bool valid = false;
};

// Decodes a NOTIFY_SENSOR_ALARM packet's payload. Returns false if the
// payload is too short to be a valid alarm (caller should just log/ignore).
bool DecodeSensorAlarm(const uint8_t *payload, size_t len, SensorAlarm &out);

}  // namespace wyzesense
