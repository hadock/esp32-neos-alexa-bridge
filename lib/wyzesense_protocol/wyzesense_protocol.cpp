#include "wyzesense_protocol.h"

namespace wyzesense {

uint16_t Checksum(const uint8_t *data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return uint16_t(sum & 0xFFFF);
}

size_t Packet::Encode(uint8_t *out, size_t outCap) const {
  size_t pos = 0;
  auto put8 = [&](uint8_t b) {
    if (pos < outCap) out[pos] = b;
    pos++;
  };

  put8(0xAA);
  put8(0x55);
  put8(uint8_t(cmd >> 8));
  if (isAck) {
    put8(uint8_t(ackedCmd & 0xFF));
    put8(uint8_t(cmd & 0xFF));
  } else {
    put8(uint8_t(payloadLen + 3));
    put8(uint8_t(cmd & 0xFF));
    for (size_t i = 0; i < payloadLen; i++) put8(payload[i]);
  }

  if (pos > outCap) return 0;
  uint16_t cs = Checksum(out, pos);
  put8(uint8_t(cs >> 8));
  put8(uint8_t(cs & 0xFF));
  if (pos > outCap) return 0;
  return pos;
}

Packet MakeInquiry() {
  Packet p;
  p.cmd = CMD_INQUIRY;
  return p;
}

Packet MakeGetMAC() {
  Packet p;
  p.cmd = CMD_GET_MAC;
  return p;
}

Packet MakeGetVersion() {
  Packet p;
  p.cmd = CMD_GET_DONGLE_VERSION;
  return p;
}

Packet MakeGetKey() {
  Packet p;
  p.cmd = CMD_GET_KEY;
  return p;
}

Packet MakeEnableScan() {
  Packet p;
  p.cmd = CMD_START_STOP_SCAN;
  uint8_t one = 0x01;
  p.SetPayload(&one, 1);
  return p;
}

Packet MakeDisableScan() {
  Packet p;
  p.cmd = CMD_START_STOP_SCAN;
  uint8_t zero = 0x00;
  p.SetPayload(&zero, 1);
  return p;
}

Packet MakeFinishAuth() {
  Packet p;
  p.cmd = CMD_FINISH_AUTH;
  uint8_t ff = 0xFF;
  p.SetPayload(&ff, 1);
  return p;
}

Packet MakeGetEnr(const uint8_t r[16]) {
  Packet p;
  p.cmd = CMD_GET_ENR;
  p.SetPayload(r, 16);
  return p;
}

Packet MakeGetSensorR1(const char mac[8], const uint8_t r[16]) {
  Packet p;
  p.cmd = CMD_GET_SENSOR_R1;
  uint8_t buf[24];
  memcpy(buf, mac, 8);
  memcpy(buf + 8, r, 16);
  p.SetPayload(buf, 24);
  return p;
}

Packet MakeVerifySensor(const char mac[8]) {
  Packet p;
  p.cmd = CMD_VERIFY_SENSOR;
  uint8_t buf[10];
  memcpy(buf, mac, 8);
  buf[8] = 0xFF;
  buf[9] = 0x04;
  p.SetPayload(buf, 10);
  return p;
}

Packet MakeDelSensor(const char mac[8]) {
  Packet p;
  p.cmd = CMD_DEL_SENSOR;
  p.SetPayload(reinterpret_cast<const uint8_t *>(mac), 8);
  return p;
}

Packet MakeGetSensorCount() {
  Packet p;
  p.cmd = CMD_GET_SENSOR_COUNT;
  return p;
}

Packet MakeGetSensorList(uint8_t count) {
  Packet p;
  p.cmd = CMD_GET_SENSOR_LIST;
  p.SetPayload(&count, 1);
  return p;
}

Packet MakeAsyncAck(uint16_t ackedCmd) {
  Packet p;
  p.cmd = ASYNC_ACK;
  p.isAck = true;
  p.ackedCmd = ackedCmd;
  return p;
}

Packet MakeSyncTimeAck(uint64_t nowMs) {
  Packet p;
  p.cmd = NOTIFY_SYNC_TIME + 1;
  uint8_t buf[8];
  for (int i = 0; i < 8; i++) buf[i] = uint8_t(nowMs >> (8 * (7 - i)));
  p.SetPayload(buf, 8);
  return p;
}

static bool HasSyncMarker(const uint8_t *buf) {
  return (buf[0] == 0x55 && buf[1] == 0xAA) ||
         (buf[0] == 0xAA && buf[1] == 0x55);
}

int ParsePacket(const uint8_t *buf, size_t len, Packet &out) {
  if (len < 5) return 0;  // need more data before we can even read the header
  if (!HasSyncMarker(buf)) return -1;

  uint8_t cmdType = buf[2];
  uint8_t b2 = buf[3];
  uint8_t cmdId = buf[4];
  uint16_t cmd = MakeCmd(cmdType, cmdId);

  size_t consumed;
  if (cmd == ASYNC_ACK) {
    consumed = 7;
    if (len < consumed) return 0;
    out.isAck = true;
    out.cmd = cmd;
    out.ackedCmd = MakeCmd(cmdType, b2);
    out.payloadLen = 0;
  } else {
    consumed = size_t(b2) + 4;
    if (len < consumed) return 0;
    if (consumed < 7) return -1;  // b2 must be >= 3 for a well-formed frame
    out.isAck = false;
    out.cmd = cmd;
    out.SetPayload(buf + 5, consumed - 7);
  }

  uint16_t csRemote = (uint16_t(buf[consumed - 2]) << 8) | buf[consumed - 1];
  uint16_t csLocal = Checksum(buf, consumed - 2);
  if (csRemote != csLocal) return -1;

  return int(consumed);
}

bool DecodeSensorAlarm(const uint8_t *payload, size_t len, SensorAlarm &out) {
  out = SensorAlarm{};
  if (len < 18) return false;

  // ">QB8s": 8-byte big-endian ms timestamp, 1-byte event type, 8-byte ascii MAC.
  uint64_t ts = 0;
  for (int i = 0; i < 8; i++) ts = (ts << 8) | payload[i];
  uint8_t eventType = payload[8];
  memcpy(out.mac, payload + 9, 8);
  out.mac[8] = '\0';

  const uint8_t *alarm = payload + 17;
  size_t alarmLen = len - 17;
  out.timestampMs = ts;
  out.rawEventType = eventType;

  if (eventType == 0xA2 && alarmLen >= 9) {
    switch (alarm[0]) {
      case 0x01: out.type = SensorType::kSwitch; break;
      case 0x02: out.type = SensorType::kMotion; break;
      case 0x03: out.type = SensorType::kLeak; break;
      default: out.type = SensorType::kUnknown; break;
    }
    out.boolState = (alarm[5] == 1);
    out.battery = alarm[2];
    out.signal = alarm[8];
    out.valid = true;
  } else if (eventType == 0xE8 && alarmLen >= 9 && alarm[0] == 0x03) {
    out.type = SensorType::kLeakTemperature;
    out.tempWhole = alarm[5];
    out.tempFrac = alarm[6];
    out.battery = alarm[2];
    out.signal = alarm[8];
    out.valid = true;
  } else {
    out.type = SensorType::kUnknown;
    out.valid = true;  // valid frame, just an event type we don't decode further
  }

  return true;
}

}  // namespace wyzesense
