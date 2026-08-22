// Transport-agnostic port of gateway.py's Dongle class -- the startup
// handshake, sensor-list retrieval, and unsolicited notification handling
// (live sensor alarms, time sync). Deliberately has zero dependency on
// EspUsbHost or any Arduino header, so it can be driven by synthetic byte
// streams in a native unit test (test_native/test_dongle.cpp) before ever
// touching hardware. main.cpp wires it to the real USB transport.
//
// Not yet ported: Delete(). Sensors already paired to the physical bridge
// stay paired regardless of which host talks to it -- pairing state lives
// in the dongle's own memory -- so Start()+GetSensorList picks up sensors
// paired earlier via the Python tool with no extra work. StartScan() below
// covers pairing *new* sensors directly from the ESP32.
//
// Usage:
//   Dongle dongle([](const uint8_t *d, size_t n){ /* transmit via USB */ });
//   dongle.OnReady([&]{ ... });
//   dongle.OnSensorEvent([&](const SensorAlarm &e){ ... });
//   dongle.OnSensorList([&](const char (*macs)[9], size_t n){ ... });
//   dongle.Start(millis());
//   // in loop(): dongle.Update(millis());
//   // when USB data arrives: dongle.FeedBytes(data, len);
//   // to pair a new sensor once ready: dongle.StartScan(millis(), 60000, cb);

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include "wyzesense_protocol.h"

namespace wyzesense {

enum class DongleState {
  kIdle,
  kInquiry,
  kGetEnr,
  kGetMac,
  kGetVersion,
  kFinishAuth,
  kGetSensorCount,
  kGetSensorList,
  kReady,
  kFailed,
  kScanEnabling,
  kScanWaiting,
  kScanGettingR1,
  kScanDisabling,
  kScanVerifying,
};

constexpr size_t kMaxSensors = 16;

class Dongle {
 public:
  using SendFn = std::function<void(const uint8_t *data, size_t len)>;
  using ReadyFn = std::function<void()>;
  using SensorEventFn = std::function<void(const SensorAlarm &)>;
  using SensorListFn = std::function<void(const char (*macs)[9], size_t count)>;
  using LogFn = std::function<void(const char *msg)>;
  using ScanResultFn = std::function<void(bool found, const char *mac, uint8_t type, uint8_t version)>;

  explicit Dongle(SendFn sendFn) : send_(std::move(sendFn)) {}

  void OnReady(ReadyFn fn) { onReady_ = std::move(fn); }
  void OnSensorEvent(SensorEventFn fn) { onSensorEvent_ = std::move(fn); }
  void OnSensorList(SensorListFn fn) { onSensorList_ = std::move(fn); }
  void OnLog(LogFn fn) { onLog_ = std::move(fn); }

  // Kicks off Inquiry -> GetEnr -> GetMAC -> GetVersion -> FinishAuth ->
  // GetSensorCount -> GetSensorList. Call once the USB device is connected.
  void Start(uint32_t nowMs);

  // Feed newly-received raw bytes from the transport (HID vendor input
  // reports, one 64-byte report per call is fine -- frames spanning
  // multiple reports are reassembled internally). `nowMs` is used for the
  // sync-time-ack reply and is otherwise just cached.
  void FeedBytes(const uint8_t *data, size_t len, uint32_t nowMs);

  // Call regularly (e.g. every loop() iteration) to drive handshake
  // timeouts/retries.
  void Update(uint32_t nowMs);

  bool IsReady() const { return state_ == DongleState::kReady; }
  DongleState State() const { return state_; }
  const char *MAC() const { return mac_; }
  const char *Version() const { return version_; }

  // Puts the bridge into pairing mode: enables scanning, waits up to
  // timeoutMs for a new sensor to announce itself (this is when you
  // physically trigger the sensor's pairing broadcast -- battery
  // tab/reset pinhole), completes the key exchange + verification if one
  // is found, then disables scanning and calls cb with the result. No-op
  // (with a log message) if the dongle isn't IsReady() or a scan is
  // already in progress.
  void StartScan(uint32_t nowMs, uint32_t timeoutMs, ScanResultFn cb);

  // Ends an in-progress scan early, before its timeout -- same effect as
  // the scan timing out on its own (disables scanning, then calls the
  // ScanResultFn with found=false). No-op (with a log message) if no scan
  // is currently waiting for a sensor to announce itself.
  void CancelScan(uint32_t nowMs);

  bool IsScanning() const { return state_ == DongleState::kScanWaiting; }

 private:
  void SendPacket(const Packet &p);
  void HandlePacket(const Packet &p);
  void HandleUnsolicited(const Packet &p);
  void EnterState(DongleState s, uint32_t nowMs);
  void SendForState(DongleState s);
  void Log(const char *msg);

  SendFn send_;
  ReadyFn onReady_;
  SensorEventFn onSensorEvent_;
  SensorListFn onSensorList_;
  LogFn onLog_;

  DongleState state_ = DongleState::kIdle;
  uint32_t stateDeadlineMs_ = 0;
  uint8_t retriesLeft_ = 0;
  static constexpr uint32_t kCmdTimeoutMs = 2000;
  static constexpr uint8_t kMaxRetries = 3;

  uint8_t rxBuf_[512];
  size_t rxLen_ = 0;

  char mac_[9] = {};
  char version_[32] = {};
  uint8_t enr_[16] = {};

  uint8_t sensorCount_ = 0;
  uint8_t sensorsReceived_ = 0;
  char sensorMacs_[kMaxSensors][9] = {};

  uint32_t lastNowMs_ = 0;

  ScanResultFn onScanResult_;
  uint32_t scanTimeoutMs_ = 0;
  uint32_t scanDeadlineMs_ = 0;
  bool scanFound_ = false;
  char scanFoundMac_[9] = {};
  uint8_t scanFoundType_ = 0;
  uint8_t scanFoundVersion_ = 0;
};

}  // namespace wyzesense
