#include "wyzesense_dongle.h"

#include <cstring>

namespace wyzesense {

void Dongle::Log(const char *msg) {
  if (onLog_) onLog_(msg);
}

void Dongle::SendPacket(const Packet &p) {
  uint8_t buf[80];
  size_t n = p.Encode(buf, sizeof(buf));
  if (n == 0 || n > sizeof(buf)) {
    Log("SendPacket: encode overflow");
    return;
  }
  if (send_) send_(buf, n);
}

void Dongle::SendForState(DongleState s) {
  switch (s) {
    case DongleState::kInquiry:
      SendPacket(MakeInquiry());
      break;
    case DongleState::kGetEnr: {
      uint8_t r[16];
      memset(r, '0', 16);
      SendPacket(MakeGetEnr(r));
      break;
    }
    case DongleState::kGetMac:
      SendPacket(MakeGetMAC());
      break;
    case DongleState::kGetVersion:
      SendPacket(MakeGetVersion());
      break;
    case DongleState::kFinishAuth:
      SendPacket(MakeFinishAuth());
      break;
    case DongleState::kGetSensorCount:
      SendPacket(MakeGetSensorCount());
      break;
    case DongleState::kGetSensorList:
      sensorsReceived_ = 0;
      if (sensorCount_ == 0) {
        // Nothing to wait for; finish immediately.
        if (onSensorList_) onSensorList_(sensorMacs_, 0);
        state_ = DongleState::kReady;
        if (onReady_) onReady_();
      } else {
        SendPacket(MakeGetSensorList(sensorCount_));
      }
      break;
    case DongleState::kScanEnabling:
      SendPacket(MakeEnableScan());
      break;
    case DongleState::kScanWaiting:
      break;  // nothing to send; just waiting for NOTIFY_SENSOR_SCAN
    case DongleState::kScanGettingR1: {
      // Fixed key used by the reference implementation for this exchange.
      const uint8_t key[16] = {'O', 'k', '5', 'H', 'P', 'N', 'Q', '4',
                                'l', 'f', '7', '7', 'u', '7', '5', '4'};
      SendPacket(MakeGetSensorR1(scanFoundMac_, key));
      break;
    }
    case DongleState::kScanDisabling:
      SendPacket(MakeDisableScan());
      break;
    case DongleState::kScanVerifying:
      SendPacket(MakeVerifySensor(scanFoundMac_));
      break;
    default:
      break;
  }
}

void Dongle::StartScan(uint32_t nowMs, uint32_t timeoutMs, ScanResultFn cb) {
  if (state_ != DongleState::kReady) {
    Log("Dongle: StartScan() ignored, dongle not ready or scan already in progress");
    return;
  }
  onScanResult_ = std::move(cb);
  scanFound_ = false;
  scanFoundMac_[0] = '\0';
  scanTimeoutMs_ = timeoutMs;
  EnterState(DongleState::kScanEnabling, nowMs);
}

void Dongle::CancelScan(uint32_t nowMs) {
  if (state_ != DongleState::kScanWaiting) {
    Log("Dongle: CancelScan() ignored, no scan waiting for a sensor");
    return;
  }
  Log("Dongle: scan cancelled");
  scanFound_ = false;
  EnterState(DongleState::kScanDisabling, nowMs);
}

void Dongle::EnterState(DongleState s, uint32_t nowMs) {
  state_ = s;
  stateDeadlineMs_ = nowMs + kCmdTimeoutMs;
  retriesLeft_ = kMaxRetries;
  SendForState(s);
}

void Dongle::Start(uint32_t nowMs) {
  sensorCount_ = 0;
  sensorsReceived_ = 0;
  EnterState(DongleState::kInquiry, nowMs);
}

void Dongle::Update(uint32_t nowMs) {
  lastNowMs_ = nowMs;

  if (state_ == DongleState::kScanWaiting) {
    // Waiting for an unsolicited NOTIFY_SENSOR_SCAN (the sensor announcing
    // itself), not a response to something we sent -- a much longer,
    // user-supplied timeout, and nothing to retry-send in the meantime.
    if (int32_t(nowMs - scanDeadlineMs_) >= 0) {
      Log("Dongle: scan timed out, no sensor found");
      scanFound_ = false;
      EnterState(DongleState::kScanDisabling, nowMs);
    }
    return;
  }

  switch (state_) {
    case DongleState::kIdle:
    case DongleState::kReady:
    case DongleState::kFailed:
      return;
    default:
      break;
  }

  if (int32_t(nowMs - stateDeadlineMs_) < 0) return;  // not due yet

  if (retriesLeft_ == 0) {
    Log("Dongle: handshake step timed out, giving up");
    state_ = DongleState::kFailed;
    return;
  }
  retriesLeft_--;
  Log("Dongle: handshake step timed out, retrying");
  stateDeadlineMs_ = nowMs + kCmdTimeoutMs;
  SendForState(state_);
}

void Dongle::FeedBytes(const uint8_t *data, size_t len, uint32_t nowMs) {
  lastNowMs_ = nowMs;

  if (rxLen_ + len > sizeof(rxBuf_)) {
    Log("Dongle: rx buffer overflow, dropping buffered data");
    rxLen_ = 0;
  }
  size_t toCopy = len;
  if (rxLen_ + toCopy > sizeof(rxBuf_)) toCopy = sizeof(rxBuf_) - rxLen_;
  memcpy(rxBuf_ + rxLen_, data, toCopy);
  rxLen_ += toCopy;

  for (;;) {
    Packet pkt;
    int consumed = ParsePacket(rxBuf_, rxLen_, pkt);
    if (consumed == 0) {
      break;  // need more data
    } else if (consumed < 0) {
      // Bad sync/checksum at the front -- drop one byte and resync.
      memmove(rxBuf_, rxBuf_ + 1, rxLen_ - 1);
      rxLen_ -= 1;
      continue;
    } else {
      memmove(rxBuf_, rxBuf_ + consumed, rxLen_ - size_t(consumed));
      rxLen_ -= size_t(consumed);
      HandlePacket(pkt);
    }
  }
}

void Dongle::HandlePacket(const Packet &p) {
  // Mirrors _HandlePacket in gateway.py: any received async command (not
  // itself an ack) gets acked immediately, regardless of whether it's also
  // the response we're waiting on.
  if (!p.isAck && (p.cmd >> 8) == TYPE_ASYNC && p.cmd != ASYNC_ACK) {
    SendPacket(MakeAsyncAck(p.cmd));
  }

  bool consumed = false;

  switch (state_) {
    case DongleState::kInquiry:
      if (p.cmd == CMD_INQUIRY + 1) {
        consumed = true;
        if (p.payloadLen == 1 && p.payload[0] == 1) {
          EnterState(DongleState::kGetEnr, lastNowMs_);
        } else {
          Log("Dongle: Inquiry failed");
          state_ = DongleState::kFailed;
        }
      }
      break;

    case DongleState::kGetEnr:
      if (p.cmd == CMD_GET_ENR + 1) {
        consumed = true;
        if (p.payloadLen == 16) {
          memcpy(enr_, p.payload, 16);
          EnterState(DongleState::kGetMac, lastNowMs_);
        } else {
          Log("Dongle: GetEnr bad payload");
          state_ = DongleState::kFailed;
        }
      }
      break;

    case DongleState::kGetMac:
      if (p.cmd == CMD_GET_MAC + 1) {
        consumed = true;
        if (p.payloadLen == 8) {
          memcpy(mac_, p.payload, 8);
          mac_[8] = '\0';
          EnterState(DongleState::kGetVersion, lastNowMs_);
        } else {
          Log("Dongle: GetMAC bad payload");
          state_ = DongleState::kFailed;
        }
      }
      break;

    case DongleState::kGetVersion:
      if (p.cmd == CMD_GET_DONGLE_VERSION + 1) {
        consumed = true;
        size_t n = p.payloadLen < sizeof(version_) - 1 ? p.payloadLen : sizeof(version_) - 1;
        memcpy(version_, p.payload, n);
        version_[n] = '\0';
        EnterState(DongleState::kFinishAuth, lastNowMs_);
      }
      break;

    case DongleState::kFinishAuth:
      if (p.cmd == CMD_FINISH_AUTH + 1) {
        consumed = true;
        EnterState(DongleState::kGetSensorCount, lastNowMs_);
      }
      break;

    case DongleState::kGetSensorCount:
      if (p.cmd == CMD_GET_SENSOR_COUNT + 1) {
        consumed = true;
        if (p.payloadLen == 1) {
          sensorCount_ = p.payload[0];
          EnterState(DongleState::kGetSensorList, lastNowMs_);
        } else {
          Log("Dongle: GetSensorCount bad payload");
          state_ = DongleState::kFailed;
        }
      }
      break;

    case DongleState::kGetSensorList:
      if (p.cmd == CMD_GET_SENSOR_LIST + 1) {
        consumed = true;
        if (p.payloadLen == 8 && sensorsReceived_ < kMaxSensors) {
          memcpy(sensorMacs_[sensorsReceived_], p.payload, 8);
          sensorMacs_[sensorsReceived_][8] = '\0';
          sensorsReceived_++;
        }
        if (sensorsReceived_ >= sensorCount_) {
          if (onSensorList_) onSensorList_(sensorMacs_, sensorsReceived_);
          state_ = DongleState::kReady;
          if (onReady_) onReady_();
        }
      }
      break;

    case DongleState::kScanEnabling:
      if (p.cmd == CMD_START_STOP_SCAN + 1) {
        consumed = true;
        EnterState(DongleState::kScanWaiting, lastNowMs_);
        // EnterState() set stateDeadlineMs_ to the short command timeout,
        // which kScanWaiting's special-cased Update() branch ignores in
        // favor of this, computed fresh now so the caller's requested
        // window starts from when we're actually listening, not from
        // StartScan()'s call time.
        scanDeadlineMs_ = lastNowMs_ + scanTimeoutMs_;
      }
      break;

    case DongleState::kScanWaiting:
      if (p.cmd == NOTIFY_SENSOR_SCAN) {
        consumed = true;
        if (p.payloadLen == 11) {
          memcpy(scanFoundMac_, p.payload + 1, 8);
          scanFoundMac_[8] = '\0';
          scanFoundType_ = p.payload[9];
          scanFoundVersion_ = p.payload[10];
          scanFound_ = true;
          EnterState(DongleState::kScanGettingR1, lastNowMs_);
        } else {
          Log("Dongle: NOTIFY_SENSOR_SCAN bad payload");
        }
      }
      break;

    case DongleState::kScanGettingR1:
      if (p.cmd == CMD_GET_SENSOR_R1 + 1) {
        consumed = true;
        // Payload (the sensor's R1) isn't otherwise used, matching the
        // reference implementation -- completing the exchange is what
        // matters here.
        EnterState(DongleState::kScanDisabling, lastNowMs_);
      }
      break;

    case DongleState::kScanDisabling:
      if (p.cmd == CMD_START_STOP_SCAN + 1) {
        consumed = true;
        if (scanFound_) {
          EnterState(DongleState::kScanVerifying, lastNowMs_);
        } else {
          state_ = DongleState::kReady;
          if (onScanResult_) onScanResult_(false, "", 0, 0);
        }
      }
      break;

    case DongleState::kScanVerifying:
      if (p.cmd == CMD_VERIFY_SENSOR + 1) {
        consumed = true;
        state_ = DongleState::kReady;
        if (onScanResult_) onScanResult_(true, scanFoundMac_, scanFoundType_, scanFoundVersion_);
      }
      break;

    default:
      break;
  }

  if (!consumed) {
    HandleUnsolicited(p);
  }
}

void Dongle::HandleUnsolicited(const Packet &p) {
  if (p.isAck) return;  // nothing to do with an ack we weren't waiting on

  if (p.cmd == NOTIFY_SENSOR_ALARM) {
    SensorAlarm alarm;
    if (DecodeSensorAlarm(p.payload, p.payloadLen, alarm) && onSensorEvent_) {
      onSensorEvent_(alarm);
    }
  } else if (p.cmd == NOTIFY_SYNC_TIME) {
    // Real wall-clock time isn't available on-device without NTP; the
    // dongle doesn't appear to validate this beyond expecting *a* reply,
    // so a monotonic ms counter is sufficient to keep the handshake happy.
    SendPacket(MakeSyncTimeAck(lastNowMs_));
  } else if (p.cmd == NOTIFY_EVENT_LOG) {
    Log("Dongle: event log notification (not decoded)");
  }
  // else: late/duplicate response or something we don't model -- ignore.
}

}  // namespace wyzesense
