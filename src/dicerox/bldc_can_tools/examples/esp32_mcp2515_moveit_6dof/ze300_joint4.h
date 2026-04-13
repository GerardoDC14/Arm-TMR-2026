#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <string.h>

namespace ze300_joint4 {

constexpr uint8_t DEVICE_ADDRESS = 13;
constexpr uint16_t REQUEST_TAG_MASK = 0x100;
constexpr uint16_t COUNTS_PER_REV = 16384;
constexpr uint8_t CMD_READ_POS = 0xA3;
constexpr uint8_t CMD_SET_SPEED = 0xB2;
constexpr uint8_t CMD_ABS_POS = 0xC2;
constexpr uint8_t CMD_DISABLE = 0xCF;
constexpr unsigned long REPLY_TIMEOUT_MS = 250;
constexpr double DEFAULT_SPEED_DPS = 90.0;
// ZE300 only ACKs B2 (set-speed) after a short firmware warmup at first power-on;
// allow several retries so initialize() succeeds without manual intervention.
constexpr size_t INIT_MAX_RETRIES = 5;
constexpr unsigned long INIT_RETRY_DELAY_MS = 100;

struct State {
  bool speedConfigured;
  uint32_t speedCentiRpm;
  int32_t lastCounts;
  double lastAbsoluteDeg;
};

inline void reset(State &state) {
  state.speedConfigured = false;
  state.speedCentiRpm = 0;
  state.lastCounts = 0;
  state.lastAbsoluteDeg = 0.0;
}

inline uint16_t requestId() {
  return static_cast<uint16_t>(REQUEST_TAG_MASK | DEVICE_ADDRESS);
}

inline uint16_t replyId() {
  return DEVICE_ADDRESS;
}

inline int32_t readI32LE(const uint8_t *data, uint8_t offset) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(data[offset]) |
      (static_cast<uint32_t>(data[offset + 1]) << 8) |
      (static_cast<uint32_t>(data[offset + 2]) << 16) |
      (static_cast<uint32_t>(data[offset + 3]) << 24));
}

inline int32_t degreesToCounts(double degrees) {
  const double scaled = degrees * static_cast<double>(COUNTS_PER_REV) / 360.0;
  return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

inline uint32_t dpsToCentiRpm(double dps) {
  const double scaled = dps / 6.0 * 100.0;
  if (scaled <= 0.0) {
    return 0;
  }
  return static_cast<uint32_t>(scaled + 0.5);
}

inline bool sendFrame(MCP2515 &controller, uint16_t id, const uint8_t *data, uint8_t dlc) {
  can_frame frame = {};
  frame.can_id = id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);
  return controller.sendMessage(&frame) == MCP2515::ERROR_OK;
}

inline bool waitForReply(MCP2515 &controller, uint8_t command, can_frame *replyFrame) {
  const unsigned long startMs = millis();
  can_frame frame = {};
  while (millis() - startMs < REPLY_TIMEOUT_MS) {
    if (controller.readMessage(&frame) == MCP2515::ERROR_OK) {
      if ((frame.can_id & CAN_SFF_MASK) == replyId() &&
          frame.can_dlc >= 1 &&
          frame.data[0] == command) {
        if (replyFrame != nullptr) {
          *replyFrame = frame;
        }
        return true;
      }
    }
    delay(2);
  }
  return false;
}

inline bool setSpeed(MCP2515 &controller, State &state, double outputDps) {
  const uint32_t centiRpm = dpsToCentiRpm(outputDps);
  const uint8_t payload[5] = {
      CMD_SET_SPEED,
      static_cast<uint8_t>(centiRpm & 0xFF),
      static_cast<uint8_t>((centiRpm >> 8) & 0xFF),
      static_cast<uint8_t>((centiRpm >> 16) & 0xFF),
      static_cast<uint8_t>((centiRpm >> 24) & 0xFF),
  };

  can_frame reply = {};
  if (!sendFrame(controller, requestId(), payload, sizeof(payload)) ||
      !waitForReply(controller, CMD_SET_SPEED, &reply) ||
      reply.can_dlc != 5) {
    return false;
  }

  state.speedConfigured = true;
  state.speedCentiRpm = static_cast<uint32_t>(readI32LE(reply.data, 1));
  return true;
}

inline bool initialize(MCP2515 &controller, State &state) {
  reset(state);
  // ZE300 B2 (set-speed) reply is intermittent on the first few commands after
  // power-on — the motor firmware takes a moment before it ACKs.  Retry several
  // times with a short gap before declaring failure.
  for (size_t attempt = 0; attempt < INIT_MAX_RETRIES; ++attempt) {
    if (attempt > 0) {
      delay(INIT_RETRY_DELAY_MS);
    }
    if (setSpeed(controller, state, DEFAULT_SPEED_DPS)) {
      return true;
    }
  }
  return false;
}

inline bool sendAbsoluteDegrees(MCP2515 &controller, State &state, double absoluteDeg) {
  const int32_t counts = degreesToCounts(absoluteDeg);
  const uint8_t payload[5] = {
      CMD_ABS_POS,
      static_cast<uint8_t>(counts & 0xFF),
      static_cast<uint8_t>((counts >> 8) & 0xFF),
      static_cast<uint8_t>((counts >> 16) & 0xFF),
      static_cast<uint8_t>((counts >> 24) & 0xFF),
  };

  if (!sendFrame(controller, requestId(), payload, sizeof(payload))) {
    return false;
  }

  state.lastCounts = counts;
  state.lastAbsoluteDeg = absoluteDeg;
  return true;
}

inline bool disable(MCP2515 &controller) {
  const uint8_t payload[1] = {CMD_DISABLE};
  return sendFrame(controller, requestId(), payload, sizeof(payload));
}

}  // namespace ze300_joint4
