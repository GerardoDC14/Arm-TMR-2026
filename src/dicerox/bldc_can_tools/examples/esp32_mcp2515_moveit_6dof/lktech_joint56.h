#pragma once

#include <Arduino.h>
#include <mcp2515.h>
#include <string.h>

namespace lktech_joint56 {

constexpr uint8_t CMD_MOTOR_OFF = 0x80;
constexpr uint8_t CMD_MOTOR_STOP = 0x81;
constexpr uint8_t CMD_MOTOR_ON = 0x88;
constexpr uint8_t CMD_READ_MULTI_LOOP_ANGLE = 0x92;
constexpr uint8_t CMD_MULTI_LOOP_CONTROL_2 = 0xA4;
constexpr uint16_t REQUEST_ID_BASE = 0x140;
constexpr unsigned long REPLY_TIMEOUT_MS = 250;
constexpr size_t JOINT_COUNT = 2;
// CMD_MOTOR_ON and CMD_READ_MULTI_LOOP_ANGLE replies can be intermittent on
// the first few transactions — allow several retries per joint at init time.
constexpr size_t INIT_MAX_RETRIES = 5;
constexpr unsigned long INIT_RETRY_DELAY_MS = 100;

struct JointConfig {
  const char *name;
  uint16_t canId;
  double gearRatio;
  double defaultOutputSpeedDps;
};

static constexpr JointConfig kJointConfigs[JOINT_COUNT] = {
    {"joint5", 0x14E, 10.0, 60.0},
    {"joint6", 0x14F, 10.0, 60.0},
};

struct JointRuntimeState {
  bool enabled;
  bool haveZero;
  bool haveTarget;
  double zeroMotorDeg;
  double lastMotorDeg;
  double lastOutputDeg;
  double outputSpeedDps;
  unsigned long lastCommandMs;
};

struct State {
  JointRuntimeState joints[JOINT_COUNT];
};

inline int64_t readSigned56LE(const uint8_t *data) {
  uint64_t raw = 0;
  for (int i = 0; i < 7; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (8 * i);
  }
  if (data[6] & 0x80) {
    raw |= 0xFF00000000000000ULL;
  }
  return static_cast<int64_t>(raw);
}

inline double centiDegreesToDegrees(int64_t centiDegrees) {
  return static_cast<double>(centiDegrees) / 100.0;
}

inline int32_t degreesToCentiDegrees(double degrees) {
  const double scaled = degrees * 100.0;
  return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}

inline void reset(State &state) {
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    state.joints[i].enabled = false;
    state.joints[i].haveZero = false;
    state.joints[i].haveTarget = false;
    state.joints[i].zeroMotorDeg = 0.0;
    state.joints[i].lastMotorDeg = 0.0;
    state.joints[i].lastOutputDeg = 0.0;
    state.joints[i].outputSpeedDps = kJointConfigs[i].defaultOutputSpeedDps;
    state.joints[i].lastCommandMs = 0;
  }
}

inline uint16_t outputSpeedToMotorDps(size_t index, double outputDps) {
  const double motorDps = outputDps * kJointConfigs[index].gearRatio;
  if (motorDps <= 0.0) {
    return 0;
  }
  if (motorDps >= 65535.0) {
    return 65535;
  }
  return static_cast<uint16_t>(motorDps + 0.5);
}

inline double outputToMotorDegrees(const State &state, size_t index, double outputDeg) {
  return state.joints[index].zeroMotorDeg + outputDeg * kJointConfigs[index].gearRatio;
}

inline bool sendFrame(MCP2515 &controller, uint16_t id, const uint8_t *data, uint8_t dlc = 8) {
  can_frame frame = {};
  frame.can_id = id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);
  return controller.sendMessage(&frame) == MCP2515::ERROR_OK;
}

inline void drainReceiveQueue(MCP2515 &controller) {
  can_frame frame = {};
  while (controller.readMessage(&frame) == MCP2515::ERROR_OK) {
  }
}

inline bool waitForReply(MCP2515 &controller, uint16_t expectedId, uint8_t expectedCommand, can_frame *replyFrame) {
  const unsigned long startMs = millis();
  can_frame frame = {};
  while (millis() - startMs < REPLY_TIMEOUT_MS) {
    while (controller.readMessage(&frame) == MCP2515::ERROR_OK) {
      if ((frame.can_id & CAN_SFF_MASK) == expectedId &&
          frame.can_dlc >= 1 &&
          frame.data[0] == expectedCommand) {
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

inline bool sendSimpleCommandAndWait(MCP2515 &controller, size_t index, uint8_t command, can_frame *replyFrame = nullptr) {
  const uint8_t payload[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  drainReceiveQueue(controller);
  return sendFrame(controller, kJointConfigs[index].canId, payload, sizeof(payload)) &&
         waitForReply(controller, kJointConfigs[index].canId, command, replyFrame);
}

inline bool readCurrentMotorAngle(MCP2515 &controller, State &state, size_t index, double *motorAngleDeg) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(controller, index, CMD_READ_MULTI_LOOP_ANGLE, &reply)) {
    return false;
  }

  const int64_t angleCdeg = readSigned56LE(&reply.data[1]);
  state.joints[index].lastMotorDeg = centiDegreesToDegrees(angleCdeg);
  if (motorAngleDeg != nullptr) {
    *motorAngleDeg = state.joints[index].lastMotorDeg;
  }
  return true;
}

inline bool captureZero(MCP2515 &controller, State &state, size_t index) {
  double motorAngleDeg = 0.0;
  if (!readCurrentMotorAngle(controller, state, index, &motorAngleDeg)) {
    return false;
  }

  state.joints[index].zeroMotorDeg = motorAngleDeg;
  state.joints[index].haveZero = true;
  state.joints[index].haveTarget = true;
  state.joints[index].lastOutputDeg = 0.0;
  return true;
}

inline bool motorOnAndCaptureZero(MCP2515 &controller, State &state, size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(controller, index, CMD_MOTOR_ON, &reply)) {
    return false;
  }

  state.joints[index].enabled = true;
  delay(20);
  return captureZero(controller, state, index);
}

inline bool initialize(MCP2515 &controller, State &state) {
  reset(state);
  bool allOk = true;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    bool jointOk = false;
    for (size_t attempt = 0; attempt < INIT_MAX_RETRIES && !jointOk; ++attempt) {
      if (attempt > 0) {
        delay(INIT_RETRY_DELAY_MS);
      }
      jointOk = motorOnAndCaptureZero(controller, state, i);
    }
    if (!jointOk) {
      allOk = false;
    }
    delay(10);
  }
  return allOk;
}

inline bool sendMotorDegrees(MCP2515 &controller, State &state, size_t index, double targetMotorDeg) {
  const uint16_t motorSpeedDps = outputSpeedToMotorDps(index, state.joints[index].outputSpeedDps);
  const int32_t targetCdeg = degreesToCentiDegrees(targetMotorDeg);
  const uint8_t payload[8] = {
      CMD_MULTI_LOOP_CONTROL_2,
      0x00,
      static_cast<uint8_t>(motorSpeedDps & 0xFF),
      static_cast<uint8_t>((motorSpeedDps >> 8) & 0xFF),
      static_cast<uint8_t>(targetCdeg & 0xFF),
      static_cast<uint8_t>((targetCdeg >> 8) & 0xFF),
      static_cast<uint8_t>((targetCdeg >> 16) & 0xFF),
      static_cast<uint8_t>((targetCdeg >> 24) & 0xFF),
  };

  if (!sendFrame(controller, kJointConfigs[index].canId, payload, sizeof(payload))) {
    return false;
  }

  state.joints[index].lastMotorDeg = targetMotorDeg;
  state.joints[index].lastCommandMs = millis();
  return true;
}

inline bool sendOutputDegrees(MCP2515 &controller, State &state, size_t index, double outputDeg) {
  if (!state.joints[index].haveZero) {
    return false;
  }

  const double targetMotorDeg = outputToMotorDegrees(state, index, outputDeg);
  if (!sendMotorDegrees(controller, state, index, targetMotorDeg)) {
    return false;
  }

  state.joints[index].lastOutputDeg = outputDeg;
  state.joints[index].haveTarget = true;
  return true;
}

inline bool stop(MCP2515 &controller, State &state, size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(controller, index, CMD_MOTOR_STOP, &reply)) {
    return false;
  }

  state.joints[index].haveTarget = false;
  return true;
}

inline bool stopAll(MCP2515 &controller, State &state) {
  bool allOk = true;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    if (!stop(controller, state, i)) {
      allOk = false;
    }
    delay(5);
  }
  return allOk;
}

}  // namespace lktech_joint56
