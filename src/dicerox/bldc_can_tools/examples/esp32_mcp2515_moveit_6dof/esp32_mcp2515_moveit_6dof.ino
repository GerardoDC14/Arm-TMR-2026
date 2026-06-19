#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif

#ifndef CAN_RTR_FLAG
#define CAN_RTR_FLAG 0x40000000UL
#endif

#include "odrive_can_support.h"
#include "odrive_can_state.h"
#include "ze300_joint4.h"
#include "lktech_joint56.h"

namespace {

using namespace odrive_can_sniffer;

constexpr float JOINT4_MIN_DEG = -90.0f;
constexpr float JOINT4_MAX_DEG = 90.0f;
constexpr float JOINT4_FIRST_COMMAND_MAX_DEG = 30.0f;
constexpr float JOINT5_MIN_DEG = -90.0f;
constexpr float JOINT5_MAX_DEG = 90.0f;
constexpr float JOINT6_MIN_DEG = -90.0f;
constexpr float JOINT6_MAX_DEG = 90.0f;
constexpr uint32_t JS6_SNAPSHOT_TIMEOUT_MS = 450;
constexpr size_t COORDINATED_JOINT_COUNT = 6;
constexpr uint32_t RAMP_DEBUG_LOG_INTERVAL_MS = 250;
constexpr uint8_t CAN_SEND_FAIL_THRESHOLD = 5;
constexpr uint32_t CAN_FAULT_WINDOW_MS = 1000;
constexpr uint32_t MOTOR_HEARTBEAT_TIMEOUT_MS = 2500;
constexpr uint8_t MOTOR_COMMAND_FAIL_THRESHOLD = 3;
constexpr bool CAN_FAULT_DISARM_IF_REACHABLE = true;

enum class FirmwareState : uint8_t {
  MotorsUninitialized,
  MotorsInitializing,
  MotorsReady,
  FaultCan,
};

const float DEFAULT_COORDINATED_MAX_VELOCITY_DPS[COORDINATED_JOINT_COUNT] = {
    10.0f, 8.0f, 8.0f, 20.0f, 20.0f, 20.0f};
const float DEFAULT_COORDINATED_MAX_ACCEL_DPS2[COORDINATED_JOINT_COUNT] = {
    30.0f, 24.0f, 24.0f, 60.0f, 60.0f, 60.0f};

struct AuxJointTargetState {
  bool joint4HaveTarget;
  bool joint5HaveTarget;
  bool joint6HaveTarget;
  float joint4Deg;
  float joint5Deg;
  float joint6Deg;
  unsigned long joint4LastTxMs;
  unsigned long joint5LastTxMs;
  unsigned long joint6LastTxMs;
};

struct CoordinatedSextetTargetState {
  bool haveTarget;
  bool rampInitialized;
  uint32_t epoch;
  float joint1Deg;
  float joint2Deg;
  float joint3Deg;
  float joint4Deg;
  float joint5Deg;
  float joint6Deg;
  float commandedDeg[COORDINATED_JOINT_COUNT];
  float velocityDps[COORDINATED_JOINT_COUNT];
  unsigned long lastStreamMs;
  unsigned long lastRampMs;
  unsigned long lastDebugLogMs;
};

struct NodeMotionCacheSnapshot {
  bool haveZeroReference;
  bool haveActiveTarget;
  float zeroReferenceTurns;
  float lastRelativeCommandTurns;
  float lastRelativeCommandDegrees;
  float lastAbsoluteCommandTurns;
  unsigned long lastTargetStreamMs;
};

struct AuxTargetSnapshot {
  bool joint4HaveTarget;
  bool joint5HaveTarget;
  bool joint6HaveTarget;
  float joint4Deg;
  float joint5Deg;
  float joint6Deg;
  unsigned long joint4LastTxMs;
  unsigned long joint5LastTxMs;
  unsigned long joint6LastTxMs;
};

struct SextetSendResult {
  bool ok1;
  bool ok2;
  bool ok3;
  bool ok4;
  bool ok5;
  bool ok6;
};

MCP2515 canController(PIN_CAN_CS);
struct can_frame rxFrame;

CanMode g_mode = CanMode::Normal;
Bitrate g_bitrate = Bitrate::Kbps500;
Oscillator g_oscillator = Oscillator::MHz8;
bool g_auxFiltersEnabled = false;  // when false, LKTech/ZE300 IDs are excluded from RX filters
bool g_canControllerReady = false;
bool g_showAllStandardFrames = false;
bool g_printEncoderTelemetry = false;
bool g_streamLastTarget = true;
bool g_backgroundBringupEnabled = false;
FirmwareState g_firmwareState = FirmwareState::MotorsUninitialized;
bool g_motorsInitialized = false;
uint8_t g_canSendFailCount = 0;
uint8_t g_motorCommandFailCount = 0;
uint32_t g_canSendFailCountTotal = 0;
uint32_t g_canIncompleteBurstCount = 0;
uint32_t g_jointCommandFailCount[COORDINATED_JOINT_COUNT] = {};
uint32_t g_canLastFailedId = 0;
int g_canLastFailedCode = static_cast<int>(MCP2515::ERROR_OK);
bool g_canFaultStrictDebug = false;
unsigned long g_canFaultWindowStartMs = 0;
unsigned long g_motorFaultWindowStartMs = 0;
char g_lastFirmwareFault[96] = "none";
uint8_t g_activeNodeId = DEFAULT_ODRIVE_NODE_ID;
size_t g_nextBackgroundBringupIndex = 0;
unsigned long g_lastBackgroundBringupMs = 0;
unsigned long g_lastSerialActivityMs = 0;
NodeRuntimeState g_nodeStates[SUPPORTED_NODE_COUNT];
char g_serialLine[SERIAL_LINE_BUFFER_SIZE] = {};
size_t g_serialLineLen = 0;
ze300_joint4::State g_joint4State;
lktech_joint56::State g_joint56State;
AuxJointTargetState g_auxTargets = {};
CoordinatedSextetTargetState g_coordinatedSextetTarget = {};
uint32_t g_nextCoordinatedSextetEpoch = 1;
bool g_nextCoordinatedSextetAuxFirst = true;
float g_coordinatedMaxVelocityDps[COORDINATED_JOINT_COUNT] = {
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[0],
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[1],
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[2],
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[3],
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[4],
    DEFAULT_COORDINATED_MAX_VELOCITY_DPS[5],
};
float g_coordinatedMaxAccelDps2[COORDINATED_JOINT_COUNT] = {
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[0],
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[1],
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[2],
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[3],
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[4],
    DEFAULT_COORDINATED_MAX_ACCEL_DPS2[5],
};

void processIncomingFrames();
void serviceTargetStreaming();
void serviceBackgroundBringup();
void serviceSafetyFailsafes();
void printHexByte(uint8_t value);
void printCanControllerDiagnostics(const char *prefix, uint8_t eflg);
bool configureReceiveFilters(bool includeAux);
void restoreMotionCache(NodeRuntimeState &state,
                        bool haveZeroReference,
                        bool haveActiveTarget,
                        float zeroReferenceTurns,
                        float lastRelativeCommandTurns,
                        float lastRelativeCommandDegrees,
                        float lastAbsoluteCommandTurns);
bool sendRemoteRequestToNode(uint8_t nodeId, uint8_t cmd, uint8_t dlc);
bool sendRelativePositionDegreesToNode(uint8_t nodeId, float relativeDegrees, bool printSummary, bool logCanFrame);
bool sendJoint4RelativeDegrees(float relativeDegrees, bool printSummary);
bool sendJoint5AbsoluteDegrees(float absoluteDegrees, bool printSummary);
bool sendJoint6AbsoluteDegrees(float absoluteDegrees, bool printSummary);
bool handleMoveItBridgeCommand(const char *line);
bool handleMoveItIdleCommand(const char *line);
bool handleJointSnapshotRequest(const char *line);
bool handleAuxJointTestCommand(const char *line);
bool handleCoordinatedRampConfigCommand(const char *line);
bool ensureActiveNodeReadyForMotion(const char *reason);
void runAutomaticBringupAllNodes();
void clearActiveTargetForNode(uint8_t nodeId, const char *reason);
void recoverFromTxFailure(const can_frame &frame, MCP2515::ERROR sendResult);
bool nodeHasBootConfiguration(const NodeRuntimeState &state);
bool allNodesHaveBootConfiguration();
void printBootConfigurationSummary(const char *prefix);
bool nodeHasActiveFault(const NodeRuntimeState &state);
void clearFailsafeLatch(NodeRuntimeState &state);
void clearHeartbeatWarning(NodeRuntimeState &state, bool logRecovery);
void noteHeartbeatWarning(NodeRuntimeState &state, unsigned long now);
bool sendAxisStateRequestToNode(uint8_t nodeId, uint32_t state, const char *message, bool logFrame);
void engageNodeFailsafe(uint8_t nodeId, const char *reason, bool requestIdleIfReachable);
NodeMotionCacheSnapshot captureMotionCache(const NodeRuntimeState &state);
void restoreMotionCache(NodeRuntimeState &state, const NodeMotionCacheSnapshot &snapshot);
AuxTargetSnapshot captureAuxTargetSnapshot();
void restoreAuxTargetSnapshot(const AuxTargetSnapshot &snapshot);
void clearCoordinatedSextetTarget();
void storeCoordinatedSextetTarget(float joint1Deg,
                                  float joint2Deg,
                                  float joint3Deg,
                                  float joint4Deg,
                                  float joint5Deg,
                                  float joint6Deg,
                                  unsigned long lastStreamMs);
SextetSendResult sendCoordinatedSextetBurst(float joint1Deg,
                                            float joint2Deg,
                                            float joint3Deg,
                                            float joint4Deg,
                                            float joint5Deg,
                                            float joint6Deg,
                                            bool auxGroupFirst);
bool sextetSendSucceeded(const SextetSendResult &result);
void printSextetSendResult(const char *prefix, const SextetSendResult &result);
void printCoordinatedRampLimits();
bool readJointSnapshot(float *jointDeg, char *errorText, size_t errorTextSize);
void printJointSnapshot();
void armAuxJointHoldTargetsAfterBoot();
bool initializeAllMotorsExplicitly();
void printFirmwareStatus6();
void disarmAllMotors(const char *reason);
void enterCanFault(const char *reason);
void noteCanSendResult(bool success, const char *context);
void noteMotorCommandResult(bool success, const char *context);
void serviceCanHealth();
bool handleLifecycleCommand(const char *line);
const char *mcp2515ErrorName(int code);
void printEflgFlags(uint8_t eflg);
uint8_t activeCanSendFailThreshold();
uint8_t activeMotorCommandFailThreshold();

const char *mcp2515ErrorName(int code) {
  switch (code) {
    case MCP2515::ERROR_OK: return "ERROR_OK";
    case MCP2515::ERROR_FAIL: return "ERROR_FAIL";
    case MCP2515::ERROR_ALLTXBUSY: return "ERROR_ALLTXBUSY";
    case MCP2515::ERROR_FAILINIT: return "ERROR_FAILINIT";
    case MCP2515::ERROR_FAILTX: return "ERROR_FAILTX";
    case MCP2515::ERROR_NOMSG: return "ERROR_NOMSG";
    case -1: return "DRIVER_BOOL_FAILURE";
    default: return "ERROR_UNKNOWN";
  }
}

void printEflgFlags(uint8_t eflg) {
  bool printed = false;
  auto emit = [&](uint8_t mask, const char *name) {
    if ((eflg & mask) == 0) return;
    if (printed) Serial.print(',');
    Serial.print(name);
    printed = true;
  };
  emit(MCP2515::EFLG_RX1OVR, "RX1OVR");
  emit(MCP2515::EFLG_RX0OVR, "RX0OVR");
  emit(MCP2515::EFLG_TXBO, "TXBO");
  emit(MCP2515::EFLG_TXEP, "TXEP");
  emit(MCP2515::EFLG_RXEP, "RXEP");
  emit(MCP2515::EFLG_TXWAR, "TXWAR");
  emit(MCP2515::EFLG_RXWAR, "RXWAR");
  emit(MCP2515::EFLG_EWARN, "EWARN");
  if (!printed) Serial.print("NONE");
}

uint8_t activeCanSendFailThreshold() {
  return g_canFaultStrictDebug ? 2 : CAN_SEND_FAIL_THRESHOLD;
}

uint8_t activeMotorCommandFailThreshold() {
  return g_canFaultStrictDebug ? 2 : MOTOR_COMMAND_FAIL_THRESHOLD;
}

const char *firmwareStateName(FirmwareState state) {
  switch (state) {
    case FirmwareState::MotorsUninitialized: return "MOTORS_UNINITIALIZED";
    case FirmwareState::MotorsInitializing: return "MOTORS_INITIALIZING";
    case FirmwareState::MotorsReady: return "MOTORS_READY";
    case FirmwareState::FaultCan: return "FAULT_CAN";
  }
  return "UNKNOWN";
}

bool isTrackedNodeId(uint8_t nodeId) {
  return odrive_can_sniffer::isSupportedNodeId(nodeId);
}

NodeRuntimeState *lookupNodeState(uint8_t nodeId) {
  return odrive_can_sniffer::findNodeState(g_nodeStates, SUPPORTED_NODE_COUNT, nodeId);
}

NodeRuntimeState &selectedNodeState() {
  return odrive_can_sniffer::activeNodeState(g_nodeStates, SUPPORTED_NODE_COUNT, g_activeNodeId);
}

void initializeRuntimeStates() {
  odrive_can_sniffer::initializeNodeStates(g_nodeStates, SUPPORTED_NODE_COUNT);
}

void selectActiveNode(uint8_t nodeId) {
  if (!isTrackedNodeId(nodeId)) {
    Serial.print("Unsupported node ID 0x");
    printHexByte(nodeId);
    Serial.println();
    return;
  }

  g_activeNodeId = nodeId;

  Serial.print("Active node set to 0x");
  printHexByte(g_activeNodeId);
  Serial.print(". Cached zero=");
  Serial.print(selectedNodeState().haveZeroReference ? "yes" : "no");
  Serial.println(".");
}

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printCanControllerDiagnostics(const char *prefix, uint8_t eflg) {
  if (prefix != nullptr && prefix[0] != '\0') {
    Serial.print(prefix);
    Serial.print(' ');
  }

  const uint8_t tec = canController.errorCountTX();
  const uint8_t rec = canController.errorCountRX();
  const uint8_t intf = canController.getInterrupts();

  Serial.print("MCP2515 diag: EFLG=0x");
  Serial.print(eflg, HEX);
  Serial.print(" TEC=");
  Serial.print(tec);
  Serial.print(" REC=");
  Serial.print(rec);
  Serial.print(" CANINTF=0x");
  Serial.println(intf, HEX);
}

// includeAux = true  → bank1 slots 3-5 accept ZE300 and LKTech reply IDs (full operation)
// includeAux = false → bank1 slots 3-5 duplicate the encoder filter so LKTech/ZE300
//                      broadcasts cannot consume RXB1 slots needed for ODrive heartbeats
//                      and encoder estimates during bringup.
bool configureReceiveFilters(bool includeAux) {
  const uint32_t commandOnlyMask = 0x01FU;

  struct FilterConfig {
    MCP2515::RXF filter;
    uint32_t id;
  };

  const FilterConfig bank0[] = {
      {MCP2515::RXF0, CMD_HEARTBEAT},
      {MCP2515::RXF1, CMD_GET_ERROR},
  };
  const FilterConfig bank1Full[] = {
      {MCP2515::RXF2, CMD_GET_ENCODER_ESTIMATES},
      {MCP2515::RXF3, ze300_joint4::replyId()},
      {MCP2515::RXF4, lktech_joint56::kJointConfigs[0].canId},
      {MCP2515::RXF5, lktech_joint56::kJointConfigs[1].canId},
  };
  const FilterConfig bank1OdriveOnly[] = {
      {MCP2515::RXF2, CMD_GET_ENCODER_ESTIMATES},
      {MCP2515::RXF3, CMD_GET_ENCODER_ESTIMATES},
      {MCP2515::RXF4, CMD_GET_ENCODER_ESTIMATES},
      {MCP2515::RXF5, CMD_GET_ENCODER_ESTIMATES},
  };

  auto setMask = [&](MCP2515::MASK mask, uint32_t value) -> bool {
    const MCP2515::ERROR result = canController.setFilterMask(mask, false, value);
    if (result != MCP2515::ERROR_OK) {
      Serial.print("ERROR: failed to set MCP2515 RX mask code=");
      Serial.println(static_cast<int>(result));
      return false;
    }
    return true;
  };

  auto setFilters = [&](const FilterConfig *configs, size_t count) -> bool {
    for (size_t i = 0; i < count; ++i) {
      const MCP2515::ERROR result = canController.setFilter(configs[i].filter, false, configs[i].id);
      if (result != MCP2515::ERROR_OK) {
        Serial.print("ERROR: failed to set MCP2515 RX filter code=");
        Serial.print(static_cast<int>(result));
        Serial.print(" filter=");
        Serial.print(static_cast<int>(configs[i].filter));
        Serial.print(" id=0x");
        Serial.println(configs[i].id, HEX);
        return false;
      }
    }
    return true;
  };

  const FilterConfig *bank1 = includeAux ? bank1Full : bank1OdriveOnly;
  const size_t bank1Count = includeAux ? sizeof(bank1Full) / sizeof(bank1Full[0])
                                       : sizeof(bank1OdriveOnly) / sizeof(bank1OdriveOnly[0]);

  if (!setMask(MCP2515::MASK0, commandOnlyMask) ||
      !setFilters(bank0, sizeof(bank0) / sizeof(bank0[0])) ||
      !setMask(MCP2515::MASK1, commandOnlyMask) ||
      !setFilters(bank1, bank1Count)) {
    return false;
  }

  if (includeAux) {
    Serial.println("MCP2515 RX filters configured for ODrive heartbeat/error/encoder + ZE300/LK replies");
  } else {
    Serial.println("MCP2515 RX filters configured for ODrive heartbeat/error/encoder only (aux excluded)");
  }
  return true;
}

void restoreMotionCache(NodeRuntimeState &state,
                        bool haveZeroReference,
                        bool haveActiveTarget,
                        float zeroReferenceTurns,
                        float lastRelativeCommandTurns,
                        float lastRelativeCommandDegrees,
                        float lastAbsoluteCommandTurns) {
  state.haveZeroReference = haveZeroReference;
  state.haveActiveTarget = haveActiveTarget;
  state.zeroReferenceTurns = zeroReferenceTurns;
  state.lastRelativeCommandTurns = lastRelativeCommandTurns;
  state.lastRelativeCommandDegrees = lastRelativeCommandDegrees;
  state.lastAbsoluteCommandTurns = lastAbsoluteCommandTurns;
}

NodeMotionCacheSnapshot captureMotionCache(const NodeRuntimeState &state) {
  NodeMotionCacheSnapshot snapshot = {};
  snapshot.haveZeroReference = state.haveZeroReference;
  snapshot.haveActiveTarget = state.haveActiveTarget;
  snapshot.zeroReferenceTurns = state.zeroReferenceTurns;
  snapshot.lastRelativeCommandTurns = state.lastRelativeCommandTurns;
  snapshot.lastRelativeCommandDegrees = state.lastRelativeCommandDegrees;
  snapshot.lastAbsoluteCommandTurns = state.lastAbsoluteCommandTurns;
  snapshot.lastTargetStreamMs = state.lastTargetStreamMs;
  return snapshot;
}

void restoreMotionCache(NodeRuntimeState &state, const NodeMotionCacheSnapshot &snapshot) {
  restoreMotionCache(state,
                     snapshot.haveZeroReference,
                     snapshot.haveActiveTarget,
                     snapshot.zeroReferenceTurns,
                     snapshot.lastRelativeCommandTurns,
                     snapshot.lastRelativeCommandDegrees,
                     snapshot.lastAbsoluteCommandTurns);
  state.lastTargetStreamMs = snapshot.lastTargetStreamMs;
}

AuxTargetSnapshot captureAuxTargetSnapshot() {
  AuxTargetSnapshot snapshot = {};
  snapshot.joint4HaveTarget = g_auxTargets.joint4HaveTarget;
  snapshot.joint5HaveTarget = g_auxTargets.joint5HaveTarget;
  snapshot.joint6HaveTarget = g_auxTargets.joint6HaveTarget;
  snapshot.joint4Deg = g_auxTargets.joint4Deg;
  snapshot.joint5Deg = g_auxTargets.joint5Deg;
  snapshot.joint6Deg = g_auxTargets.joint6Deg;
  snapshot.joint4LastTxMs = g_auxTargets.joint4LastTxMs;
  snapshot.joint5LastTxMs = g_auxTargets.joint5LastTxMs;
  snapshot.joint6LastTxMs = g_auxTargets.joint6LastTxMs;
  return snapshot;
}

void restoreAuxTargetSnapshot(const AuxTargetSnapshot &snapshot) {
  g_auxTargets.joint4HaveTarget = snapshot.joint4HaveTarget;
  g_auxTargets.joint5HaveTarget = snapshot.joint5HaveTarget;
  g_auxTargets.joint6HaveTarget = snapshot.joint6HaveTarget;
  g_auxTargets.joint4Deg = snapshot.joint4Deg;
  g_auxTargets.joint5Deg = snapshot.joint5Deg;
  g_auxTargets.joint6Deg = snapshot.joint6Deg;
  g_auxTargets.joint4LastTxMs = snapshot.joint4LastTxMs;
  g_auxTargets.joint5LastTxMs = snapshot.joint5LastTxMs;
  g_auxTargets.joint6LastTxMs = snapshot.joint6LastTxMs;
}

void clearCoordinatedSextetTarget() {
  g_coordinatedSextetTarget.haveTarget = false;
  g_coordinatedSextetTarget.rampInitialized = false;
  g_coordinatedSextetTarget.epoch = 0;
  g_coordinatedSextetTarget.lastStreamMs = 0;
  g_coordinatedSextetTarget.lastRampMs = 0;
  g_coordinatedSextetTarget.lastDebugLogMs = 0;
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    g_coordinatedSextetTarget.velocityDps[i] = 0.0f;
  }
}

void storeCoordinatedSextetTarget(float joint1Deg,
                                  float joint2Deg,
                                  float joint3Deg,
                                  float joint4Deg,
                                  float joint5Deg,
                                  float joint6Deg,
                                  unsigned long lastStreamMs) {
  const bool alreadyStreaming = g_coordinatedSextetTarget.haveTarget;
  g_coordinatedSextetTarget.haveTarget = true;
  g_coordinatedSextetTarget.epoch = g_nextCoordinatedSextetEpoch++;
  g_coordinatedSextetTarget.joint1Deg = joint1Deg;
  g_coordinatedSextetTarget.joint2Deg = joint2Deg;
  g_coordinatedSextetTarget.joint3Deg = joint3Deg;
  g_coordinatedSextetTarget.joint4Deg = joint4Deg;
  g_coordinatedSextetTarget.joint5Deg = joint5Deg;
  g_coordinatedSextetTarget.joint6Deg = joint6Deg;
  if (!alreadyStreaming) {
    g_coordinatedSextetTarget.lastStreamMs = 0;
  }
  if (!g_coordinatedSextetTarget.rampInitialized) {
    NodeRuntimeState *state10 = lookupNodeState(NODE_ID_10);
    NodeRuntimeState *state11 = lookupNodeState(NODE_ID_11);
    NodeRuntimeState *state12 = lookupNodeState(NODE_ID_12);
    g_coordinatedSextetTarget.commandedDeg[0] =
        state10 != nullptr && state10->haveActiveTarget ? state10->lastRelativeCommandDegrees : 0.0f;
    g_coordinatedSextetTarget.commandedDeg[1] =
        state11 != nullptr && state11->haveActiveTarget ? state11->lastRelativeCommandDegrees : 0.0f;
    g_coordinatedSextetTarget.commandedDeg[2] =
        state12 != nullptr && state12->haveActiveTarget ? state12->lastRelativeCommandDegrees : 0.0f;
    g_coordinatedSextetTarget.commandedDeg[3] =
        g_auxTargets.joint4HaveTarget ? g_auxTargets.joint4Deg : 0.0f;
    g_coordinatedSextetTarget.commandedDeg[4] =
        g_auxTargets.joint5HaveTarget ? g_auxTargets.joint5Deg : 0.0f;
    g_coordinatedSextetTarget.commandedDeg[5] =
        g_auxTargets.joint6HaveTarget ? g_auxTargets.joint6Deg : 0.0f;
    g_coordinatedSextetTarget.rampInitialized = true;
    g_coordinatedSextetTarget.lastRampMs = lastStreamMs;
    for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
      g_coordinatedSextetTarget.velocityDps[i] = 0.0f;
    }
  }
}

float stepRampLimited(float currentDeg,
                      float targetDeg,
                      float *velocityDps,
                      float maxVelocityDps,
                      float maxAccelDps2,
                      float dtSec) {
  const float errorDeg = targetDeg - currentDeg;
  if (fabs(errorDeg) <= 0.0005f && fabs(*velocityDps) <= 0.0005f) {
    *velocityDps = 0.0f;
    return targetDeg;
  }

  const float direction = errorDeg >= 0.0f ? 1.0f : -1.0f;
  const float stoppingSpeed = sqrtf(fmaxf(0.0f, 2.0f * maxAccelDps2 * fabs(errorDeg)));
  const float desiredVelocity = direction * fminf(maxVelocityDps, stoppingSpeed);
  const float maxVelocityStep = maxAccelDps2 * dtSec;

  if (desiredVelocity > *velocityDps) {
    *velocityDps = fminf(desiredVelocity, *velocityDps + maxVelocityStep);
  } else {
    *velocityDps = fmaxf(desiredVelocity, *velocityDps - maxVelocityStep);
  }

  float nextDeg = currentDeg + (*velocityDps) * dtSec;
  const float remainingAfterStep = targetDeg - nextDeg;
  if (errorDeg == 0.0f || errorDeg * remainingAfterStep <= 0.0f) {
    *velocityDps = 0.0f;
    return targetDeg;
  }
  return nextDeg;
}

void updateCoordinatedRampedCommand(unsigned long now) {
  if (!g_coordinatedSextetTarget.rampInitialized) {
    return;
  }

  unsigned long dtMs = now - g_coordinatedSextetTarget.lastRampMs;
  if (dtMs == 0) {
    dtMs = TARGET_STREAM_INTERVAL_MS;
  }
  if (dtMs > 250) {
    dtMs = 250;
  }
  const float dtSec = static_cast<float>(dtMs) / 1000.0f;
  g_coordinatedSextetTarget.lastRampMs = now;

  const float targets[COORDINATED_JOINT_COUNT] = {
      g_coordinatedSextetTarget.joint1Deg,
      g_coordinatedSextetTarget.joint2Deg,
      g_coordinatedSextetTarget.joint3Deg,
      g_coordinatedSextetTarget.joint4Deg,
      g_coordinatedSextetTarget.joint5Deg,
      g_coordinatedSextetTarget.joint6Deg,
  };
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    g_coordinatedSextetTarget.commandedDeg[i] = stepRampLimited(
        g_coordinatedSextetTarget.commandedDeg[i],
        targets[i],
        &g_coordinatedSextetTarget.velocityDps[i],
        g_coordinatedMaxVelocityDps[i],
        g_coordinatedMaxAccelDps2[i],
        dtSec);
  }
}

SextetSendResult sendCoordinatedSextetBurst(float joint1Deg,
                                            float joint2Deg,
                                            float joint3Deg,
                                            float joint4Deg,
                                            float joint5Deg,
                                            float joint6Deg,
                                            bool auxGroupFirst) {
  SextetSendResult result = {true, true, true, true, true, true};

  auto sendOdriveGroup = [&]() {
    result.ok1 = sendRelativePositionDegreesToNode(NODE_ID_10, joint1Deg, false, false);
    result.ok2 = sendRelativePositionDegreesToNode(NODE_ID_11, joint2Deg, false, false);
    result.ok3 = sendRelativePositionDegreesToNode(NODE_ID_12, joint3Deg, false, false);
  };

  auto sendAuxGroup = [&]() {
    result.ok4 = sendJoint4RelativeDegrees(joint4Deg, false);
    result.ok5 = sendJoint5AbsoluteDegrees(joint5Deg, false);
    result.ok6 = sendJoint6AbsoluteDegrees(joint6Deg, false);
  };

  // Drain pending telemetry/replies first so the six-command burst starts from
  // a clean RX state.  The burst itself is split into two three-frame groups so
  // we never overfill the MCP2515's three TX buffers.
  processIncomingFrames();

  // Minimize inter-group gap to reduce temporal skew between joint groups.
  // 1 ms is enough for the MCP2515 to shift TXB0-TXB2 onto the wire (~0.3 ms
  // per SPI transaction at 10 MHz) without accumulating visible stagger.
  if (auxGroupFirst) {
    sendAuxGroup();
    delay(1);
    sendOdriveGroup();
  } else {
    sendOdriveGroup();
    delay(1);
    sendAuxGroup();
  }

  // Quick drain — 1 ms is enough for any reply frames triggered by the burst
  // to land in RXB0/RXB1 without adding to the next cycle's backlog.
  delay(1);
  processIncomingFrames();
  return result;
}

bool sextetSendSucceeded(const SextetSendResult &result) {
  return result.ok1 && result.ok2 && result.ok3 &&
         result.ok4 && result.ok5 && result.ok6;
}

void printSextetSendResult(const char *prefix, const SextetSendResult &result) {
  if (prefix != nullptr && prefix[0] != '\0') {
    Serial.print(prefix);
  }
  Serial.print("joint1=");
  Serial.print(result.ok1 ? "ok" : "fail");
  Serial.print(" joint2=");
  Serial.print(result.ok2 ? "ok" : "fail");
  Serial.print(" joint3=");
  Serial.print(result.ok3 ? "ok" : "fail");
  Serial.print(" joint4=");
  Serial.print(result.ok4 ? "ok" : "fail");
  Serial.print(" joint5=");
  Serial.print(result.ok5 ? "ok" : "fail");
  Serial.print(" joint6=");
  Serial.println(result.ok6 ? "ok" : "fail");
}

// -----------------------------
// CAN / MCP2515 configuration
// -----------------------------
bool applyCanMode() {
  switch (g_mode) {
    case CanMode::Normal:
      return canController.setNormalMode() == MCP2515::ERROR_OK;
    case CanMode::NormalOneShot:
      return canController.setNormalOneShotMode() == MCP2515::ERROR_OK;
    case CanMode::ListenOnly:
      return canController.setListenOnlyMode() == MCP2515::ERROR_OK;
    case CanMode::Loopback:
      return canController.setLoopbackMode() == MCP2515::ERROR_OK;
    default:
      return false;
  }
}

bool configureCan() {
  g_canControllerReady = false;
  canController.reset();

  const CAN_SPEED speed = bitrateValue(g_bitrate);
  auto osc = (g_oscillator == Oscillator::MHz8) ? MCP_8MHZ : MCP_16MHZ;

  const MCP2515::ERROR bitrateResult = canController.setBitrate(speed, osc);
  if (bitrateResult != MCP2515::ERROR_OK) {
    Serial.print("ERROR: failed to set MCP2515 bitrate code=");
    Serial.print(static_cast<int>(bitrateResult));
    Serial.print(" bitrate=");
    Serial.print(bitrateName(g_bitrate));
    Serial.print(" oscillator=");
    Serial.println(oscillatorName(g_oscillator));
    return false;
  }

  if (!configureReceiveFilters(g_auxFiltersEnabled)) {
    return false;
  }

  if (!applyCanMode()) {
    Serial.print("ERROR: failed to set MCP2515 mode=");
    Serial.println(modeName(g_mode));
    return false;
  }

  // Allow any in-flight frames from other nodes to complete before we attempt
  // the first transmission. setNormalMode() returns as soon as OPMOD confirms
  // normal mode; without this delay the very first TX after recovery can hit
  // a CAN error if the bus had activity at that exact moment, immediately
  // setting TXERR on TXB0 and causing the next sendMessage to return FAILTX.
  delay(5);

  // Drain any frames received during reconfiguration to prevent RX overflow
  // on the first main-loop processIncomingFrames() call after recovery.
  {
    can_frame discard = {};
    while (canController.readMessage(&discard) == MCP2515::ERROR_OK) {}
  }

  Serial.println("MCP2515 configured OK");
  g_canControllerReady = true;
  return true;
}

// -----------------------------
// Printing / decoding
// -----------------------------
void printRawFrame(const char *prefix, const can_frame &frame) {
  const uint32_t stdId = getStandardId(frame);
  const uint8_t nodeId = extractNodeId(stdId);
  const uint8_t cmdId  = extractCommandId(stdId);

  Serial.print(prefix);
  Serial.print(" id=0x");
  Serial.print(stdId, HEX);
  Serial.print(" node=0x");
  printHexByte(nodeId);
  Serial.print(" cmd=0x");
  printHexByte(cmdId);
  Serial.print(" (");
  Serial.print(commandName(cmdId));
  Serial.print(")");
  Serial.print(" ext=");
  Serial.print(isExtendedFrame(frame) ? 1 : 0);
  Serial.print(" rtr=");
  Serial.print(isRemoteFrame(frame) ? 1 : 0);
  Serial.print(" dlc=");
  Serial.print(frame.can_dlc);
  Serial.print(" data=");

  if (frame.can_dlc == 0) {
    Serial.print("(empty)");
  } else {
    for (uint8_t i = 0; i < frame.can_dlc; ++i) {
      if (i > 0) Serial.print(' ');
      printHexByte(frame.data[i]);
    }
  }

  Serial.println();
}

void printCachedAxisStatus(const char *prefix) {
  const NodeRuntimeState &state = selectedNodeState();
  Serial.print(prefix);
  Serial.print(" state=");
  Serial.print(axisStateName(state.lastAxisState));
  Serial.print(" heartbeat_errors=0x");
  Serial.print(state.lastHeartbeatActiveErrors, HEX);
  Serial.print(" active_errors=0x");
  Serial.print(state.lastErrorActiveErrors, HEX);
  Serial.print(" disarm_reason=0x");
  Serial.println(state.lastDisarmReason, HEX);
}

bool nodeHasBootConfiguration(const NodeRuntimeState &state) {
  return state.haveZeroReference &&
         state.haveLatestEncoderEstimate &&
         nodeIsInClosedLoop(state);
}

bool allNodesHaveBootConfiguration() {
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    if (!nodeHasBootConfiguration(g_nodeStates[i])) {
      return false;
    }
  }
  return true;
}

bool nodeHasActiveFault(const NodeRuntimeState &state) {
  if (!state.haveErrorStatus) {
    return false;
  }

  if (millis() - state.lastErrorMs > ERROR_STATUS_STALE_TIMEOUT_MS) {
    return false;
  }

  return state.lastErrorActiveErrors != 0 || state.lastDisarmReason != 0;
}

void clearFailsafeLatch(NodeRuntimeState &state) {
  if (state.failsafeLatched) {
    Serial.print("Failsafe cleared for node 0x");
    printHexByte(state.nodeId);
    Serial.println();
  }
  state.failsafeLatched = false;
  state.lastFailsafeMs = 0;
}

void clearHeartbeatWarning(NodeRuntimeState &state, bool logRecovery) {
  if (!state.heartbeatWarningActive) {
    return;
  }

  state.heartbeatWarningActive = false;
  state.lastHeartbeatWarningLogMs = 0;

  if (logRecovery) {
    Serial.print("Heartbeat warning cleared for node 0x");
    printHexByte(state.nodeId);
    Serial.println();
  }
}

void noteHeartbeatWarning(NodeRuntimeState &state, unsigned long now) {
  if (!state.haveHeartbeat || nodeHeartbeatIsHardTimedOut(state, now)) {
    return;
  }

  if (!state.heartbeatWarningActive) {
    state.heartbeatWarningActive = true;
    state.lastHeartbeatWarningLogMs = 0;
  }

  if (state.lastHeartbeatWarningLogMs != 0 &&
      now - state.lastHeartbeatWarningLogMs < HEARTBEAT_WARNING_LOG_INTERVAL_MS) {
    return;
  }

  state.lastHeartbeatWarningLogMs = now;

  Serial.print("WARNING: ODrive heartbeat degraded node=0x");
  printHexByte(state.nodeId);
  Serial.print(" age_ms=");
  Serial.print(heartbeatAgeMs(state, now));
  Serial.print(" axis=");
  Serial.print(axisStateName(state.lastAxisState));
  Serial.print(" heartbeat_errors=0x");
  Serial.print(state.lastHeartbeatActiveErrors, HEX);
  Serial.println(" continuing Set_Input_Pos streaming");
}

void printBootConfigurationSummary(const char *prefix) {
  if (prefix != nullptr && prefix[0] != '\0') {
    Serial.println(prefix);
  }

  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    const NodeRuntimeState &state = g_nodeStates[i];
    const unsigned long now = millis();
    Serial.print("  node 0x");
    printHexByte(state.nodeId);
    Serial.print(" zero=");
    Serial.print(state.haveZeroReference ? "yes" : "no");
    Serial.print(" encoder=");
    Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
    Serial.print(" heartbeat=");
    Serial.print(heartbeatStateName(state, now));
    Serial.print(" hb_age_ms=");
    if (state.haveHeartbeat) {
      Serial.print(heartbeatAgeMs(state, now));
    } else {
      Serial.print("n/a");
    }
    Serial.print(" axis=");
    Serial.print(axisStateName(state.lastAxisState));
    Serial.print(" failsafe=");
    Serial.println(state.failsafeLatched ? "yes" : "no");
  }
}

void clearActiveTargetForNode(uint8_t nodeId, const char *reason) {
  NodeRuntimeState *state = lookupNodeState(nodeId);
  if (state == nullptr || !state->haveActiveTarget) {
    return;
  }

  state->haveActiveTarget = false;
  Serial.print("Paused target streaming for node 0x");
  printHexByte(nodeId);
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println();
}

void recoverFromTxFailure(const can_frame &frame, MCP2515::ERROR sendResult) {
  if (!isExtendedFrame(frame) && !isRemoteFrame(frame)) {
    const uint32_t stdId = getStandardId(frame);
    const uint8_t nodeId = extractNodeId(stdId);
    const uint8_t cmdId = extractCommandId(stdId);
    if (cmdId == CMD_SET_INPUT_POS) {
      clearActiveTargetForNode(nodeId, "TX failure");
    }
  }

  // Read EFLG once before any clear operations so the diagnostic print and the
  // branch logic both see the same register snapshot.  The four SPI writes below
  // take ~400 µs, during which TEC can cross a threshold and change EFLG.
  const uint8_t eflg = canController.getErrorFlags();
  printCanControllerDiagnostics("TX failure", eflg);

  canController.clearTXInterrupts();
  canController.clearRXnOVRFlags();
  canController.clearERRIF();
  canController.clearMERR();

  if (sendResult == MCP2515::ERROR_ALLTXBUSY) {
    if (eflg & MCP2515::EFLG_TXBO) {
      Serial.println("All TX buffers busy and bus-off; reconfiguring controller");
      (void)configureCan();
    } else if (eflg & MCP2515::EFLG_TXEP) {
      // Do not reconfigure on TXEP: resetting TEC to zero would allow the next
      // send attempt through immediately, causing a reconfigure-loop.
      // The caller's EFLG-gated retry check will pause sends until bus recovers.
      Serial.println("All TX buffers busy and TX error-passive; backing off");
    } else {
      // EFLG=0 but all TX buffers still have TXREQ=1: frames are stuck waiting
      // for arbitration. This happens when a higher-priority node (e.g. LKTech
      // at 0x14E/0x14F) keeps winning bus arbitration over ODrive frames (0x200+).
      // In normal mode the MCP2515 never drops the pending frames itself, so the
      // buffers stay locked forever. Reset the controller to clear TXREQ on all
      // three buffers; the caller's retry logic will resend the frame.
      Serial.println("All TX buffers busy on healthy bus (arbitration starvation); resetting controller");
      (void)configureCan();
    }
    return;
  }

  if (sendResult == MCP2515::ERROR_FAILTX) {
    if (eflg & MCP2515::EFLG_TXBO) {
      Serial.println("MCP2515 entered bus-off; reconfiguring controller");
      (void)configureCan();
    } else if (eflg & MCP2515::EFLG_TXEP) {
      // Do not reconfigure on TXEP: resetting TEC to zero re-opens the send path
      // and causes a tight reconfigure-loop (configureCan every 100 ms).
      // Leave TXEP set; the EFLG-gated retry logic will skip sends until TEC drops
      // or escalates to TXBO, at which point the TXBO branch above takes over.
      Serial.println("MCP2515 TX error-passive; backing off without reconfigure");
    } else {
      // No bus-error flags (TXEP/TXBO both clear) but the library returned
      // ERROR_FAILTX because TXBnCTRL.TXERR is set from a previous error.
      // clearTXInterrupts() above clears CANINTF TX flags but does NOT touch
      // TXBnCTRL.TXERR.  The library checks TXERR before loading a new frame
      // and returns ERROR_FAILTX without trying other buffers, so the stuck
      // buffer blocks all further transmissions.  TEC=0 means the bus is
      // healthy; reset the controller to clear TXERR on all three buffers.
      Serial.println("TX failed with clean bus (stuck TXERR); resetting controller");
      (void)configureCan();
    }
  }
}

void engageNodeFailsafe(uint8_t nodeId, const char *reason, bool requestIdleIfReachable) {
  NodeRuntimeState *state = lookupNodeState(nodeId);
  if (state == nullptr || state->failsafeLatched) {
    return;
  }

  clearCoordinatedSextetTarget();
  clearHeartbeatWarning(*state, false);
  state->failsafeLatched = true;
  state->lastFailsafeMs = millis();
  state->haveActiveTarget = false;
  state->haveZeroReference = false;
  state->lastRelativeCommandTurns = 0.0f;
  state->lastRelativeCommandDegrees = 0.0f;

  Serial.print("FAILSAFE node=0x");
  printHexByte(nodeId);
  Serial.print(" reason=");
  Serial.println(reason != nullptr ? reason : "unknown");

  const bool recentlyReachable =
      state->haveHeartbeat &&
      !nodeHeartbeatIsHardTimedOut(*state, millis());
  if (requestIdleIfReachable && recentlyReachable && modeAllowsTransmit(g_mode)) {
    (void)sendAxisStateRequestToNode(nodeId, AXIS_STATE_IDLE, "Failsafe requested IDLE", true);
  }

  // Request error status from the failing node so the reason is captured
  // without the user needing to run 'r' manually.
  if (recentlyReachable && modeAllowsTransmit(g_mode)) {
    (void)sendRemoteRequestToNode(nodeId, CMD_GET_ERROR, 8);
    const unsigned long diagStart = millis();
    while (millis() - diagStart < 150) {
      processIncomingFrames();
      delay(1);
    }
    Serial.print("Node 0x");
    printHexByte(nodeId);
    Serial.print(" status: state=");
    Serial.print(axisStateName(state->lastAxisState));
    Serial.print(" heartbeat_errors=0x");
    Serial.print(state->lastHeartbeatActiveErrors, HEX);
    Serial.print(" active_errors=0x");
    Serial.print(state->lastErrorActiveErrors, HEX);
    Serial.print(" disarm_reason=0x");
    Serial.println(state->lastDisarmReason, HEX);
  }
}

void decodeHeartbeat(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 5) {
    if (g_showAllStandardFrames) {
      Serial.println("  Heartbeat frame too short");
    }
    return;
  }

  const uint32_t activeErrors = bytesToU32(frame.data);
  const uint8_t axisState = frame.data[4];
  const bool changed = !state.haveHeartbeat ||
                       activeErrors != state.lastHeartbeatActiveErrors ||
                       axisState != state.lastAxisState;

  state.haveHeartbeat = true;
  state.lastHeartbeatActiveErrors = activeErrors;
  state.lastAxisState = axisState;
  state.lastHeartbeatMs = millis();
  clearHeartbeatWarning(state, true);

  if (axisState == AXIS_STATE_CLOSED_LOOP_CONTROL && activeErrors == 0 && state.haveLatestEncoderEstimate) {
    clearFailsafeLatch(state);
  }

  if (changed && state.nodeId == g_activeNodeId) {
    Serial.print("Heartbeat state=");
    Serial.print(axisStateName(axisState));
    Serial.print(" active_errors=0x");
    Serial.println(activeErrors, HEX);
  }
}

void decodeGetError(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    if (g_showAllStandardFrames) {
      Serial.println("  Get_Error frame too short");
    }
    return;
  }

  const uint32_t activeErrors = bytesToU32(frame.data);
  const uint32_t disarmReason = bytesToU32(frame.data + 4);
  const bool changed = !state.haveErrorStatus ||
                       activeErrors != state.lastErrorActiveErrors ||
                       disarmReason != state.lastDisarmReason;

  state.haveErrorStatus = true;
  state.lastErrorActiveErrors = activeErrors;
  state.lastDisarmReason = disarmReason;
  state.lastErrorMs = millis();

  if (activeErrors != 0 || disarmReason != 0) {
    engageNodeFailsafe(state.nodeId, "odrive error/disarm", true);
  }

  if (state.nodeId == g_activeNodeId && (changed || g_showAllStandardFrames)) {
    Serial.print("ODrive active_errors=0x");
    Serial.print(activeErrors, HEX);
    Serial.print(" disarm_reason=0x");
    Serial.println(disarmReason, HEX);
  }
}

void decodeEncoderEstimates(NodeRuntimeState &state, const can_frame &frame) {
  if (frame.can_dlc < 8) {
    if (g_showAllStandardFrames) {
      Serial.println("  Encoder frame too short");
    }
    return;
  }

  const float posTurns = bytesToFloat(frame.data);
  const float velTurnsPerSecond = bytesToFloat(frame.data + 4);
  state.lastEncoderPosTurns = posTurns;
  state.lastEncoderVelTurnsPerSecond = velTurnsPerSecond;
  state.haveLatestEncoderEstimate = true;
  state.lastEncoderEstimateMs = millis();

  if (state.haveHeartbeat &&
      state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
      state.lastHeartbeatActiveErrors == 0) {
    clearFailsafeLatch(state);
  }

  if (state.nodeId == g_activeNodeId &&
      g_printEncoderTelemetry &&
      (state.lastEncoderTelemetryPrintMs == 0 ||
       state.lastEncoderEstimateMs - state.lastEncoderTelemetryPrintMs >= ENCODER_TELEMETRY_INTERVAL_MS)) {
    state.lastEncoderTelemetryPrintMs = state.lastEncoderEstimateMs;
    Serial.print("Encoder pos_turns=");
    Serial.print(posTurns, 6);
    Serial.print(" vel_turns_s=");
    Serial.println(velTurnsPerSecond, 6);
  }
}

void decodeFrame(const can_frame &frame) {
  if (isExtendedFrame(frame) || isRemoteFrame(frame)) {
    if (g_showAllStandardFrames) {
      printRawFrame("RX", frame);
    }
    return;
  }

  const uint32_t stdId = getStandardId(frame);
  const uint8_t nodeId = extractNodeId(stdId);
  const uint8_t cmdId  = extractCommandId(stdId);

  NodeRuntimeState *state = lookupNodeState(nodeId);
  if (state == nullptr) {
    if (g_showAllStandardFrames) {
      printRawFrame("RX", frame);
    }
    return;
  }

  if (g_showAllStandardFrames) {
    printRawFrame("RX", frame);
  }

  switch (cmdId) {
    case CMD_HEARTBEAT:
      decodeHeartbeat(*state, frame);
      break;
    case CMD_GET_ERROR:
      decodeGetError(*state, frame);
      break;
    case CMD_GET_ENCODER_ESTIMATES:
      decodeEncoderEstimates(*state, frame);
      break;
    default:
      break;
  }
}

// -----------------------------
// TX helpers
// -----------------------------
bool sendFrame(const can_frame &frame, const char *message = nullptr, bool logFrame = true) {
  can_frame tx = frame;
  if (logFrame) {
    printRawFrame("TX", tx);
  }

  const MCP2515::ERROR sendResult = canController.sendMessage(&tx);
  if (sendResult != MCP2515::ERROR_OK) {
    const uint32_t arbitrationId = isExtendedFrame(tx)
                                       ? (tx.can_id & 0x1FFFFFFFUL)
                                       : getStandardId(tx);
    g_canLastFailedId = arbitrationId;
    g_canLastFailedCode = static_cast<int>(sendResult);
    noteCanSendResult(false, "odrive_send");
    Serial.print("ERROR: send failed code=");
    Serial.print(static_cast<int>(sendResult));
    Serial.print(" meaning=");
    Serial.print(mcp2515ErrorName(static_cast<int>(sendResult)));
    Serial.print(" bitrate=");
    Serial.print(bitrateName(g_bitrate));
    Serial.print(" oscillator=");
    Serial.print(oscillatorName(g_oscillator));
    Serial.print(" mode=");
    Serial.print(modeName(g_mode));
    Serial.print(" id=0x");
    Serial.println(arbitrationId, HEX);
    recoverFromTxFailure(tx, sendResult);
    return false;
  }

  noteCanSendResult(true, "odrive_send");

  if (message != nullptr) {
    Serial.println(message);
  }

  return true;
}

void noteCanSendResult(bool success, const char *context) {
  const unsigned long now = millis();
  if (success) {
    if (g_canFaultWindowStartMs != 0 &&
        now - g_canFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
      g_canSendFailCount = 0;
      g_canFaultWindowStartMs = 0;
    }
    return;
  }
  if (g_canFaultWindowStartMs == 0 || now - g_canFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
    g_canFaultWindowStartMs = now;
    g_canSendFailCount = 0;
  }
  ++g_canSendFailCountTotal;
  if (g_canSendFailCount < 255) {
    ++g_canSendFailCount;
  }
  if (g_canSendFailCount >= activeCanSendFailThreshold() &&
      g_firmwareState != FirmwareState::FaultCan) {
    char reason[96] = {};
    snprintf(reason, sizeof(reason), "can_bus_lost context=%s failures=%u",
             context != nullptr ? context : "unknown", g_canSendFailCount);
    enterCanFault(reason);
  }
}

void noteMotorCommandResult(bool success, const char *context) {
  const unsigned long now = millis();
  if (success) {
    if (g_motorFaultWindowStartMs != 0 &&
        now - g_motorFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
      g_motorCommandFailCount = 0;
      g_motorFaultWindowStartMs = 0;
    }
    return;
  }
  if (g_motorFaultWindowStartMs == 0 || now - g_motorFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
    g_motorFaultWindowStartMs = now;
    g_motorCommandFailCount = 0;
  }
  if (g_motorCommandFailCount < 255) {
    ++g_motorCommandFailCount;
  }
  if (g_motorCommandFailCount >= activeMotorCommandFailThreshold()) {
    char reason[96] = {};
    snprintf(reason, sizeof(reason), "motor_command_failed context=%s failures=%u",
             context != nullptr ? context : "unknown", g_motorCommandFailCount);
    enterCanFault(reason);
  }
}

void sendAxisStateRequest(uint32_t state) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, CMD_SET_AXIS_STATE);
  frame.can_dlc = 4;
  frame.data[0] = static_cast<uint8_t>( state        & 0xFF);
  frame.data[1] = static_cast<uint8_t>((state >> 8 ) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((state >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((state >> 24) & 0xFF);

  if (state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
    sendFrame(frame, "Requested CLOSED_LOOP_CONTROL");
  } else if (state == AXIS_STATE_IDLE) {
    sendFrame(frame, "Requested IDLE");
  } else {
    sendFrame(frame, "Requested axis state");
  }
}

bool sendAxisStateRequestToNode(uint8_t nodeId, uint32_t state, const char *message, bool logFrame) {
  can_frame frame = {};
  frame.can_id = makeCanId(nodeId, CMD_SET_AXIS_STATE);
  frame.can_dlc = 4;
  frame.data[0] = static_cast<uint8_t>( state        & 0xFF);
  frame.data[1] = static_cast<uint8_t>((state >> 8 ) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((state >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((state >> 24) & 0xFF);
  return sendFrame(frame, message, logFrame);
}

bool sendAxisStateRequest(uint32_t state, const char *message, bool logFrame) {
  return sendAxisStateRequestToNode(g_activeNodeId, state, message, logFrame);
}

void sendRemoteRequest(uint8_t cmd, uint8_t dlc, const char *message) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, cmd) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  sendFrame(frame, message);
}

bool sendRemoteRequest(uint8_t cmd, uint8_t dlc, const char *message, bool logFrame) {
  can_frame frame = {};
  frame.can_id = makeCanId(g_activeNodeId, cmd) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  return sendFrame(frame, message, logFrame);
}

bool sendRemoteRequestToNode(uint8_t nodeId, uint8_t cmd, uint8_t dlc) {
  can_frame frame = {};
  frame.can_id = makeCanId(nodeId, cmd) | CAN_RTR_FLAG;
  frame.can_dlc = dlc;
  return sendFrame(frame, nullptr, false);
}

void sendInputPositionToNode(uint8_t nodeId,
                             float absoluteTurns,
                             int16_t velFeedforward = 0,
                             int16_t torqueFeedforward = 0,
                             const char *message = "Sent Set_Input_Pos",
                             bool logFrame = true) {
  can_frame frame = {};
  frame.can_id = makeCanId(nodeId, CMD_SET_INPUT_POS);
  frame.can_dlc = 8;
  writeFloatToBytes(absoluteTurns, &frame.data[0]);
  writeI16ToBytes(velFeedforward, &frame.data[4]);
  writeI16ToBytes(torqueFeedforward, &frame.data[6]);
  sendFrame(frame, message, logFrame);
}

void sendInputPosition(float absoluteTurns,
                       int16_t velFeedforward = 0,
                       int16_t torqueFeedforward = 0,
                       const char *message = "Sent Set_Input_Pos",
                       bool logFrame = true) {
  sendInputPositionToNode(g_activeNodeId, absoluteTurns, velFeedforward, torqueFeedforward, message, logFrame);
}

void sendSelfTestFrame() {
  can_frame frame = {};
  frame.can_id = 0x123;
  frame.can_dlc = 4;
  frame.data[0] = 0xDE;
  frame.data[1] = 0xAD;
  frame.data[2] = 0xBE;
  frame.data[3] = 0xEF;
  sendFrame(frame, "Sent self-test frame 123#DEADBEEF");
}

void captureZeroFromLatestPosition() {
  NodeRuntimeState &state = selectedNodeState();
  clearCoordinatedSextetTarget();

  if (!state.haveLatestEncoderEstimate) {
    Serial.println("Cannot set zero yet: no encoder estimate received. Wait for 0x249 or send 'e'.");
    return;
  }

  state.zeroReferenceTurns = state.lastEncoderPosTurns;
  state.haveZeroReference = true;
  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = 0.0f;
  state.lastRelativeCommandDegrees = 0.0f;
  state.lastAbsoluteCommandTurns = state.zeroReferenceTurns;
  state.lastTargetStreamMs = millis();
  clearFailsafeLatch(state);

  Serial.print("Zero reference captured at ");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.println(" turns");

  if (modeAllowsTransmit(g_mode)) {
    sendInputPosition(state.lastAbsoluteCommandTurns, 0, 0, "Holding current position as zero reference");
  }
}

void sendRelativePositionTurns(float relativeTurns) {
  NodeRuntimeState &state = selectedNodeState();
  clearCoordinatedSextetTarget();

  if (!ensureActiveNodeReadyForMotion("turn command")) {
    Serial.println("Cannot send relative turns: automatic bringup did not complete.");
    return;
  }

  const float relativeJointDegrees = motorTurnsToJointDegrees(g_activeNodeId, relativeTurns);
  if (!jointDegreesWithinLimits(g_activeNodeId, relativeJointDegrees)) {
    Serial.print("Rejected turn command: joint target ");
    Serial.print(relativeJointDegrees, 3);
    Serial.print(" deg is outside limits [");
    Serial.print(nodeMinJointDegrees(g_activeNodeId), 1);
    Serial.print(", ");
    Serial.print(nodeMaxJointDegrees(g_activeNodeId), 1);
    Serial.println("]");
    return;
  }

  const float absoluteTurns = state.zeroReferenceTurns + relativeTurns;
  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = relativeTurns;
  state.lastRelativeCommandDegrees = relativeJointDegrees;
  state.lastAbsoluteCommandTurns = absoluteTurns;
  state.lastTargetStreamMs = millis();

  Serial.print("Relative target=");
  Serial.print(relativeTurns, 6);
  Serial.print(" turns -> absolute target=");
  Serial.print(absoluteTurns, 6);
  Serial.println(" turns");

  sendInputPosition(absoluteTurns);
}

void sendRelativePositionDegrees(float relativeDegrees) {
  NodeRuntimeState &state = selectedNodeState();
  clearCoordinatedSextetTarget();

  if (!ensureActiveNodeReadyForMotion("angle command")) {
    Serial.println("Cannot send relative angle: automatic bringup did not complete.");
    return;
  }

  if (!jointDegreesWithinLimits(g_activeNodeId, relativeDegrees)) {
    Serial.print("Rejected angle command: ");
    Serial.print(relativeDegrees, 3);
    Serial.print(" deg is outside limits [");
    Serial.print(nodeMinJointDegrees(g_activeNodeId), 1);
    Serial.print(", ");
    Serial.print(nodeMaxJointDegrees(g_activeNodeId), 1);
    Serial.println("]");
    return;
  }

  const float relativeTurns = jointDegreesToMotorTurns(g_activeNodeId, relativeDegrees);
  const float absoluteTurns = state.zeroReferenceTurns + relativeTurns;

  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = relativeTurns;
  state.lastRelativeCommandDegrees = relativeDegrees;
  state.lastAbsoluteCommandTurns = absoluteTurns;
  state.lastTargetStreamMs = millis();

  Serial.print("Relative target=");
  Serial.print(relativeDegrees, 3);
  Serial.print(" deg -> motor_delta=");
  Serial.print(relativeTurns, 6);
  Serial.print(" turns -> absolute target=");
  Serial.print(absoluteTurns, 6);
  Serial.println(" turns");

  sendInputPosition(absoluteTurns);
}

bool sendRelativePositionDegreesToNode(uint8_t nodeId,
                                       float relativeDegrees,
                                       bool printSummary,
                                       bool logCanFrame) {
  const uint8_t previousActiveNodeId = g_activeNodeId;
  g_activeNodeId = nodeId;
  NodeRuntimeState &state = selectedNodeState();

  if (!ensureActiveNodeReadyForMotion("moveit command")) {
    if (printSummary) {
      Serial.print("Cannot send relative angle for node 0x");
      printHexByte(nodeId);
      Serial.println(": automatic bringup did not complete.");
    }
    g_activeNodeId = previousActiveNodeId;
    return false;
  }

  if (!jointDegreesWithinLimits(g_activeNodeId, relativeDegrees)) {
    if (printSummary) {
      Serial.print("Rejected angle command for node 0x");
      printHexByte(nodeId);
      Serial.print(": ");
      Serial.print(relativeDegrees, 3);
      Serial.print(" deg is outside limits [");
      Serial.print(nodeMinJointDegrees(g_activeNodeId), 1);
      Serial.print(", ");
      Serial.print(nodeMaxJointDegrees(g_activeNodeId), 1);
      Serial.println("]");
    }
    g_activeNodeId = previousActiveNodeId;
    return false;
  }

  const float relativeTurns = jointDegreesToMotorTurns(g_activeNodeId, relativeDegrees);
  const float absoluteTurns = state.zeroReferenceTurns + relativeTurns;

  state.haveActiveTarget = true;
  state.lastRelativeCommandTurns = relativeTurns;
  state.lastRelativeCommandDegrees = relativeDegrees;
  state.lastAbsoluteCommandTurns = absoluteTurns;
  state.lastTargetStreamMs = millis();

  if (printSummary) {
    Serial.print("Relative target node=0x");
    printHexByte(nodeId);
    Serial.print(" deg=");
    Serial.print(relativeDegrees, 3);
    Serial.print(" -> motor_delta=");
    Serial.print(relativeTurns, 6);
    Serial.print(" turns -> absolute target=");
    Serial.print(absoluteTurns, 6);
    Serial.println(" turns");
  }

  sendInputPositionToNode(nodeId, absoluteTurns, 0, 0, printSummary ? "Sent Set_Input_Pos" : nullptr, logCanFrame);
  g_activeNodeId = previousActiveNodeId;
  return true;
}

bool sendJoint4RelativeDegrees(float relativeDegrees, bool printSummary) {
  if (relativeDegrees < JOINT4_MIN_DEG || relativeDegrees > JOINT4_MAX_DEG) {
    if (printSummary) {
      Serial.print("Rejected joint4 relative target ");
      Serial.print(relativeDegrees, 3);
      Serial.print(" deg outside limits [");
      Serial.print(JOINT4_MIN_DEG, 1);
      Serial.print(", ");
      Serial.print(JOINT4_MAX_DEG, 1);
      Serial.println("]");
    }
    return false;
  }

  // Speed and software zero must have been configured at boot (setup()).  Do NOT call
  // ze300_joint4::initialize() here — it blocks for up to 1750 ms
  // (5 retries × 250 ms waitForReply + inter-retry delays), which stalls
  // the coordinated sextet burst while other joints already have CAN frames
  // in flight, creating dangerous split-motion.
  if (!g_joint4State.speedConfigured || !g_joint4State.haveSoftwareZero) {
    if (printSummary) {
      Serial.println("joint4 ZE300 speed/software zero not configured at boot — cannot command in hot path");
    }
    return false;
  }

  if (!g_auxTargets.joint4HaveTarget && fabs(relativeDegrees) > JOINT4_FIRST_COMMAND_MAX_DEG) {
    if (printSummary) {
      Serial.print("Rejected first joint4 relative target ");
      Serial.print(relativeDegrees, 3);
      Serial.print(" deg; first command limit is +/-");
      Serial.print(JOINT4_FIRST_COMMAND_MAX_DEG, 1);
      Serial.println(" deg from captured software zero");
    }
    return false;
  }

  const bool ok = ze300_joint4::sendRelativeDegrees(canController, g_joint4State, relativeDegrees);
  if (ok) {
    g_auxTargets.joint4HaveTarget = true;
    g_auxTargets.joint4Deg = relativeDegrees;
    g_auxTargets.joint4LastTxMs = millis();
  }
  if (printSummary) {
    Serial.print("Joint4 relative target=");
    Serial.print(relativeDegrees, 3);
    Serial.print(" deg zero=");
    Serial.print(g_joint4State.zeroOffsetDeg, 3);
    Serial.print(" abs=");
    Serial.print(g_joint4State.lastAbsoluteDeg, 3);
    Serial.print(" deg send=");
    Serial.println(ok ? "ok" : "fail");
  }
  return ok;
}

bool sendJoint5AbsoluteDegrees(float absoluteDegrees, bool printSummary) {
  if (absoluteDegrees < JOINT5_MIN_DEG || absoluteDegrees > JOINT5_MAX_DEG) {
    if (printSummary) {
      Serial.print("Rejected joint5 target ");
      Serial.print(absoluteDegrees, 3);
      Serial.print(" deg outside limits [");
      Serial.print(JOINT5_MIN_DEG, 1);
      Serial.print(", ");
      Serial.print(JOINT5_MAX_DEG, 1);
      Serial.println("]");
    }
    return false;
  }

  if (!g_joint56State.joints[0].haveZero) {
    if (printSummary) {
      Serial.println("joint5 zero capture not ready");
    }
    return false;
  }

  const bool ok = lktech_joint56::sendOutputDegrees(canController, g_joint56State, 0, absoluteDegrees);
  if (ok) {
    g_auxTargets.joint5HaveTarget = true;
    g_auxTargets.joint5Deg = absoluteDegrees;
    g_auxTargets.joint5LastTxMs = millis();
  } else {
    const uint8_t eflg = canController.getErrorFlags();
    Serial.print("joint5 send failed EFLG=0x");
    Serial.println(eflg, HEX);
  }
  if (printSummary) {
    Serial.print("Joint5 target=");
    Serial.print(absoluteDegrees, 3);
    Serial.print(" deg send=");
    Serial.println(ok ? "ok" : "fail");
  }
  return ok;
}

bool sendJoint6AbsoluteDegrees(float absoluteDegrees, bool printSummary) {
  if (absoluteDegrees < JOINT6_MIN_DEG || absoluteDegrees > JOINT6_MAX_DEG) {
    if (printSummary) {
      Serial.print("Rejected joint6 target ");
      Serial.print(absoluteDegrees, 3);
      Serial.print(" deg outside limits [");
      Serial.print(JOINT6_MIN_DEG, 1);
      Serial.print(", ");
      Serial.print(JOINT6_MAX_DEG, 1);
      Serial.println("]");
    }
    return false;
  }

  if (!g_joint56State.joints[1].haveZero) {
    if (printSummary) {
      Serial.println("joint6 zero capture not ready");
    }
    return false;
  }

  const bool ok = lktech_joint56::sendOutputDegrees(canController, g_joint56State, 1, absoluteDegrees);
  if (ok) {
    g_auxTargets.joint6HaveTarget = true;
    g_auxTargets.joint6Deg = absoluteDegrees;
    g_auxTargets.joint6LastTxMs = millis();
  } else {
    const uint8_t eflg = canController.getErrorFlags();
    Serial.print("joint6 send failed EFLG=0x");
    Serial.println(eflg, HEX);
  }
  if (printSummary) {
    Serial.print("Joint6 target=");
    Serial.print(absoluteDegrees, 3);
    Serial.print(" deg send=");
    Serial.println(ok ? "ok" : "fail");
  }
  return ok;
}

bool handleMoveItBridgeCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }

  const bool is_triplet =
      (strncmp(line, "j ", 2) == 0 || strncmp(line, "js ", 3) == 0);
  const bool is_quartet =
      (strncmp(line, "j4 ", 3) == 0 || strncmp(line, "js4 ", 4) == 0);
  const bool is_sextet =
      (strncmp(line, "j6 ", 3) == 0 || strncmp(line, "js6 ", 4) == 0);
  if (!(is_triplet || is_quartet || is_sextet)) {
    return false;
  }

  const char *valueText = line;
  if (is_sextet) {
    if (!g_motorsInitialized || g_firmwareState != FirmwareState::MotorsReady) {
      Serial.print("MoveIt sextet rejected: firmware_state=");
      Serial.println(firmwareStateName(g_firmwareState));
      return true;
    }
    valueText += (line[1] == 's' ? 4 : 3);
  } else if (is_quartet) {
    valueText += (line[1] == 's' ? 4 : 3);
  } else {
    valueText += (line[1] == 's' ? 3 : 2);
  }
  char buffer[SERIAL_LINE_BUFFER_SIZE] = {};
  strncpy(buffer, valueText, sizeof(buffer) - 1);

  char *savePtr = nullptr;
  char *tokens[7] = {};
  for (size_t i = 0; i < 7; ++i) {
    tokens[i] = strtok_r(i == 0 ? buffer : nullptr, " \t", &savePtr);
  }

  if (is_triplet &&
      (tokens[0] == nullptr || tokens[1] == nullptr || tokens[2] == nullptr || tokens[3] != nullptr)) {
    Serial.println("Usage: j <joint1_deg> <joint2_deg> <joint3_deg>");
    return true;
  }
  if (is_quartet &&
      (tokens[0] == nullptr || tokens[1] == nullptr || tokens[2] == nullptr || tokens[3] == nullptr || tokens[4] != nullptr)) {
    Serial.println("Usage: j4 <joint1_deg> <joint2_deg> <joint3_deg> <joint4_deg>");
    return true;
  }
  if (is_sextet &&
      (tokens[0] == nullptr || tokens[1] == nullptr || tokens[2] == nullptr ||
       tokens[3] == nullptr || tokens[4] == nullptr || tokens[5] == nullptr || tokens[6] != nullptr)) {
    Serial.println("Usage: j6 <joint1_deg> <joint2_deg> <joint3_deg> <joint4_deg> <joint5_deg> <joint6_deg>");
    return true;
  }

  char *endPtrs[6] = {};
  const float joint1Deg = strtof(tokens[0], &endPtrs[0]);
  const float joint2Deg = strtof(tokens[1], &endPtrs[1]);
  const float joint3Deg = strtof(tokens[2], &endPtrs[2]);
  const float joint4Deg = (is_quartet || is_sextet) ? strtof(tokens[3], &endPtrs[3]) : 0.0f;
  const float joint5Deg = is_sextet ? strtof(tokens[4], &endPtrs[4]) : 0.0f;
  const float joint6Deg = is_sextet ? strtof(tokens[5], &endPtrs[5]) : 0.0f;
  if (endPtrs[0] == tokens[0] || *endPtrs[0] != '\0' ||
      endPtrs[1] == tokens[1] || *endPtrs[1] != '\0' ||
      endPtrs[2] == tokens[2] || *endPtrs[2] != '\0' ||
      ((is_quartet || is_sextet) && (endPtrs[3] == tokens[3] || *endPtrs[3] != '\0')) ||
      (is_sextet && (endPtrs[4] == tokens[4] || *endPtrs[4] != '\0' ||
                     endPtrs[5] == tokens[5] || *endPtrs[5] != '\0'))) {
    if (is_sextet) {
      Serial.println("Invalid joint sextet. Example: j6 10.0 20.0 -15.0 5.0 8.0 -8.0");
    } else if (is_quartet) {
      Serial.println("Invalid joint quartet. Example: j4 10.0 20.0 -15.0 5.0");
    } else {
      Serial.println("Invalid joint triplet. Example: j 10.0 20.0 -15.0");
    }
    return true;
  }

  if (is_sextet) {
    NodeRuntimeState *state10 = lookupNodeState(NODE_ID_10);
    NodeRuntimeState *state11 = lookupNodeState(NODE_ID_11);
    NodeRuntimeState *state12 = lookupNodeState(NODE_ID_12);
    const bool preflightOk =
        state10 != nullptr && state11 != nullptr && state12 != nullptr &&
        nodeIsReadyForMotion(*state10) &&
        nodeIsReadyForMotion(*state11) &&
        nodeIsReadyForMotion(*state12) &&
        jointDegreesWithinLimits(NODE_ID_10, joint1Deg) &&
        jointDegreesWithinLimits(NODE_ID_11, joint2Deg) &&
        jointDegreesWithinLimits(NODE_ID_12, joint3Deg) &&
        joint4Deg >= JOINT4_MIN_DEG && joint4Deg <= JOINT4_MAX_DEG &&
        joint5Deg >= JOINT5_MIN_DEG && joint5Deg <= JOINT5_MAX_DEG &&
        joint6Deg >= JOINT6_MIN_DEG && joint6Deg <= JOINT6_MAX_DEG &&
        g_joint56State.joints[0].haveZero &&
        g_joint56State.joints[1].haveZero;
    if (!preflightOk) {
      Serial.println("MoveIt sextet rejected by preflight check.");
      return true;
    }

    // Ginkgo pattern: store the target, let serviceTargetStreaming() send CAN
    // at 20 Hz.  The old code called sendCoordinatedSextetBurst() here on every
    // serial command (50 Hz from the bridge) PLUS re-streamed at 20 Hz —
    // producing ~420 CAN frames/sec which overwhelmed the MCP2515's 3 TX
    // buffers, causing ERROR_FAILTX cascading into bus-off.
    const unsigned long committedMs = millis();
    storeCoordinatedSextetTarget(joint1Deg, joint2Deg, joint3Deg,
                                 joint4Deg, joint5Deg, joint6Deg,
                                 committedMs);

    Serial.print("j6 received; target updated epoch=");
    Serial.print(g_coordinatedSextetTarget.epoch);
    Serial.print(": joint1=");
    Serial.print(joint1Deg, 3);
    Serial.print(" joint2=");
    Serial.print(joint2Deg, 3);
    Serial.print(" joint3=");
    Serial.print(joint3Deg, 3);
    Serial.print(" joint4=");
    Serial.print(joint4Deg, 3);
    Serial.print(" joint5=");
    Serial.print(joint5Deg, 3);
    Serial.print(" joint6=");
    Serial.println(joint6Deg, 3);
    return true;
  }

  clearCoordinatedSextetTarget();

  // Drain RX buffers before sending to all joints. ODrive encoder estimates
  // and LKTech/ZE300 replies accumulate during the ~1 ms of sequential SPI
  // writes and overflow RXB0/RXB1 if not consumed first.
  processIncomingFrames();

  const bool ok1 = sendRelativePositionDegreesToNode(NODE_ID_10, joint1Deg, false, false);
  const bool ok2 = sendRelativePositionDegreesToNode(NODE_ID_11, joint2Deg, false, false);
  const bool ok3 = sendRelativePositionDegreesToNode(NODE_ID_12, joint3Deg, false, false);
  const bool ok4 = is_quartet ? sendJoint4RelativeDegrees(joint4Deg, false) : true;
  const bool ok5 = true;
  const bool ok6 = true;

  if (!(ok1 && ok2 && ok3 && ok4 && ok5 && ok6)) {
    Serial.print(is_sextet ? "MoveIt sextet partially applied: joint1=" :
                (is_quartet ? "MoveIt quartet partially applied: joint1=" : "MoveIt triplet partially applied: joint1="));
    Serial.print(ok1 ? "ok" : "fail");
    Serial.print(" joint2=");
    Serial.print(ok2 ? "ok" : "fail");
    Serial.print(" joint3=");
    Serial.print(ok3 ? "ok" : "fail");
    if (is_quartet || is_sextet) {
      Serial.print(" joint4=");
      Serial.print(ok4 ? "ok" : "fail");
    }
    if (is_sextet) {
      Serial.print(" joint5=");
      Serial.print(ok5 ? "ok" : "fail");
      Serial.print(" joint6=");
      Serial.print(ok6 ? "ok" : "fail");
    }
    Serial.println();
    return true;
  }

  Serial.print(is_sextet ? "MoveIt sextet applied: joint1=" :
              (is_quartet ? "MoveIt quartet applied: joint1=" : "MoveIt triplet applied: joint1="));
  Serial.print(joint1Deg, 3);
  Serial.print(" joint2=");
  Serial.print(joint2Deg, 3);
  Serial.print(" joint3=");
  Serial.print(joint3Deg, 3);
  if (is_quartet || is_sextet) {
    Serial.print(" joint4=");
    Serial.print(joint4Deg, 3);
  }
  if (is_sextet) {
    Serial.print(" joint5=");
    Serial.print(joint5Deg, 3);
    Serial.print(" joint6=");
    Serial.print(joint6Deg, 3);
  }
  Serial.println();
  return true;
}

bool handleMoveItIdleCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }

  if (!(strcmp(line, "jx") == 0 || strcmp(line, "jx4") == 0 || strcmp(line, "jidle4") == 0 ||
        strcmp(line, "jx6") == 0 || strcmp(line, "jidle6") == 0)) {
    return false;
  }

  sendAxisStateRequestToNode(NODE_ID_10, AXIS_STATE_IDLE, nullptr, false);
  sendAxisStateRequestToNode(NODE_ID_11, AXIS_STATE_IDLE, nullptr, false);
  sendAxisStateRequestToNode(NODE_ID_12, AXIS_STATE_IDLE, nullptr, false);
  NodeRuntimeState *state10 = lookupNodeState(NODE_ID_10);
  NodeRuntimeState *state11 = lookupNodeState(NODE_ID_11);
  NodeRuntimeState *state12 = lookupNodeState(NODE_ID_12);
  if (state10 != nullptr) {
    state10->haveActiveTarget = false;
  }
  if (state11 != nullptr) {
    state11->haveActiveTarget = false;
  }
  if (state12 != nullptr) {
    state12->haveActiveTarget = false;
  }
  const bool ze300_ok = ze300_joint4::disable(canController);
  const bool lk_ok = lktech_joint56::stopAll(canController, g_joint56State);
  clearCoordinatedSextetTarget();
  g_auxTargets.joint4HaveTarget = false;
  g_auxTargets.joint5HaveTarget = false;
  g_auxTargets.joint6HaveTarget = false;
  g_motorsInitialized = false;
  if (g_firmwareState != FirmwareState::FaultCan) {
    g_firmwareState = FirmwareState::MotorsUninitialized;
  }
  Serial.print("MoveIt idle/disable applied (motors loose; re-init required); joint4=");
  Serial.print(ze300_ok ? "ok" : "fail");
  Serial.print(" joint5-6=");
  Serial.println(lk_ok ? "ok" : "fail");
  return true;
}

void disarmAllMotors(const char *reason) {
  clearCoordinatedSextetTarget();
  g_auxTargets.joint4HaveTarget = false;
  g_auxTargets.joint5HaveTarget = false;
  g_auxTargets.joint6HaveTarget = false;
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    g_nodeStates[i].haveActiveTarget = false;
  }

  // Best effort only: if CAN is physically disconnected these commands cannot
  // reach the drivers. Mechanical/electrical protection must not rely on this.
  if (modeAllowsTransmit(g_mode)) {
    (void)sendAxisStateRequestToNode(NODE_ID_10, AXIS_STATE_IDLE, nullptr, false);
    (void)sendAxisStateRequestToNode(NODE_ID_11, AXIS_STATE_IDLE, nullptr, false);
    (void)sendAxisStateRequestToNode(NODE_ID_12, AXIS_STATE_IDLE, nullptr, false);
    (void)ze300_joint4::disable(canController);
    (void)lktech_joint56::stopAll(canController, g_joint56State);
  }
  g_motorsInitialized = false;
  if (g_firmwareState != FirmwareState::FaultCan) {
    g_firmwareState = FirmwareState::MotorsUninitialized;
  }
  Serial.print("disarm6ok reason=");
  Serial.println(reason != nullptr ? reason : "operator");
}

void enterCanFault(const char *reason) {
  if (g_firmwareState == FirmwareState::FaultCan) {
    return;
  }
  const bool motorsWereInitialized = g_motorsInitialized;
  g_firmwareState = FirmwareState::FaultCan;
  g_motorsInitialized = false;
  clearCoordinatedSextetTarget();
  g_auxTargets.joint4HaveTarget = false;
  g_auxTargets.joint5HaveTarget = false;
  g_auxTargets.joint6HaveTarget = false;
  snprintf(g_lastFirmwareFault, sizeof(g_lastFirmwareFault), "%s",
           reason != nullptr ? reason : "can_bus_lost");
  Serial.print("fault6 ");
  Serial.println(g_lastFirmwareFault);
  Serial.println("WARNING: CAN fault latched; motor stop cannot be guaranteed if drivers are unreachable");
  if (motorsWereInitialized && CAN_FAULT_DISARM_IF_REACHABLE && modeAllowsTransmit(g_mode)) {
    Serial.println("CAN fault policy: attempting one best-effort motor disable/stop");
    (void)sendAxisStateRequestToNode(NODE_ID_10, AXIS_STATE_IDLE, nullptr, false);
    (void)sendAxisStateRequestToNode(NODE_ID_11, AXIS_STATE_IDLE, nullptr, false);
    (void)sendAxisStateRequestToNode(NODE_ID_12, AXIS_STATE_IDLE, nullptr, false);
    (void)ze300_joint4::disable(canController);
    (void)lktech_joint56::stopAll(canController, g_joint56State);
  }
}

void printFirmwareStatus6() {
  const uint8_t eflg = canController.getErrorFlags();
  const unsigned long now = millis();
  if (g_canFaultWindowStartMs != 0 && now - g_canFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
    g_canSendFailCount = 0;
    g_canFaultWindowStartMs = 0;
  }
  if (g_motorFaultWindowStartMs != 0 && now - g_motorFaultWindowStartMs > CAN_FAULT_WINDOW_MS) {
    g_motorCommandFailCount = 0;
    g_motorFaultWindowStartMs = 0;
  }
  Serial.print("status6 state=");
  Serial.print(firmwareStateName(g_firmwareState));
  Serial.print(" can_ready=");
  Serial.print(g_canControllerReady && modeAllowsTransmit(g_mode) ? "true" : "false");
  Serial.print(" mcp2515_eflg=0x");
  Serial.print(eflg, HEX);
  Serial.print(" mcp2515_eflg_flags=");
  printEflgFlags(eflg);
  Serial.print(" can_send_fail_count_total=");
  Serial.print(g_canSendFailCountTotal);
  Serial.print(" can_send_fail_count_window=");
  Serial.print(g_canSendFailCount);
  Serial.print(" can_incomplete_burst_count=");
  Serial.print(g_canIncompleteBurstCount);
  Serial.print(" can_last_failed_id=0x");
  Serial.print(g_canLastFailedId, HEX);
  Serial.print(" can_last_failed_code=");
  Serial.print(g_canLastFailedCode);
  Serial.print(" can_last_failed_code_name=");
  Serial.print(mcp2515ErrorName(g_canLastFailedCode));
  Serial.print(" can_last_fault_reason=");
  Serial.print(g_lastFirmwareFault);
  Serial.print(" can_fault_strict_debug=");
  Serial.print(g_canFaultStrictDebug ? "true" : "false");
  Serial.print(" motors_initialized=");
  Serial.print(g_motorsInitialized ? "true" : "false");
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    Serial.print(" j");
    Serial.print(i + 1);
    Serial.print("_ready=");
    Serial.print(nodeIsReadyForMotion(g_nodeStates[i]) ? "true" : "false");
    Serial.print(" j");
    Serial.print(i + 1);
    Serial.print("_zero=");
    Serial.print(g_nodeStates[i].haveZeroReference ? "true" : "false");
    Serial.print(" j");
    Serial.print(i + 1);
    Serial.print("_err=0x");
    Serial.print(g_nodeStates[i].lastHeartbeatActiveErrors, HEX);
    Serial.print(" j");
    Serial.print(i + 1);
    Serial.print("_heartbeat_age_ms=");
    if (g_nodeStates[i].haveHeartbeat) {
      Serial.print(heartbeatAgeMs(g_nodeStates[i], now));
    } else {
      Serial.print("unavailable");
    }
    Serial.print(" j");
    Serial.print(i + 1);
    Serial.print("_command_fail_count=");
    Serial.print(g_jointCommandFailCount[i]);
  }
  Serial.print(" j4_ready=");
  Serial.print(g_joint4State.speedConfigured && g_joint4State.haveSoftwareZero ? "true" : "false");
  Serial.print(" j4_zero=");
  Serial.print(g_joint4State.haveSoftwareZero ? "true" : "false");
  Serial.print(" j4_command_fail_count=");
  Serial.print(g_jointCommandFailCount[3]);
  Serial.print(" j5_ready=");
  Serial.print(g_joint56State.joints[0].enabled && g_joint56State.joints[0].haveZero ? "true" : "false");
  Serial.print(" j5_zero=");
  Serial.print(g_joint56State.joints[0].haveZero ? "true" : "false");
  Serial.print(" j5_command_fail_count=");
  Serial.print(g_jointCommandFailCount[4]);
  Serial.print(" j6_ready=");
  Serial.print(g_joint56State.joints[1].enabled && g_joint56State.joints[1].haveZero ? "true" : "false");
  Serial.print(" j6_zero=");
  Serial.print(g_joint56State.joints[1].haveZero ? "true" : "false");
  Serial.print(" j6_command_fail_count=");
  Serial.print(g_jointCommandFailCount[5]);
  Serial.print(" coordinated_allowed=");
  Serial.print(g_motorsInitialized && g_firmwareState == FirmwareState::MotorsReady ? "true" : "false");
  Serial.print(" fault=");
  Serial.println(g_lastFirmwareFault);
}

bool initializeAllMotorsExplicitly() {
  if (g_firmwareState == FirmwareState::MotorsInitializing) {
    Serial.println("init6err already_initializing");
    return false;
  }

  g_firmwareState = FirmwareState::MotorsInitializing;
  g_motorsInitialized = false;
  g_canSendFailCount = 0;
  g_motorCommandFailCount = 0;
  snprintf(g_lastFirmwareFault, sizeof(g_lastFirmwareFault), "none");
  clearCoordinatedSextetTarget();
  initializeRuntimeStates();
  ze300_joint4::reset(g_joint4State);
  lktech_joint56::reset(g_joint56State);
  g_auxTargets = {};
  Serial.println("init6 started");

  if (!modeAllowsTransmit(g_mode)) {
    g_mode = CanMode::Normal;
  }
  g_auxFiltersEnabled = false;
  if (!configureCan()) {
    g_firmwareState = FirmwareState::MotorsUninitialized;
    Serial.println("init6err can_controller_not_ready");
    return false;
  }

  g_backgroundBringupEnabled = false;
  runAutomaticBringupAllNodes();
  if (g_firmwareState == FirmwareState::FaultCan || !allNodesHaveBootConfiguration()) {
    disarmAllMotors("init6_odrive_failed");
    Serial.println("init6err odrive_not_ready");
    return false;
  }

  g_auxFiltersEnabled = true;
  if (!configureCan()) {
    disarmAllMotors("init6_aux_filters_failed");
    Serial.println("init6err aux_filter_configuration_failed");
    return false;
  }
  if (!ze300_joint4::initialize(canController, g_joint4State)) {
    disarmAllMotors("init6_joint4_failed");
    Serial.println("init6err joint4_initialize_failed");
    return false;
  }
  if (!lktech_joint56::initialize(canController, g_joint56State)) {
    disarmAllMotors("init6_joint56_failed");
    Serial.println("init6err joint56_initialize_failed");
    return false;
  }

  armAuxJointHoldTargetsAfterBoot();
  if (g_firmwareState == FirmwareState::FaultCan ||
      !g_auxTargets.joint4HaveTarget || !g_auxTargets.joint5HaveTarget ||
      !g_auxTargets.joint6HaveTarget) {
    disarmAllMotors("init6_aux_hold_failed");
    Serial.println("init6err auxiliary_hold_failed");
    return false;
  }

  g_motorsInitialized = true;
  g_firmwareState = FirmwareState::MotorsReady;
  Serial.println("init6ok ready=true joints=1,2,3,4,5,6 zeros=true");
  return true;
}

bool handleLifecycleCommand(const char *line) {
  if (strcmp(line, "init6") == 0) {
    (void)initializeAllMotorsExplicitly();
    return true;
  }
  if (strcmp(line, "status6") == 0) {
    printFirmwareStatus6();
    return true;
  }
  if (strcmp(line, "disarm6") == 0) {
    disarmAllMotors("operator");
    return true;
  }
  if (strcmp(line, "hold6") == 0) {
    if (!g_motorsInitialized || g_firmwareState != FirmwareState::MotorsReady) {
      Serial.println("hold6err motors_not_ready");
    } else {
      clearCoordinatedSextetTarget();
      Serial.println("hold6ok last motor targets retained; coordinated trajectory stopped");
    }
    return true;
  }
  if (strcmp(line, "cfs6 on") == 0 || strcmp(line, "cfs6 off") == 0) {
    g_canFaultStrictDebug = strcmp(line, "cfs6 on") == 0;
    Serial.print("cfs6ok enabled=");
    Serial.print(g_canFaultStrictDebug ? "true" : "false");
    Serial.print(" send_threshold=");
    Serial.print(activeCanSendFailThreshold());
    Serial.print(" motor_threshold=");
    Serial.println(activeMotorCommandFailThreshold());
    return true;
  }
  if (strncmp(line, "testfault6", 10) == 0) {
    if (!g_canFaultStrictDebug) {
      Serial.println("testfault6err strict_debug_disabled");
      return true;
    }
    if (strcmp(line, "testfault6 can_bus_lost") != 0) {
      Serial.println("testfault6err usage=testfault6_can_bus_lost");
      return true;
    }
    enterCanFault("can_bus_lost injected=true");
    return true;
  }
  return false;
}

void printMotionState() {
  const NodeRuntimeState &state = selectedNodeState();
  const unsigned long now = millis();
  const float relativeTurnsFromZero = state.lastEncoderPosTurns - state.zeroReferenceTurns;
  const float relativeJointDegrees = motorTurnsToJointDegrees(g_activeNodeId, relativeTurnsFromZero);
  Serial.print("motion node=0x");
  printHexByte(g_activeNodeId);
  Serial.print(" axis_state=");
  Serial.print(axisStateName(state.lastAxisState));
  Serial.print(" heartbeat_state=");
  Serial.print(heartbeatStateName(state, now));
  Serial.print(" heartbeat_age_ms=");
  if (state.haveHeartbeat) {
    Serial.print(heartbeatAgeMs(state, now));
  } else {
    Serial.print("n/a");
  }
  Serial.print(" heartbeat_errors=0x");
  Serial.print(state.lastHeartbeatActiveErrors, HEX);
  Serial.print(" disarm=0x");
  Serial.print(state.lastDisarmReason, HEX);
  Serial.print(" have_encoder=");
  Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
  Serial.print(" pos_turns=");
  Serial.print(state.lastEncoderPosTurns, 6);
  Serial.print(" vel_turns_s=");
  Serial.print(state.lastEncoderVelTurnsPerSecond, 6);
  Serial.print(" have_zero=");
  Serial.print(state.haveZeroReference ? "yes" : "no");
  Serial.print(" zero_turns=");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.print(" joint_deg=");
  Serial.print(relativeJointDegrees, 3);
  Serial.print(" gear_ratio=");
  Serial.print(nodeGearRatio(g_activeNodeId), 1);
  Serial.print(" direction=");
  Serial.print(nodeDirection(g_activeNodeId), 1);
  Serial.print(" joint_limits=[");
  Serial.print(nodeMinJointDegrees(g_activeNodeId), 1);
  Serial.print(", ");
  Serial.print(nodeMaxJointDegrees(g_activeNodeId), 1);
  Serial.print("]");
  Serial.print(" stream=");
  Serial.print(g_streamLastTarget ? "on" : "off");
  Serial.print(" failsafe=");
  Serial.print(state.failsafeLatched ? "yes" : "no");
  Serial.print(" last_rel_cmd=");
  Serial.print(state.lastRelativeCommandTurns, 6);
  Serial.print(" last_rel_deg=");
  Serial.print(state.lastRelativeCommandDegrees, 3);
  Serial.print(" last_abs_cmd=");
  Serial.println(state.lastAbsoluteCommandTurns, 6);
}

bool waitForFreshOdriveEncoder(NodeRuntimeState &state, unsigned long requestStartMs) {
  if (!sendRemoteRequestToNode(state.nodeId, CMD_GET_ENCODER_ESTIMATES, 8)) {
    return false;
  }

  const unsigned long deadlineMs = millis() + JS6_SNAPSHOT_TIMEOUT_MS;
  while (millis() <= deadlineMs) {
    processIncomingFrames();
    if (state.haveLatestEncoderEstimate && state.lastEncoderEstimateMs >= requestStartMs) {
      return true;
    }
    delay(2);
  }
  return false;
}

bool readJointSnapshot(float *jointDeg, char *errorText, size_t errorTextSize) {
  if (jointDeg == nullptr || errorText == nullptr || errorTextSize == 0) {
    return false;
  }
  errorText[0] = '\0';

  for (size_t index = 0; index < SUPPORTED_NODE_COUNT; ++index) {
    NodeRuntimeState &state = g_nodeStates[index];
    if (!state.haveZeroReference) {
      snprintf(errorText, errorTextSize, "odrive_node_0x%02X_no_zero", state.nodeId);
      return false;
    }

    const unsigned long requestStartMs = millis();
    if (!waitForFreshOdriveEncoder(state, requestStartMs)) {
      snprintf(errorText, errorTextSize, "odrive_node_0x%02X_no_fresh_encoder", state.nodeId);
      return false;
    }

    const float relativeTurns = state.lastEncoderPosTurns - state.zeroReferenceTurns;
    jointDeg[index] = motorTurnsToJointDegrees(state.nodeId, relativeTurns);
  }

  if (!g_joint4State.haveSoftwareZero) {
    snprintf(errorText, errorTextSize, "joint4_no_software_zero");
    return false;
  }
  if (!ze300_joint4::readAbsolutePosition(canController, g_joint4State)) {
    snprintf(errorText, errorTextSize, "joint4_read_failed");
    return false;
  }
  jointDeg[3] = static_cast<float>(g_joint4State.lastRelativeDeg);

  for (size_t index = 0; index < 2; ++index) {
    if (!g_joint56State.joints[index].haveZero) {
      snprintf(errorText, errorTextSize, "joint%u_no_zero", static_cast<unsigned>(index + 5));
      return false;
    }

    double motorAngleDeg = 0.0;
    if (!lktech_joint56::readCurrentMotorAngle(canController, g_joint56State, index, &motorAngleDeg)) {
      snprintf(errorText, errorTextSize, "joint%u_read_failed", static_cast<unsigned>(index + 5));
      return false;
    }
    jointDeg[index + 4] = static_cast<float>(
        motorAngleDeg - g_joint56State.joints[index].zeroMotorDeg);
  }

  return true;
}

void printJointSnapshot() {
  float jointDeg[6] = {};
  char errorText[64] = {};
  if (!readJointSnapshot(jointDeg, errorText, sizeof(errorText))) {
    Serial.print("js6err ");
    Serial.println(errorText[0] != '\0' ? errorText : "snapshot_failed");
    return;
  }

  Serial.print("js6");
  for (size_t i = 0; i < 6; ++i) {
    Serial.print(' ');
    Serial.print(jointDeg[i], 3);
  }
  Serial.println();
}

bool handleJointSnapshotRequest(const char *line) {
  if (line == nullptr || strcmp(line, "qjs6") != 0) {
    return false;
  }
  printJointSnapshot();
  return true;
}

void armAuxJointHoldTargetsAfterBoot() {
  if (g_joint4State.speedConfigured && g_joint4State.haveSoftwareZero) {
    if (sendJoint4RelativeDegrees(0.0f, false)) {
      Serial.println("joint4 hold target armed at relative zero");
    } else {
      Serial.println("joint4 hold target failed after zero capture");
    }
  } else {
    Serial.println("joint4 hold target skipped: zero/speed not configured");
  }

  if (g_joint56State.joints[0].haveZero) {
    if (sendJoint5AbsoluteDegrees(0.0f, false)) {
      Serial.println("joint5 hold target armed at relative zero");
    } else {
      Serial.println("joint5 hold target failed after zero capture");
    }
  } else {
    Serial.println("joint5 hold target skipped: zero not captured");
  }

  if (g_joint56State.joints[1].haveZero) {
    if (sendJoint6AbsoluteDegrees(0.0f, false)) {
      Serial.println("joint6 hold target armed at relative zero");
    } else {
      Serial.println("joint6 hold target failed after zero capture");
    }
  } else {
    Serial.println("joint6 hold target skipped: zero not captured");
  }
}

bool parseSingleFloatArgument(const char *text, float *value) {
  if (text == nullptr || value == nullptr) {
    return false;
  }
  while (*text == ' ' || *text == '\t') {
    ++text;
  }
  if (*text == '\0') {
    return false;
  }
  char *endPtr = nullptr;
  const float parsed = strtof(text, &endPtr);
  if (endPtr == text || !isfinite(parsed)) {
    return false;
  }
  while (*endPtr == ' ' || *endPtr == '\t') {
    ++endPtr;
  }
  if (*endPtr != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

bool handleAuxJointTestCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }

  uint8_t joint = 0;
  const char *argText = nullptr;
  if (strncmp(line, "tj4 ", 4) == 0) {
    joint = 4;
    argText = line + 4;
  } else if (strncmp(line, "tj5 ", 4) == 0) {
    joint = 5;
    argText = line + 4;
  } else if (strncmp(line, "tj6 ", 4) == 0) {
    joint = 6;
    argText = line + 4;
  } else {
    return false;
  }

  if (g_coordinatedSextetTarget.haveTarget) {
    Serial.print("err tj");
    Serial.print(joint);
    Serial.println(" moveit_active");
    return true;
  }

  float targetDeg = 0.0f;
  if (!parseSingleFloatArgument(argText, &targetDeg)) {
    Serial.print("err tj");
    Serial.print(joint);
    Serial.println(" invalid_number");
    return true;
  }

  bool ok = false;
  const char *rejectReason = nullptr;
  if (joint == 4) {
    if (targetDeg < JOINT4_MIN_DEG || targetDeg > JOINT4_MAX_DEG) {
      rejectReason = "out_of_range";
    } else if (!g_joint4State.speedConfigured || !g_joint4State.haveSoftwareZero) {
      rejectReason = "not_ready";
    } else {
      ok = sendJoint4RelativeDegrees(targetDeg, false);
    }
  } else if (joint == 5) {
    if (targetDeg < JOINT5_MIN_DEG || targetDeg > JOINT5_MAX_DEG) {
      rejectReason = "out_of_range";
    } else if (!g_joint56State.joints[0].haveZero) {
      rejectReason = "not_ready";
    } else {
      ok = sendJoint5AbsoluteDegrees(targetDeg, false);
    }
  } else if (joint == 6) {
    if (targetDeg < JOINT6_MIN_DEG || targetDeg > JOINT6_MAX_DEG) {
      rejectReason = "out_of_range";
    } else if (!g_joint56State.joints[1].haveZero) {
      rejectReason = "not_ready";
    } else {
      ok = sendJoint6AbsoluteDegrees(targetDeg, false);
    }
  }

  Serial.print(ok ? "ok tj" : "err tj");
  Serial.print(joint);
  if (ok) {
    Serial.print(" target=");
    Serial.println(targetDeg, 3);
  } else {
    Serial.print(' ');
    Serial.println(rejectReason != nullptr ? rejectReason : "send_failed");
  }
  return true;
}

bool parseSixFloatArguments(const char *text, float *values) {
  if (text == nullptr || values == nullptr) {
    return false;
  }
  char buffer[SERIAL_LINE_BUFFER_SIZE] = {};
  strncpy(buffer, text, sizeof(buffer) - 1);
  char *savePtr = nullptr;
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    char *token = strtok_r(i == 0 ? buffer : nullptr, " \t", &savePtr);
    if (token == nullptr) {
      return false;
    }
    char *endPtr = nullptr;
    values[i] = strtof(token, &endPtr);
    if (endPtr == token || *endPtr != '\0' || !isfinite(values[i])) {
      return false;
    }
  }
  return strtok_r(nullptr, " \t", &savePtr) == nullptr;
}

void printCoordinatedRampLimits() {
  Serial.print("ramp_limits velocity_dps=");
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(g_coordinatedMaxVelocityDps[i], 3);
  }
  Serial.print(" accel_dps2=");
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(g_coordinatedMaxAccelDps2[i], 3);
  }
  Serial.println();
}

bool handleCoordinatedRampConfigCommand(const char *line) {
  if (line == nullptr) {
    return false;
  }

  bool isVelocity = false;
  bool isAccel = false;
  const char *argText = nullptr;
  if (strncmp(line, "rv6 ", 4) == 0) {
    isVelocity = true;
    argText = line + 4;
  } else if (strncmp(line, "ra6 ", 4) == 0) {
    isAccel = true;
    argText = line + 4;
  } else if (strcmp(line, "rl6") == 0) {
    printCoordinatedRampLimits();
    return true;
  } else {
    return false;
  }

  float values[COORDINATED_JOINT_COUNT] = {};
  if (!parseSixFloatArguments(argText, values)) {
    Serial.println(isVelocity
        ? "err rv6 invalid_numbers; usage: rv6 <j1> <j2> <j3> <j4> <j5> <j6>"
        : "err ra6 invalid_numbers; usage: ra6 <j1> <j2> <j3> <j4> <j5> <j6>");
    return true;
  }
  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    if (values[i] <= 0.0f) {
      Serial.println(isVelocity ? "err rv6 non_positive" : "err ra6 non_positive");
      return true;
    }
  }

  for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
    if (isVelocity) {
      g_coordinatedMaxVelocityDps[i] = values[i];
    } else if (isAccel) {
      g_coordinatedMaxAccelDps2[i] = values[i];
    }
  }
  Serial.println(isVelocity ? "ok rv6" : "ok ra6");
  printCoordinatedRampLimits();
  return true;
}

// -----------------------------
// UI / commands
// -----------------------------
void printConfig() {
  const NodeRuntimeState &state = selectedNodeState();
  const unsigned long now = millis();
  Serial.print("config node=0x");
  printHexByte(g_activeNodeId);
  Serial.print(" bitrate=");
  Serial.print(bitrateName(g_bitrate));
  Serial.print(" oscillator=");
  Serial.print(oscillatorName(g_oscillator));
  Serial.print(" mode=");
  Serial.print(modeName(g_mode));
  Serial.print(" show_all_std=");
  Serial.print(g_showAllStandardFrames ? "on" : "off");
  Serial.print(" telemetry=");
  Serial.print(g_printEncoderTelemetry ? "on" : "off");
  Serial.print(" have_zero=");
  Serial.print(state.haveZeroReference ? "yes" : "no");
  Serial.print(" zero_turns=");
  Serial.print(state.zeroReferenceTurns, 6);
  Serial.print(" gear_ratio=");
  Serial.print(nodeGearRatio(g_activeNodeId), 1);
  Serial.print(" direction=");
  Serial.print(nodeDirection(g_activeNodeId), 1);
  Serial.print(" joint_limits=[");
  Serial.print(nodeMinJointDegrees(g_activeNodeId), 1);
  Serial.print(", ");
  Serial.print(nodeMaxJointDegrees(g_activeNodeId), 1);
  Serial.print("]");
  Serial.print(" stream=");
  Serial.print(g_streamLastTarget ? "on" : "off");
  Serial.print(" heartbeat=");
  Serial.print(heartbeatStateName(state, now));
  Serial.print(" hb_age_ms=");
  if (state.haveHeartbeat) {
    Serial.print(heartbeatAgeMs(state, now));
  } else {
    Serial.print("n/a");
  }
  Serial.print(" have_target=");
  Serial.print(state.haveActiveTarget ? "yes" : "no");
  Serial.print(" have_encoder=");
  Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
  Serial.print(" failsafe=");
  Serial.println(state.failsafeLatched ? "yes" : "no");
  printCoordinatedRampLimits();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h or ? : help");
  Serial.println("  p      : print config");
  Serial.println("  1      : select node 0x10");
  Serial.println("  2      : select node 0x11");
  Serial.println("  3      : select node 0x12");
  Serial.println("  b      : retry automatic bringup for selected node");
  Serial.println("  a      : toggle raw RX debug");
  Serial.println("  v      : toggle throttled encoder telemetry");
  Serial.println("  s      : send self-test frame 123#DEADBEEF");
  Serial.println("  c      : request and confirm CLOSED_LOOP_CONTROL");
  Serial.println("  x      : request IDLE");
  Serial.println("  e      : RTR Get_Encoder_Estimates");
  Serial.println("  r      : RTR Get_Error");
  Serial.println("  z      : capture current encoder position as zero");
  Serial.println("  o      : print motion state");
  Serial.println("  u      : print boot configuration summary");
  Serial.println("  f      : print failsafe-aware status for all nodes");
  Serial.println("  g <n>  : send relative joint angle in degrees from zero");
  Serial.println("  j a b c: set joints 1-3 together in degrees (MoveIt bridge)");
  Serial.println("  j4 a b c d: set joints 1-4 together; joint4 is relative to ZE300 software zero");
  Serial.println("  j6 a b c d e f: set joints 1-6 together (MoveIt bridge)");
  Serial.println("  init6  : explicitly initialize all motors and capture relative zeros");
  Serial.println("  status6: report firmware/CAN/motor/zero readiness");
  Serial.println("  hold6  : stop coordinated trajectory updates and retain last driver targets");
  Serial.println("  disarm6: disable/stop all reachable motors; re-init required");
  Serial.println("  cfs6 on|off: strict CAN fault thresholds for bench validation");
  Serial.println("  testfault6 can_bus_lost: inject fault (requires cfs6 on)");
  Serial.println("  qjs6   : query real measured joint snapshot; replies js6 or js6err");
  Serial.println("  rl6    : print coordinated firmware ramp limits");
  Serial.println("  rv6 a b c d e f: set coordinated max velocity deg/s");
  Serial.println("  ra6 a b c d e f: set coordinated max acceleration deg/s^2");
  Serial.println("  tj4 <deg>, tj5 <deg>, tj6 <deg>: test one aux joint only");
  Serial.println("  jx/jx6 : disable/loosen joints 1-6; re-init required (not hold)");
  Serial.println("  t <n>  : send relative motor turns from zero (debug)");
  Serial.println("  k      : toggle 20 Hz resend of last Set_Input_Pos");
  Serial.println("  w      : normal one-shot mode");
  Serial.println("  5      : bitrate = 500 kbps");
  Serial.println("  m      : bitrate = 1 Mbps");
  Serial.println("  n      : normal mode");
  Serial.println("  i      : listen-only mode");
  Serial.println("  l      : loopback mode");
  Serial.println("  8      : oscillator = 8 MHz");
  Serial.println("  6      : oscillator = 16 MHz");
  Serial.println();
}

bool waitForEncoderEstimate(uint32_t timeoutMs) {
  NodeRuntimeState &state = selectedNodeState();

  if (state.haveLatestEncoderEstimate && millis() - state.lastEncoderEstimateMs <= ENCODER_WAIT_TIMEOUT_MS) {
    return true;
  }

  sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates", true);
  const unsigned long startMs = millis();
  unsigned long lastRetryMs = startMs;

  while (millis() - startMs < timeoutMs) {
    processIncomingFrames();
    serviceTargetStreaming();

    if (state.haveLatestEncoderEstimate && state.lastEncoderEstimateMs >= startMs) {
      return true;
    }

    if (millis() - lastRetryMs >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      lastRetryMs = millis();
      sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, nullptr, false);
    }

    delay(1);
  }

  Serial.println("Timed out waiting for encoder estimate.");
  return false;
}

bool requestClosedLoopAndConfirm(uint32_t timeoutMs) {
  NodeRuntimeState &state = selectedNodeState();
  if (!sendAxisStateRequest(AXIS_STATE_CLOSED_LOOP_CONTROL, "Requested CLOSED_LOOP_CONTROL", true)) {
    Serial.println("Initial CLOSED_LOOP_CONTROL send failed; will retry in confirmation window.");
  }

  // startMs and lastRetryMs are set AFTER the initial send (and any recoverFromTxFailure
  // inside it) so the retry interval is measured from when the attempt actually finished,
  // not from before it started.  This prevents the retry from firing immediately when
  // configureCan() inside the initial send consumed more than CLOSED_LOOP_RETRY_INTERVAL_MS.
  const unsigned long startMs = millis();
  unsigned long lastRetryMs = millis();

  while (millis() - startMs < timeoutMs) {
    processIncomingFrames();
    serviceTargetStreaming();

    if (state.haveHeartbeat &&
        state.lastHeartbeatMs >= startMs &&
        state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
        state.lastHeartbeatActiveErrors == 0) {
      Serial.println("Closed loop confirmed.");
      return true;
    }

    if (millis() - lastRetryMs >= CLOSED_LOOP_RETRY_INTERVAL_MS) {
      const uint8_t eflg = canController.getErrorFlags();
      if (eflg & (MCP2515::EFLG_TXBO | MCP2515::EFLG_TXEP)) {
        Serial.print("Retry skipped: bus error EFLG=0x");
        Serial.println(eflg, HEX);
      } else {
        sendAxisStateRequest(AXIS_STATE_CLOSED_LOOP_CONTROL, nullptr, false);
      }
      // Reset the timer AFTER the attempt so that configureCan() time inside
      // sendAxisStateRequest is counted toward the next retry interval.
      lastRetryMs = millis();
    }

    delay(1);
  }

  Serial.println("Closed loop not confirmed.");
  Serial.println("No fresh CLOSED_LOOP heartbeat was seen; the state command may still have reached the ODrive.");
  sendRemoteRequest(CMD_GET_ERROR, 8, "Requested Get_Error after closed-loop timeout", true);

  const unsigned long errorWaitStartMs = millis();
  while (millis() - errorWaitStartMs < 150) {
    processIncomingFrames();
    delay(1);
  }

  printCachedAxisStatus("Axis status:");

  // Accept state confirmation via Get_Error / heartbeat received during the wait above.
  // On a noisy bus the heartbeat confirmation loop above can miss every heartbeat yet the
  // ODrive IS in closed loop — the post-timeout Get_Error (or a heartbeat that arrived
  // during the 150 ms window) will have updated lastAxisState.
  if (state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
      state.lastHeartbeatActiveErrors == 0) {
    Serial.println("Closed-loop confirmed via post-timeout state check.");
    return true;
  }
  return false;
}

bool runSelectedNodeBringupAttempt() {
  NodeRuntimeState &state = selectedNodeState();
  const bool previousHaveZeroReference = state.haveZeroReference;
  const bool previousHaveActiveTarget = state.haveActiveTarget;
  const float previousZeroReferenceTurns = state.zeroReferenceTurns;
  const float previousLastRelativeCommandTurns = state.lastRelativeCommandTurns;
  const float previousLastRelativeCommandDegrees = state.lastRelativeCommandDegrees;
  const float previousLastAbsoluteCommandTurns = state.lastAbsoluteCommandTurns;

  state.haveZeroReference = false;
  state.haveActiveTarget = false;
  state.lastRelativeCommandTurns = 0.0f;
  state.lastRelativeCommandDegrees = 0.0f;

  if (!requestClosedLoopAndConfirm(CLOSED_LOOP_CONFIRM_TIMEOUT_MS)) {
    if (previousHaveZeroReference && !nodeHasActiveFault(state)) {
      restoreMotionCache(state,
                         previousHaveZeroReference,
                         previousHaveActiveTarget,
                         previousZeroReferenceTurns,
                         previousLastRelativeCommandTurns,
                         previousLastRelativeCommandDegrees,
                         previousLastAbsoluteCommandTurns);
      Serial.println("Restored previous zero/target cache after unconfirmed closed-loop request.");
    }
    return false;
  }

  if (!waitForEncoderEstimate(ENCODER_WAIT_TIMEOUT_MS)) {
    if (previousHaveZeroReference && !nodeHasActiveFault(state)) {
      restoreMotionCache(state,
                         previousHaveZeroReference,
                         previousHaveActiveTarget,
                         previousZeroReferenceTurns,
                         previousLastRelativeCommandTurns,
                         previousLastRelativeCommandDegrees,
                         previousLastAbsoluteCommandTurns);
      Serial.println("Restored previous zero/target cache after encoder wait timeout.");
    }
    return false;
  }

  captureZeroFromLatestPosition();
  return nodeIsReadyForMotion(state);
}

bool runSelectedNodeBringupWithRetries(const char *reason) {
  if (!modeAllowsTransmit(g_mode)) {
    g_mode = CanMode::NormalOneShot;
    if (!configureCan()) {
      Serial.println("Bringup failed: could not switch MCP2515 to a transmit-capable mode.");
      return false;
    }
  }

  Serial.print("Auto bringup node=0x");
  printHexByte(g_activeNodeId);
  if (reason != nullptr && reason[0] != '\0') {
    Serial.print(" trigger=");
    Serial.print(reason);
  }
  Serial.println();

  for (uint8_t attempt = 1; attempt <= BRINGUP_MAX_ATTEMPTS; ++attempt) {
    Serial.print("Bringup attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(BRINGUP_MAX_ATTEMPTS);

    if (runSelectedNodeBringupAttempt()) {
      printMotionState();
      Serial.println("Bringup complete.");
      return true;
    }

    if (attempt < BRINGUP_MAX_ATTEMPTS) {
      Serial.println("Bringup retry will continue without forcing IDLE, to avoid dropping a motor that may already be in closed loop.");
    }
    delay(BRINGUP_RETRY_SETTLE_MS);
  }

  Serial.print("Bringup failed after ");
  Serial.print(BRINGUP_MAX_ATTEMPTS);
  Serial.println(" attempts.");
  printCachedAxisStatus("Axis status:");
  return false;
}

bool ensureActiveNodeReadyForMotion(const char *reason) {
  if (modeAllowsTransmit(g_mode) && nodeIsReadyForMotion(selectedNodeState())) {
    return true;
  }
  return runSelectedNodeBringupWithRetries(reason);
}

void runBringup() {
  (void)ensureActiveNodeReadyForMotion("manual");
}

void runAutomaticBringupAllNodes() {
  const uint8_t previousActiveNodeId = g_activeNodeId;

  Serial.println("Automatic bringup starting for nodes 0x10, 0x11, 0x12");
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    g_activeNodeId = g_nodeStates[i].nodeId;
    (void)ensureActiveNodeReadyForMotion("startup");
  }

  g_activeNodeId = previousActiveNodeId;
  Serial.print("Automatic bringup finished. Active node restored to 0x");
  printHexByte(g_activeNodeId);
  Serial.println();

  if (allNodesHaveBootConfiguration()) {
    g_backgroundBringupEnabled = false;
    printBootConfigurationSummary("All supported nodes configured at boot:");
  } else {
    printBootConfigurationSummary("Boot bringup incomplete; background retries will continue:");
  }
}

void serviceBackgroundBringup() {
  if (!g_backgroundBringupEnabled || !modeAllowsTransmit(g_mode)) {
    return;
  }

  if (allNodesHaveBootConfiguration()) {
    g_backgroundBringupEnabled = false;
    printBootConfigurationSummary("All supported nodes are now configured for position commands:");
    return;
  }

  const unsigned long now = millis();
  if (Serial.available() > 0 ||
      g_serialLineLen > 0 ||
      now - g_lastSerialActivityMs < BACKGROUND_BRINGUP_QUIET_AFTER_INPUT_MS ||
      now - g_lastBackgroundBringupMs < BACKGROUND_BRINGUP_RETRY_INTERVAL_MS) {
    return;
  }

  for (size_t attempt = 0; attempt < SUPPORTED_NODE_COUNT; ++attempt) {
    const size_t index = (g_nextBackgroundBringupIndex + attempt) % SUPPORTED_NODE_COUNT;
    if (nodeHasBootConfiguration(g_nodeStates[index])) {
      continue;
    }

    const uint8_t targetNodeId = g_nodeStates[index].nodeId;
    const uint8_t previousActiveNodeId = g_activeNodeId;
    g_activeNodeId = targetNodeId;
    g_lastBackgroundBringupMs = now;
    g_nextBackgroundBringupIndex = (index + 1) % SUPPORTED_NODE_COUNT;

    Serial.print("Background bringup retry for node 0x");
    printHexByte(targetNodeId);
    Serial.println();
    const bool ready = ensureActiveNodeReadyForMotion("background");
    g_activeNodeId = previousActiveNodeId;

    if (ready) {
      Serial.print("Background bringup captured zero for node 0x");
      printHexByte(targetNodeId);
      Serial.println();
    }

    if (allNodesHaveBootConfiguration()) {
      g_backgroundBringupEnabled = false;
      printBootConfigurationSummary("All supported nodes are now configured for position commands:");
    }
    return;
  }
}

void serviceSafetyFailsafes() {
  // ── Ginkgo-compatible mode ───────────────────────────────────────────
  // The legacy Ginkgo bridge streamed Set_Input_Pos blindly at 20 Hz with
  // zero heartbeat / failsafe logic and never had motor disarms.  The
  // previous ESP32 failsafe caused the very disarms it tried to prevent:
  //   missed heartbeat → stop commanding → ODrive watchdog fires → IDLE
  //   → arm drops under gravity.
  //
  // Now: we LOG diagnostics but NEVER stop commanding.  The ODrives have
  // their own internal safety (current limits, velocity limits, encoder
  // fault detection, watchdog).  If an ODrive disarms itself it ignores
  // Set_Input_Pos harmlessly; when it returns to closed-loop the next
  // streamed position takes effect immediately with no command gap.
  const unsigned long now = millis();

  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    NodeRuntimeState &state = g_nodeStates[i];

    // ── Real ODrive fault (error registers non-zero) ─────────────────
    // Log it loudly but do NOT latch failsafe or stop streaming.
    if (nodeHasActiveFault(state)) {
      Serial.print("WARNING: ODrive fault active node=0x");
      printHexByte(state.nodeId);
      Serial.print(" active_errors=0x");
      Serial.print(state.lastErrorActiveErrors, HEX);
      Serial.print(" disarm_reason=0x");
      Serial.println(state.lastDisarmReason, HEX);
      // Keep streaming — ODrive ignores Set_Input_Pos when not in closed loop.
      continue;
    }

    if (!state.haveActiveTarget && !state.haveZeroReference) {
      continue;
    }

    // ── Axis left closed loop ────────────────────────────────────────
    // The ODrive decided to disarm (current limit, spinout, etc.).
    // Log a warning; keep streaming so the motor recovers seamlessly if
    // it returns to closed-loop (e.g. after a transient current spike).
    if (state.haveHeartbeat &&
        !nodeHeartbeatIsHardTimedOut(state, now) &&
        (state.lastAxisState != AXIS_STATE_CLOSED_LOOP_CONTROL ||
         state.lastHeartbeatActiveErrors != 0)) {
      Serial.print("WARNING: axis not in closed loop node=0x");
      printHexByte(state.nodeId);
      Serial.print(" axis=");
      Serial.print(axisStateName(state.lastAxisState));
      Serial.print(" errors=0x");
      Serial.println(state.lastHeartbeatActiveErrors, HEX);
      // Do NOT engage failsafe — keep streaming.
      continue;
    }

    // ── Heartbeat degraded / missing ─────────────────────────────────
    // CAN heartbeat lost — bus noise, arbitration starvation, etc.
    // Diagnostic only; keep streaming positions.
    if (nodeHeartbeatIsDegraded(state, now)) {
      noteHeartbeatWarning(state, now);
      continue;
    }

    clearHeartbeatWarning(state, true);
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'h':
    case '?':
      printHelp();
      break;

    case 'p':
      printConfig();
      break;

    case '1':
      selectActiveNode(NODE_ID_10);
      break;

    case '2':
      selectActiveNode(NODE_ID_11);
      break;

    case '3':
      selectActiveNode(NODE_ID_12);
      break;

    case 'b':
      runBringup();
      break;

    case 'a':
      g_showAllStandardFrames = !g_showAllStandardFrames;
      Serial.print("raw_rx_debug = ");
      Serial.println(g_showAllStandardFrames ? "ON" : "OFF");
      break;

    case 'v':
      g_printEncoderTelemetry = !g_printEncoderTelemetry;
      selectedNodeState().lastEncoderTelemetryPrintMs = 0;
      Serial.print("encoder_telemetry = ");
      Serial.println(g_printEncoderTelemetry ? "ON" : "OFF");
      break;

    case 's':
      sendSelfTestFrame();
      break;

    case 'c':
      requestClosedLoopAndConfirm(CLOSED_LOOP_CONFIRM_TIMEOUT_MS);
      break;

    case 'x':
      sendAxisStateRequest(AXIS_STATE_IDLE);
      break;

    case 'e':
      sendRemoteRequest(CMD_GET_ENCODER_ESTIMATES, 8, "Requested Get_Encoder_Estimates");
      break;

    case 'r':
      sendRemoteRequest(CMD_GET_ERROR, 8, "Requested Get_Error");
      break;

    case 'z':
      captureZeroFromLatestPosition();
      break;

    case 'o':
      printMotionState();
      break;

    case 'u':
      printBootConfigurationSummary("Boot configuration status:");
      break;

    case 'f':
      printBootConfigurationSummary("Failsafe / boot status:");
      break;

    case 'k':
      g_streamLastTarget = !g_streamLastTarget;
      Serial.print("stream_last_target = ");
      Serial.println(g_streamLastTarget ? "ON" : "OFF");
      break;

    case 'w':
      g_mode = CanMode::NormalOneShot;
      configureCan();
      printConfig();
      break;

    case '5':
      g_bitrate = Bitrate::Kbps500;
      configureCan();
      printConfig();
      break;

    case 'm':
      g_bitrate = Bitrate::Kbps1000;
      configureCan();
      printConfig();
      break;

    case 'n':
      g_mode = CanMode::Normal;
      configureCan();
      printConfig();
      break;

    case 'i':
      g_mode = CanMode::ListenOnly;
      configureCan();
      printConfig();
      break;

    case 'l':
      g_mode = CanMode::Loopback;
      configureCan();
      printConfig();
      break;

    case '8':
      g_oscillator = Oscillator::MHz8;
      configureCan();
      printConfig();
      break;

    case '6':
      g_oscillator = Oscillator::MHz16;
      configureCan();
      printConfig();
      break;

    case '\r':
    case '\n':
    case ' ':
    case '\t':
      break;

    default:
      Serial.print("Unknown command: ");
      Serial.println(c);
      printHelp();
      break;
  }
}

void handleLineCommand(const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }

  if (handleLifecycleCommand(line)) {
    return;
  }

  if (handleMoveItBridgeCommand(line)) {
    return;
  }

  if (handleMoveItIdleCommand(line)) {
    return;
  }

  if (handleJointSnapshotRequest(line)) {
    return;
  }

  if (handleCoordinatedRampConfigCommand(line)) {
    return;
  }

  if (handleAuxJointTestCommand(line)) {
    return;
  }

  const bool isTurnCommand = line[0] == 't';
  const bool isDegreeShortCommand = line[0] == 'g';
  const bool isDegreeWordCommand =
      strncmp(line, "deg", 3) == 0 &&
      (line[3] == ' ' || line[3] == '\t' || line[3] == '\0');

  if (isTurnCommand || isDegreeShortCommand || isDegreeWordCommand) {
    const char *valueText = line + (isDegreeWordCommand ? 3 : 1);
    while (*valueText == ' ' || *valueText == '\t') {
      ++valueText;
    }

    if (*valueText == '\0') {
      if (isTurnCommand) {
        Serial.println("Usage: t <turns>");
      } else {
        Serial.println("Usage: g <degrees>");
      }
      return;
    }

    char *endPtr = nullptr;
    const float value = strtof(valueText, &endPtr);
    if (endPtr == valueText) {
      if (isTurnCommand) {
        Serial.println("Invalid turns value. Example: t 0.25");
      } else {
        Serial.println("Invalid degree value. Example: g 45");
      }
      return;
    }

    while (*endPtr == ' ' || *endPtr == '\t') {
      ++endPtr;
    }
    if (*endPtr != '\0') {
      Serial.println("Unexpected extra text after numeric value.");
      return;
    }

    if (isTurnCommand) {
      sendRelativePositionTurns(value);
    } else {
      sendRelativePositionDegrees(value);
    }
    return;
  }

  if (line[1] == '\0') {
    handleCommand(line[0]);
    return;
  }

  Serial.print("Unknown line command: ");
  Serial.println(line);
  printHelp();
}

void serviceTargetStreaming() {
  if (!g_streamLastTarget || !modeAllowsTransmit(g_mode) ||
      !g_motorsInitialized || g_firmwareState != FirmwareState::MotorsReady) {
    return;
  }

  const unsigned long now = millis();
  if (g_coordinatedSextetTarget.haveTarget) {
    if (now - g_coordinatedSextetTarget.lastStreamMs < TARGET_STREAM_INTERVAL_MS) {
      return;
    }

    updateCoordinatedRampedCommand(now);
    const SextetSendResult result = sendCoordinatedSextetBurst(
        g_coordinatedSextetTarget.commandedDeg[0],
        g_coordinatedSextetTarget.commandedDeg[1],
        g_coordinatedSextetTarget.commandedDeg[2],
        g_coordinatedSextetTarget.commandedDeg[3],
        g_coordinatedSextetTarget.commandedDeg[4],
        g_coordinatedSextetTarget.commandedDeg[5],
        g_nextCoordinatedSextetAuxFirst);
    g_coordinatedSextetTarget.lastStreamMs = now;
    if (sextetSendSucceeded(result)) {
      g_motorCommandFailCount = 0;
      g_motorFaultWindowStartMs = 0;
      g_nextCoordinatedSextetAuxFirst = !g_nextCoordinatedSextetAuxFirst;
      if (now - g_coordinatedSextetTarget.lastDebugLogMs >= RAMP_DEBUG_LOG_INTERVAL_MS) {
        g_coordinatedSextetTarget.lastDebugLogMs = now;
        Serial.print("ramped command sent to motor drivers epoch=");
        Serial.print(g_coordinatedSextetTarget.epoch);
        Serial.print(": joint1=");
        Serial.print(g_coordinatedSextetTarget.commandedDeg[0], 3);
        Serial.print(" joint2=");
        Serial.print(g_coordinatedSextetTarget.commandedDeg[1], 3);
        Serial.print(" joint3=");
        Serial.print(g_coordinatedSextetTarget.commandedDeg[2], 3);
        Serial.print(" joint4=");
        Serial.print(g_coordinatedSextetTarget.commandedDeg[3], 3);
        Serial.print(" joint5=");
        Serial.print(g_coordinatedSextetTarget.commandedDeg[4], 3);
        Serial.print(" joint6=");
        Serial.println(g_coordinatedSextetTarget.commandedDeg[5], 3);
      }
    } else {
      ++g_canIncompleteBurstCount;
      const bool jointOk[COORDINATED_JOINT_COUNT] = {
          result.ok1, result.ok2, result.ok3, result.ok4, result.ok5, result.ok6};
      const uint32_t jointCanIds[COORDINATED_JOINT_COUNT] = {
          makeCanId(NODE_ID_10, CMD_SET_INPUT_POS),
          makeCanId(NODE_ID_11, CMD_SET_INPUT_POS),
          makeCanId(NODE_ID_12, CMD_SET_INPUT_POS),
          ze300_joint4::requestId(),
          lktech_joint56::kJointConfigs[0].canId,
          lktech_joint56::kJointConfigs[1].canId,
      };
      for (size_t i = 0; i < COORDINATED_JOINT_COUNT; ++i) {
        if (jointOk[i]) continue;
        ++g_jointCommandFailCount[i];
        // ODrive failures were already recorded with the exact library code
        // by sendFrame(). Aux helpers currently return bool, so preserve an
        // explicit synthetic code instead of guessing the library result.
        if (i >= 3) {
          g_canLastFailedId = jointCanIds[i];
          g_canLastFailedCode = -1;
          noteCanSendResult(false, "aux_coordinated_send");
        }
      }
      printSextetSendResult("WARNING: coordinated sextet stream send incomplete; ", result);
      noteMotorCommandResult(false, "coordinated_sextet");
    }
    return;
  }

  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    NodeRuntimeState &state = g_nodeStates[i];
    if (!state.haveActiveTarget) {
      continue;
    }
    if (now - state.lastTargetStreamMs < TARGET_STREAM_INTERVAL_MS) {
      continue;
    }

    state.lastTargetStreamMs = now;
    sendInputPositionToNode(state.nodeId, state.lastAbsoluteCommandTurns, 0, 0, nullptr, false);
  }

  if (g_auxTargets.joint4HaveTarget && now - g_auxTargets.joint4LastTxMs >= TARGET_STREAM_INTERVAL_MS) {
    const bool ok = ze300_joint4::sendRelativeDegrees(
        canController, g_joint4State, g_auxTargets.joint4Deg);
    noteMotorCommandResult(ok, "joint4");
    if (ok) {
      g_auxTargets.joint4LastTxMs = now;
    } else {
      ++g_jointCommandFailCount[3];
      g_canLastFailedId = ze300_joint4::requestId();
      g_canLastFailedCode = -1;
      noteCanSendResult(false, "joint4");
    }
  }

  if (g_auxTargets.joint5HaveTarget && now - g_auxTargets.joint5LastTxMs >= TARGET_STREAM_INTERVAL_MS) {
    const bool ok = lktech_joint56::sendOutputDegrees(
        canController, g_joint56State, 0, g_auxTargets.joint5Deg);
    noteMotorCommandResult(ok, "joint5");
    if (ok) {
      g_auxTargets.joint5LastTxMs = now;
    } else {
      ++g_jointCommandFailCount[4];
      g_canLastFailedId = lktech_joint56::kJointConfigs[0].canId;
      g_canLastFailedCode = -1;
      noteCanSendResult(false, "joint5");
    }
  }

  if (g_auxTargets.joint6HaveTarget && now - g_auxTargets.joint6LastTxMs >= TARGET_STREAM_INTERVAL_MS) {
    const bool ok = lktech_joint56::sendOutputDegrees(
        canController, g_joint56State, 1, g_auxTargets.joint6Deg);
    noteMotorCommandResult(ok, "joint6");
    if (ok) {
      g_auxTargets.joint6LastTxMs = now;
    } else {
      ++g_jointCommandFailCount[5];
      g_canLastFailedId = lktech_joint56::kJointConfigs[1].canId;
      g_canLastFailedCode = -1;
      noteCanSendResult(false, "joint6");
    }
  }
}

void serviceCanHealth() {
  if (!g_motorsInitialized || g_firmwareState != FirmwareState::MotorsReady) {
    return;
  }
  const uint8_t eflg = canController.getErrorFlags();
  if (eflg & MCP2515::EFLG_TXBO) {
    enterCanFault("mcp2515_bus_off");
    return;
  }
  if (eflg & MCP2515::EFLG_TXEP) {
    enterCanFault("mcp2515_error_passive");
    return;
  }
  const unsigned long now = millis();
  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    const NodeRuntimeState &state = g_nodeStates[i];
    if (!state.haveHeartbeat || now - state.lastHeartbeatMs > MOTOR_HEARTBEAT_TIMEOUT_MS) {
      char reason[96] = {};
      snprintf(reason, sizeof(reason), "motor_heartbeat_timeout joint=%u",
               static_cast<unsigned int>(i + 1));
      enterCanFault(reason);
      return;
    }
  }
}

// -----------------------------
// RX processing
// -----------------------------
void processIncomingFrames() {
  uint16_t frameCount = 0;

  while (frameCount < MAX_RX_FRAMES_PER_PASS) {
    if (canController.readMessage(&rxFrame) != MCP2515::ERROR_OK) {
      break;
    }

    decodeFrame(rxFrame);
    ++frameCount;
  }
}

} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  g_lastSerialActivityMs = millis();
  initializeRuntimeStates();
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

  Serial.println("ESP32 + MCP2515 MoveIt controller (6-DOF)");
  Serial.println("Supported joints: 0x10, 0x11, 0x12 (ODrive) + joint4 ZE300 + joint5/joint6 LKTech");
  Serial.print("Default active node: 0x12, bitrate: ");
  Serial.print(bitrateName(g_bitrate));
  Serial.print(", mode: ");
  Serial.print(modeName(g_mode));
  Serial.println(", serial: 115200");

  if (!configureCan()) {
    Serial.println("Initial CAN configuration failed");
  }

  // Passive boot: configure communication only. Motor closed-loop requests,
  // zero capture and hold commands are performed exclusively by explicit init6.
  g_auxFiltersEnabled = true;
  if (!configureCan()) {
    Serial.println("WARNING: failed to reconfigure MCP2515 with full RX filters; aux motor replies may be lost");
  }

  ze300_joint4::reset(g_joint4State);
  lktech_joint56::reset(g_joint56State);
  clearCoordinatedSextetTarget();
  Serial.println("boot6 ready=true motors_initialized=false passive=true");
  Serial.println("Motors are uninitialized; send init6 explicitly before motion");

  printConfig();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    g_lastSerialActivityMs = millis();

    if (c == '\r' || c == '\n') {
      if (g_serialLineLen > 0) {
        g_serialLine[g_serialLineLen] = '\0';
        handleLineCommand(g_serialLine);
        g_serialLineLen = 0;
      }
      continue;
    }

    const bool beginsLineCommand =
        (c == 't' || c == 'g' || c == 'd' || c == 'j' || c == 'q' ||
         c == 'i' || c == 's' || c == 'h' || c == 'c');
    if (g_serialLineLen == 0 && !beginsLineCommand) {
      switch (c) {
        case 'h':
        case '?':
        case 'p':
        case '1':
        case '2':
        case '3':
        case 'b':
        case 'a':
        case 'v':
        case 's':
        case 'c':
        case 'x':
        case 'e':
        case 'r':
        case 'z':
        case 'o':
        case 'u':
        case 'f':
        case 'k':
        case 'w':
        case '5':
        case 'm':
        case 'n':
        case 'i':
        case 'l':
        case '8':
        case '6':
          handleCommand(c);
          continue;
        case ' ':
        case '\t':
          continue;
        default:
          break;
      }
    }

    if (g_serialLineLen + 1 < SERIAL_LINE_BUFFER_SIZE) {
      g_serialLine[g_serialLineLen++] = c;
    } else {
      Serial.println("Command too long, clearing input buffer.");
      g_serialLineLen = 0;
    }
  }

  processIncomingFrames();
  serviceSafetyFailsafes();
  serviceCanHealth();
  serviceTargetStreaming();
  serviceBackgroundBringup();
}
