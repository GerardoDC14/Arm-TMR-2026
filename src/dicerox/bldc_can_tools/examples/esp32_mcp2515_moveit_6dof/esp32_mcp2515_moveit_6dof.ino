#include <Arduino.h>
#include <SPI.h>
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
constexpr float JOINT5_MIN_DEG = -90.0f;
constexpr float JOINT5_MAX_DEG = 90.0f;
constexpr float JOINT6_MIN_DEG = -90.0f;
constexpr float JOINT6_MAX_DEG = 90.0f;

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

MCP2515 canController(PIN_CAN_CS);
struct can_frame rxFrame;

CanMode g_mode = CanMode::Normal;
Bitrate g_bitrate = Bitrate::Kbps500;
Oscillator g_oscillator = Oscillator::MHz8;
bool g_auxFiltersEnabled = false;  // when false, LKTech/ZE300 IDs are excluded from RX filters
bool g_showAllStandardFrames = false;
bool g_printEncoderTelemetry = false;
bool g_streamLastTarget = true;
bool g_backgroundBringupEnabled = true;
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
bool sendJoint4AbsoluteDegrees(float absoluteDegrees, bool printSummary);
bool sendJoint5AbsoluteDegrees(float absoluteDegrees, bool printSummary);
bool sendJoint6AbsoluteDegrees(float absoluteDegrees, bool printSummary);
bool handleMoveItBridgeCommand(const char *line);
bool handleMoveItIdleCommand(const char *line);
bool ensureActiveNodeReadyForMotion(const char *reason);
void runAutomaticBringupAllNodes();
void clearActiveTargetForNode(uint8_t nodeId, const char *reason);
void recoverFromTxFailure(const can_frame &frame, MCP2515::ERROR sendResult);
bool nodeHasBootConfiguration(const NodeRuntimeState &state);
bool allNodesHaveBootConfiguration();
void printBootConfigurationSummary(const char *prefix);
bool nodeHasActiveFault(const NodeRuntimeState &state);
void clearFailsafeLatch(NodeRuntimeState &state);
bool sendAxisStateRequestToNode(uint8_t nodeId, uint32_t state, const char *message, bool logFrame);
void engageNodeFailsafe(uint8_t nodeId, const char *reason, bool requestIdleIfReachable);

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
         state.haveHeartbeat &&
         state.lastAxisState == AXIS_STATE_CLOSED_LOOP_CONTROL &&
         state.lastHeartbeatActiveErrors == 0;
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

void printBootConfigurationSummary(const char *prefix) {
  if (prefix != nullptr && prefix[0] != '\0') {
    Serial.println(prefix);
  }

  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    const NodeRuntimeState &state = g_nodeStates[i];
    Serial.print("  node 0x");
    printHexByte(state.nodeId);
    Serial.print(" zero=");
    Serial.print(state.haveZeroReference ? "yes" : "no");
    Serial.print(" encoder=");
    Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
    Serial.print(" heartbeat=");
    Serial.print(state.haveHeartbeat ? "yes" : "no");
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
      millis() - state->lastHeartbeatMs <= HEARTBEAT_STALE_TIMEOUT_MS;
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
    Serial.print("ERROR: send failed code=");
    Serial.print(static_cast<int>(sendResult));
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

  if (message != nullptr) {
    Serial.println(message);
  }

  return true;
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

bool sendJoint4AbsoluteDegrees(float absoluteDegrees, bool printSummary) {
  if (absoluteDegrees < JOINT4_MIN_DEG || absoluteDegrees > JOINT4_MAX_DEG) {
    if (printSummary) {
      Serial.print("Rejected joint4 absolute target ");
      Serial.print(absoluteDegrees, 3);
      Serial.print(" deg outside limits [");
      Serial.print(JOINT4_MIN_DEG, 1);
      Serial.print(", ");
      Serial.print(JOINT4_MAX_DEG, 1);
      Serial.println("]");
    }
    return false;
  }

  if (!g_joint4State.speedConfigured && !ze300_joint4::initialize(canController, g_joint4State)) {
    if (printSummary) {
      Serial.println("joint4 ZE300 speed init failed");
    }
    return false;
  }

  const bool ok = ze300_joint4::sendAbsoluteDegrees(canController, g_joint4State, absoluteDegrees);
  if (ok) {
    g_auxTargets.joint4HaveTarget = true;
    g_auxTargets.joint4Deg = absoluteDegrees;
    g_auxTargets.joint4LastTxMs = millis();
  }
  if (printSummary) {
    Serial.print("Joint4 target=");
    Serial.print(absoluteDegrees, 3);
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
    const NodeRuntimeState *state10 = lookupNodeState(NODE_ID_10);
    const NodeRuntimeState *state11 = lookupNodeState(NODE_ID_11);
    const NodeRuntimeState *state12 = lookupNodeState(NODE_ID_12);
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
  }

  // Drain RX buffers before sending to all joints. ODrive encoder estimates
  // and LKTech/ZE300 replies accumulate during the ~1 ms of sequential SPI
  // writes and overflow RXB0/RXB1 if not consumed first.
  processIncomingFrames();

  const bool ok1 = sendRelativePositionDegreesToNode(NODE_ID_10, joint1Deg, false, false);
  const bool ok2 = sendRelativePositionDegreesToNode(NODE_ID_11, joint2Deg, false, false);
  const bool ok3 = sendRelativePositionDegreesToNode(NODE_ID_12, joint3Deg, false, false);
  const bool ok4 = (is_quartet || is_sextet) ? sendJoint4AbsoluteDegrees(joint4Deg, false) : true;

  if (is_sextet) {
    // The 3 ODrive frames + ZE300 frame may still be occupying all 3 TX
    // buffers at this point. A 2 ms pause ensures TXB0/1/2 have drained
    // (3 × ~110 µs CAN frame time) before the LKTech frames are loaded,
    // preventing ERROR_ALLTXBUSY on the joint5/joint6 sendMessage calls.
    delay(2);
  }

  const bool ok5 = is_sextet ? sendJoint5AbsoluteDegrees(joint5Deg, false) : true;
  const bool ok6 = is_sextet ? sendJoint6AbsoluteDegrees(joint6Deg, false) : true;

  if (is_sextet) {
    // LKTech motors reply to each 0xA4 position command within ~2 ms using
    // the same CAN ID (0x14E / 0x14F). If those replies are still in flight
    // when the next sextet's joint5/6 commands are sent, both sides transmit
    // frames with identical CAN IDs; the data bytes differ, causing a bit
    // error in the data phase (not an arbitration loss) which sets TXERR.
    // A 5 ms wait + drain consumes the replies before the next command cycle.
    delay(5);
    processIncomingFrames();
  }

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
  const bool ze300_ok = ze300_joint4::disable(canController);
  const bool lk_ok = lktech_joint56::stopAll(canController, g_joint56State);
  g_auxTargets.joint4HaveTarget = false;
  g_auxTargets.joint5HaveTarget = false;
  g_auxTargets.joint6HaveTarget = false;
  Serial.print("MoveIt idle applied; joint4=");
  Serial.print(ze300_ok ? "ok" : "fail");
  Serial.print(" joint5-6=");
  Serial.println(lk_ok ? "ok" : "fail");
  return true;
}

void printMotionState() {
  const NodeRuntimeState &state = selectedNodeState();
  const float relativeTurnsFromZero = state.lastEncoderPosTurns - state.zeroReferenceTurns;
  const float relativeJointDegrees = motorTurnsToJointDegrees(g_activeNodeId, relativeTurnsFromZero);
  Serial.print("motion node=0x");
  printHexByte(g_activeNodeId);
  Serial.print(" axis_state=");
  Serial.print(axisStateName(state.lastAxisState));
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

// -----------------------------
// UI / commands
// -----------------------------
void printConfig() {
  const NodeRuntimeState &state = selectedNodeState();
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
  Serial.print(" have_target=");
  Serial.print(state.haveActiveTarget ? "yes" : "no");
  Serial.print(" have_encoder=");
  Serial.print(state.haveLatestEncoderEstimate ? "yes" : "no");
  Serial.print(" failsafe=");
  Serial.println(state.failsafeLatched ? "yes" : "no");
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
  Serial.println("  j4 a b c d: set joints 1-4 together, with joint4 absolute ZE300 degrees");
  Serial.println("  j6 a b c d e f: set joints 1-6 together (MoveIt bridge)");
  Serial.println("  jx/jx6 : idle joints 1-6 together (MoveIt bridge)");
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
  const unsigned long now = millis();

  for (size_t i = 0; i < SUPPORTED_NODE_COUNT; ++i) {
    NodeRuntimeState &state = g_nodeStates[i];

    if (nodeHasActiveFault(state)) {
      engageNodeFailsafe(state.nodeId, "odrive fault active", true);
      continue;
    }

    if (!state.haveActiveTarget && !state.haveZeroReference) {
      continue;
    }

    if (!state.haveHeartbeat || now - state.lastHeartbeatMs > HEARTBEAT_STALE_TIMEOUT_MS) {
      engageNodeFailsafe(state.nodeId, "heartbeat timeout", false);
      continue;
    }

    if (state.lastAxisState != AXIS_STATE_CLOSED_LOOP_CONTROL || state.lastHeartbeatActiveErrors != 0) {
      engageNodeFailsafe(state.nodeId, "axis left closed loop", true);
      continue;
    }
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

  if (handleMoveItBridgeCommand(line)) {
    return;
  }

  if (handleMoveItIdleCommand(line)) {
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
  if (!g_streamLastTarget || !modeAllowsTransmit(g_mode)) {
    return;
  }

  const unsigned long now = millis();
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
    if (ze300_joint4::sendAbsoluteDegrees(canController, g_joint4State, g_auxTargets.joint4Deg)) {
      g_auxTargets.joint4LastTxMs = now;
    }
  }

  if (g_auxTargets.joint5HaveTarget && now - g_auxTargets.joint5LastTxMs >= TARGET_STREAM_INTERVAL_MS) {
    if (lktech_joint56::sendOutputDegrees(canController, g_joint56State, 0, g_auxTargets.joint5Deg)) {
      g_auxTargets.joint5LastTxMs = now;
    }
  }

  if (g_auxTargets.joint6HaveTarget && now - g_auxTargets.joint6LastTxMs >= TARGET_STREAM_INTERVAL_MS) {
    if (lktech_joint56::sendOutputDegrees(canController, g_joint56State, 1, g_auxTargets.joint6Deg)) {
      g_auxTargets.joint6LastTxMs = now;
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

  runAutomaticBringupAllNodes();

  // ODrive bringup is complete. Enable auxiliary RX filters so ZE300 and
  // LKTech reply frames can reach RXB1. Must go through configureCan() — not
  // configureReceiveFilters() directly — because setFilterMask/setFilter
  // internally call setConfigMode() without restoring normal mode afterward.
  // configureCan() ends with applyCanMode() which puts the chip back into
  // normal mode so frames can be received/transmitted again.
  g_auxFiltersEnabled = true;
  if (!configureCan()) {
    Serial.println("WARNING: failed to reconfigure MCP2515 with full RX filters; aux motor replies may be lost");
  }

  if (ze300_joint4::initialize(canController, g_joint4State)) {
    Serial.println("joint4 ZE300 speed configured");
  } else {
    Serial.println("joint4 ZE300 speed configuration failed");
  }

  if (lktech_joint56::initialize(canController, g_joint56State)) {
    Serial.println("joint5/joint6 LKTech zero capture complete");
  } else {
    Serial.println("joint5/joint6 LKTech zero capture failed");
  }

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

    const bool beginsLineCommand = (c == 't' || c == 'g' || c == 'd' || c == 'j');
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
  serviceTargetStreaming();
  serviceBackgroundBringup();
}
