#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

// Minimal ESP32 + MCP2515 controller for the two LKTech joints on the shared
// Dicerox CAN bus.
//
// Design goals:
// - Only control joint5 and joint6.
// - Ignore ODrive and ZE300 traffic on the same CAN bus.
// - Keep the serial interface simple and predictable.
//
// This sketch assumes the same LKTech bus IDs already used elsewhere in this
// repo:
//   joint5 -> 0x14E
//   joint6 -> 0x14F
//
// If your actual LKTech device IDs are hexadecimal 0x14 and 0x15 instead of
// decimal 14 and 15, update kMotors[].can_id to 0x154 and 0x155.

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t SPI_SCK_PIN = 18;
static constexpr uint8_t SPI_MISO_PIN = 19;
static constexpr uint8_t SPI_MOSI_PIN = 23;
static constexpr uint8_t MCP2515_CS_PIN = 5;
static constexpr uint8_t MCP2515_INT_PIN = 4;

static constexpr CAN_SPEED FIXED_CAN_SPEED = CAN_500KBPS;
static constexpr CAN_CLOCK FIXED_CAN_CLOCK = MCP_8MHZ;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;

static constexpr uint8_t CMD_MOTOR_OFF = 0x80;
static constexpr uint8_t CMD_MOTOR_STOP = 0x81;
static constexpr uint8_t CMD_MOTOR_ON = 0x88;
static constexpr uint8_t CMD_READ_MULTI_LOOP_ANGLE = 0x92;
static constexpr uint8_t CMD_MULTI_LOOP_CONTROL_2 = 0xA4;

struct LKMotorConfig {
  const char *name;
  const char *short_name;
  uint16_t can_id;
  double gear_ratio;
  double default_output_speed_dps;
};

static constexpr LKMotorConfig kMotors[] = {
    {"joint5", "j5", 0x14E, 10.0, 60.0},
    {"joint6", "j6", 0x14F, 10.0, 60.0},
};

static constexpr size_t kMotorCount = sizeof(kMotors) / sizeof(kMotors[0]);

struct MotorRuntimeState {
  bool enabled;
  bool have_zero;
  double zero_motor_deg;
  double last_motor_deg;
  double output_speed_dps;
};

MCP2515 g_mcp2515(MCP2515_CS_PIN);
MotorRuntimeState g_motor_states[kMotorCount] = {};
size_t g_active_motor_index = 0;
char g_serial_line[160] = {};
size_t g_serial_line_len = 0;

const LKMotorConfig &motorConfig(size_t index) {
  return kMotors[index];
}

MotorRuntimeState &motorState(size_t index) {
  return g_motor_states[index];
}

double motorToOutputDegrees(size_t index, double motor_deg) {
  return (motor_deg - motorState(index).zero_motor_deg) / motorConfig(index).gear_ratio;
}

double outputToMotorDegrees(size_t index, double output_deg) {
  return motorState(index).zero_motor_deg + output_deg * motorConfig(index).gear_ratio;
}

uint16_t outputSpeedToMotorDps(size_t index, double output_dps) {
  const double motor_dps = output_dps * motorConfig(index).gear_ratio;
  if (motor_dps <= 0.0) {
    return 0;
  }
  if (motor_dps >= 65535.0) {
    return 65535;
  }
  return static_cast<uint16_t>(motor_dps + 0.5);
}

int32_t degreesToCentiDegrees(double degrees) {
  const double scaled = degrees * 100.0;
  if (scaled >= 0.0) {
    return static_cast<int32_t>(scaled + 0.5);
  }
  return static_cast<int32_t>(scaled - 0.5);
}

double centiDegreesToDegrees(int64_t centi_degrees) {
  return static_cast<double>(centi_degrees) / 100.0;
}

int64_t readSigned56LE(const uint8_t *data) {
  uint64_t raw = 0;
  for (int i = 0; i < 7; ++i) {
    raw |= static_cast<uint64_t>(data[i]) << (8 * i);
  }
  if (data[6] & 0x80) {
    raw |= 0xFF00000000000000ULL;
  }
  return static_cast<int64_t>(raw);
}

bool equalsIgnoreCase(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) {
    return false;
  }

  while (*left != '\0' && *right != '\0') {
    if (tolower(static_cast<unsigned char>(*left)) !=
        tolower(static_cast<unsigned char>(*right))) {
      return false;
    }
    ++left;
    ++right;
  }

  return *left == '\0' && *right == '\0';
}

void initializeMotorStates() {
  for (size_t index = 0; index < kMotorCount; ++index) {
    MotorRuntimeState &state = motorState(index);
    state.enabled = false;
    state.have_zero = false;
    state.zero_motor_deg = 0.0;
    state.last_motor_deg = 0.0;
    state.output_speed_dps = motorConfig(index).default_output_speed_dps;
  }
}

void printFrame(const can_frame &frame, const char *prefix) {
  Serial.printf("%s id=0x%03lX dlc=%u data=",
                prefix,
                static_cast<unsigned long>(frame.can_id & CAN_SFF_MASK),
                static_cast<unsigned>(frame.can_dlc));
  for (uint8_t i = 0; i < frame.can_dlc; ++i) {
    if (i) {
      Serial.print(" ");
    }
    Serial.printf("%02X", frame.data[i]);
  }
  Serial.println();
}

void printMotorStatus(size_t index) {
  const LKMotorConfig &config = motorConfig(index);
  const MotorRuntimeState &state = motorState(index);

  Serial.printf("%s name=%s can_id=0x%03X speed_out=%.2f dps speed_motor=%u dps enabled=%s zero=%s last_motor_deg=%.2f",
                index == g_active_motor_index ? "*" : " ",
                config.name,
                config.can_id,
                state.output_speed_dps,
                static_cast<unsigned>(outputSpeedToMotorDps(index, state.output_speed_dps)),
                state.enabled ? "yes" : "no",
                state.have_zero ? "yes" : "no",
                state.last_motor_deg);
  if (state.have_zero) {
    Serial.printf(" zero_motor_deg=%.2f last_output_deg=%.2f",
                  state.zero_motor_deg,
                  motorToOutputDegrees(index, state.last_motor_deg));
  }
  Serial.println();
}

void printAllMotorStatus() {
  Serial.println("Configured LKTech motors:");
  for (size_t index = 0; index < kMotorCount; ++index) {
    printMotorStatus(index);
  }
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("     show this help");
  Serial.println("  list");
  Serial.println("     list joint5/joint6 state");
  Serial.println("  use TARGET");
  Serial.println("     TARGET = joint5|j5|14|0x14|0x14E or joint6|j6|15|0x15|0x14F");
  Serial.println("  status [TARGET|all]");
  Serial.println("     print cached state");
  Serial.println("  on [TARGET|all]");
  Serial.println("     motor on + capture current pose as software zero");
  Serial.println("  zero [TARGET|all]");
  Serial.println("     capture current pose as software zero");
  Serial.println("  pos [TARGET|all]");
  Serial.println("     read current multi-loop angle");
  Serial.println("  goto DEG");
  Serial.println("  goto TARGET DEG");
  Serial.println("     move output-side angle relative to software zero");
  Serial.println("  sync DEG5 DEG6");
  Serial.println("     send one output-side target to joint5 and joint6");
  Serial.println("  motor DEG");
  Serial.println("  motor TARGET DEG");
  Serial.println("     send raw motor-side multi-loop angle");
  Serial.println("  speed DPS");
  Serial.println("  speed TARGET DPS");
  Serial.println("     set output-side speed in deg/s");
  Serial.println("  off [TARGET|all]");
  Serial.println("     send 0x80 motor off");
  Serial.println("  stop [TARGET|all]");
  Serial.println("     send 0x81 stop");
}

bool configureController() {
  g_mcp2515.reset();

  const MCP2515::ERROR bitrate_result = g_mcp2515.setBitrate(FIXED_CAN_SPEED, FIXED_CAN_CLOCK);
  if (bitrate_result != MCP2515::ERROR_OK) {
    Serial.printf("setBitrate failed: %d\n", static_cast<int>(bitrate_result));
    return false;
  }

  // Exact hardware receive filtering: only accept the two LKTech reply IDs.
  // This keeps ODrive heartbeats and ZE300 replies from filling the MCP2515 RX
  // buffers while we wait for LKTech replies.
  g_mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  g_mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  g_mcp2515.setFilter(MCP2515::RXF0, false, kMotors[0].can_id);
  g_mcp2515.setFilter(MCP2515::RXF1, false, kMotors[1].can_id);
  g_mcp2515.setFilter(MCP2515::RXF2, false, kMotors[0].can_id);
  g_mcp2515.setFilter(MCP2515::RXF3, false, kMotors[1].can_id);
  g_mcp2515.setFilter(MCP2515::RXF4, false, kMotors[0].can_id);
  g_mcp2515.setFilter(MCP2515::RXF5, false, kMotors[1].can_id);

  const MCP2515::ERROR mode_result = g_mcp2515.setNormalMode();
  if (mode_result != MCP2515::ERROR_OK) {
    Serial.printf("setNormalMode failed: %d\n", static_cast<int>(mode_result));
    return false;
  }

  Serial.println("MCP2515 configured successfully.");
  Serial.printf("RX filters locked to 0x%03X and 0x%03X\n", kMotors[0].can_id, kMotors[1].can_id);
  printAllMotorStatus();
  return true;
}

bool sendFrame(uint16_t arbitration_id, const uint8_t *data, uint8_t dlc = 8) {
  can_frame frame = {};
  frame.can_id = arbitration_id;
  frame.can_dlc = dlc;
  memcpy(frame.data, data, dlc);

  const MCP2515::ERROR result = g_mcp2515.sendMessage(&frame);
  if (result != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed: %d id=0x%03X\n",
                  static_cast<int>(result),
                  arbitration_id);
    return false;
  }

  printFrame(frame, "TX");
  return true;
}

void drainReceiveQueue() {
  can_frame frame = {};
  while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    printFrame(frame, "RX(stale)");
  }
}

bool waitForReply(size_t index, uint8_t expected_command, can_frame *reply_frame) {
  const unsigned long start_ms = millis();
  can_frame frame = {};

  while (millis() - start_ms < REPLY_TIMEOUT_MS) {
    while (g_mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      const uint16_t arbitration_id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
      const bool matching_reply = arbitration_id == motorConfig(index).can_id &&
                                  frame.can_dlc >= 1 &&
                                  frame.data[0] == expected_command;
      printFrame(frame, matching_reply ? "RX" : "RX(other)");
      if (matching_reply) {
        if (reply_frame != nullptr) {
          *reply_frame = frame;
        }
        return true;
      }
    }
    delay(2);
  }

  Serial.printf("timeout waiting for cmd=0x%02X on %s (id=0x%03X)\n",
                static_cast<unsigned>(expected_command),
                motorConfig(index).name,
                motorConfig(index).can_id);
  return false;
}

bool sendSimpleCommandAndWait(size_t index, uint8_t command, can_frame *reply_frame = nullptr) {
  const uint8_t payload[8] = {command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  drainReceiveQueue();
  if (!sendFrame(motorConfig(index).can_id, payload, sizeof(payload))) {
    return false;
  }
  return waitForReply(index, command, reply_frame);
}

bool readCurrentMotorAngle(size_t index, double *motor_angle_deg) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_READ_MULTI_LOOP_ANGLE, &reply)) {
    return false;
  }

  const int64_t angle_cdeg = readSigned56LE(&reply.data[1]);
  MotorRuntimeState &state = motorState(index);
  state.last_motor_deg = centiDegreesToDegrees(angle_cdeg);
  if (motor_angle_deg != nullptr) {
    *motor_angle_deg = state.last_motor_deg;
  }

  Serial.printf("%s current motor multi-loop angle = %.2f deg\n",
                motorConfig(index).name,
                state.last_motor_deg);
  if (state.have_zero) {
    Serial.printf("%s current output angle = %.2f deg (zero_motor_deg=%.2f)\n",
                  motorConfig(index).name,
                  motorToOutputDegrees(index, state.last_motor_deg),
                  state.zero_motor_deg);
  }
  return true;
}

bool captureZeroOffset(size_t index) {
  double motor_angle_deg = 0.0;
  if (!readCurrentMotorAngle(index, &motor_angle_deg)) {
    return false;
  }

  MotorRuntimeState &state = motorState(index);
  state.zero_motor_deg = motor_angle_deg;
  state.have_zero = true;
  Serial.printf("%s software zero captured at motor_deg=%.2f\n",
                motorConfig(index).name,
                state.zero_motor_deg);
  return true;
}

bool sendMotorOnAndCaptureZero(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_ON, &reply)) {
    return false;
  }

  motorState(index).enabled = true;
  Serial.printf("%s motor on acknowledged\n", motorConfig(index).name);
  delay(20);
  return captureZeroOffset(index);
}

bool sendMotorOff(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_OFF, &reply)) {
    return false;
  }

  motorState(index).enabled = false;
  Serial.printf("%s motor off acknowledged\n", motorConfig(index).name);
  return true;
}

bool sendMotorStop(size_t index) {
  can_frame reply = {};
  if (!sendSimpleCommandAndWait(index, CMD_MOTOR_STOP, &reply)) {
    return false;
  }

  Serial.printf("%s motor stop acknowledged\n", motorConfig(index).name);
  return true;
}

bool commandMotorAngleDegrees(size_t index, double target_motor_deg) {
  MotorRuntimeState &state = motorState(index);
  const uint16_t motor_speed_dps = outputSpeedToMotorDps(index, state.output_speed_dps);
  const int32_t target_cdeg = degreesToCentiDegrees(target_motor_deg);

  const uint8_t payload[8] = {
      CMD_MULTI_LOOP_CONTROL_2,
      0x00,
      static_cast<uint8_t>(motor_speed_dps & 0xFF),
      static_cast<uint8_t>((motor_speed_dps >> 8) & 0xFF),
      static_cast<uint8_t>(target_cdeg & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 8) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 16) & 0xFF),
      static_cast<uint8_t>((target_cdeg >> 24) & 0xFF),
  };

  drainReceiveQueue();
  if (!sendFrame(motorConfig(index).can_id, payload, sizeof(payload))) {
    return false;
  }

  state.last_motor_deg = target_motor_deg;
  if (state.have_zero) {
    Serial.printf("%s motor target=%.2f deg output_equiv=%.2f deg speed_out=%.2f dps speed_motor=%u dps\n",
                  motorConfig(index).name,
                  target_motor_deg,
                  motorToOutputDegrees(index, target_motor_deg),
                  state.output_speed_dps,
                  static_cast<unsigned>(motor_speed_dps));
  } else {
    Serial.printf("%s motor target=%.2f deg speed_out=%.2f dps speed_motor=%u dps (no software zero)\n",
                  motorConfig(index).name,
                  target_motor_deg,
                  state.output_speed_dps,
                  static_cast<unsigned>(motor_speed_dps));
  }
  return true;
}

bool commandOutputAngleDegrees(size_t index, double target_output_deg) {
  const MotorRuntimeState &state = motorState(index);
  if (!state.have_zero) {
    Serial.printf("%s has no software zero yet, send `on %s` or `zero %s` first\n",
                  motorConfig(index).name,
                  motorConfig(index).name,
                  motorConfig(index).name);
    return false;
  }

  const double target_motor_deg = outputToMotorDegrees(index, target_output_deg);
  Serial.printf("%s output target=%.2f deg using zero_motor_deg=%.2f ratio=%.2f\n",
                motorConfig(index).name,
                target_output_deg,
                state.zero_motor_deg,
                motorConfig(index).gear_ratio);
  return commandMotorAngleDegrees(index, target_motor_deg);
}

char *trimWhitespace(char *text) {
  while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
    ++text;
  }

  char *end = text + strlen(text);
  while (end > text && isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  *end = '\0';
  return text;
}

bool parseUnsignedLongToken(const char *token, unsigned long *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }

  char *end = nullptr;
  const unsigned long parsed = strtoul(token, &end, 0);
  if (end == token || *end != '\0') {
    return false;
  }

  *value = parsed;
  return true;
}

bool parseDoubleToken(const char *token, double *value) {
  if (token == nullptr || token[0] == '\0' || value == nullptr) {
    return false;
  }

  char *end = nullptr;
  const double parsed = strtod(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }

  *value = parsed;
  return true;
}

int tokenize(char *text, char *tokens[], int max_tokens) {
  int count = 0;
  for (char *token = strtok(text, " \t"); token != nullptr && count < max_tokens; token = strtok(nullptr, " \t")) {
    tokens[count++] = token;
  }
  return count;
}

bool findMotorIndexByToken(const char *token, size_t *index_out) {
  if (token == nullptr || index_out == nullptr) {
    return false;
  }

  if (equalsIgnoreCase(token, "joint5") || equalsIgnoreCase(token, "j5") ||
      equalsIgnoreCase(token, "14") || equalsIgnoreCase(token, "0x14") ||
      equalsIgnoreCase(token, "14e") || equalsIgnoreCase(token, "0x14e")) {
    *index_out = 0;
    return true;
  }

  if (equalsIgnoreCase(token, "joint6") || equalsIgnoreCase(token, "j6") ||
      equalsIgnoreCase(token, "15") || equalsIgnoreCase(token, "0x15") ||
      equalsIgnoreCase(token, "14f") || equalsIgnoreCase(token, "0x14f")) {
    *index_out = 1;
    return true;
  }

  unsigned long parsed = 0;
  if (parseUnsignedLongToken(token, &parsed)) {
    if (parsed == kMotors[0].can_id) {
      *index_out = 0;
      return true;
    }
    if (parsed == kMotors[1].can_id) {
      *index_out = 1;
      return true;
    }
  }

  return false;
}

void printUnknownTarget(const char *token) {
  Serial.printf("unknown target: %s\n", token == nullptr ? "<null>" : token);
  Serial.println("use joint5|j5|14|0x14|0x14E or joint6|j6|15|0x15|0x14F");
}

void runOnAllMotors(bool (*fn)(size_t)) {
  for (size_t index = 0; index < kMotorCount; ++index) {
    fn(index);
    delay(10);
  }
}

bool onMotor(size_t index) {
  return sendMotorOnAndCaptureZero(index);
}

bool zeroMotor(size_t index) {
  return captureZeroOffset(index);
}

bool posMotor(size_t index) {
  return readCurrentMotorAngle(index, nullptr);
}

bool offMotor(size_t index) {
  return sendMotorOff(index);
}

bool stopMotor(size_t index) {
  return sendMotorStop(index);
}

void handleUseCommand(int argc, char *argv[]) {
  if (argc != 2) {
    Serial.println("usage: use TARGET");
    return;
  }

  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }

  g_active_motor_index = index;
  Serial.printf("active motor set to %s (can_id=0x%03X)\n",
                motorConfig(index).name,
                motorConfig(index).can_id);
}

void handleStatusCommand(int argc, char *argv[]) {
  if (argc == 1 || (argc == 2 && equalsIgnoreCase(argv[1], "all"))) {
    printAllMotorStatus();
    return;
  }

  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }

  printMotorStatus(index);
}

void handleSimpleMotorAction(int argc, char *argv[], const char *command_name, bool (*fn)(size_t)) {
  if (argc == 1) {
    fn(g_active_motor_index);
    return;
  }

  if (argc != 2) {
    Serial.printf("usage: %s [TARGET|all]\n", command_name);
    return;
  }

  if (equalsIgnoreCase(argv[1], "all")) {
    runOnAllMotors(fn);
    return;
  }

  size_t index = 0;
  if (!findMotorIndexByToken(argv[1], &index)) {
    printUnknownTarget(argv[1]);
    return;
  }

  fn(index);
}

void handleSpeedCommand(int argc, char *argv[]) {
  size_t index = g_active_motor_index;
  const char *value_token = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findMotorIndexByToken(argv[1], &index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: speed DPS  or  speed TARGET DPS");
    return;
  }

  double output_dps = 0.0;
  if (!parseDoubleToken(value_token, &output_dps) || output_dps < 0.0) {
    Serial.println("invalid speed, expected non-negative deg/s");
    return;
  }

  motorState(index).output_speed_dps = output_dps;
  Serial.printf("%s output speed set to %.2f dps (motor speed=%u dps)\n",
                motorConfig(index).name,
                motorState(index).output_speed_dps,
                static_cast<unsigned>(outputSpeedToMotorDps(index, motorState(index).output_speed_dps)));
}

void handleGotoCommand(int argc, char *argv[]) {
  size_t index = g_active_motor_index;
  const char *value_token = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findMotorIndexByToken(argv[1], &index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: goto DEG  or  goto TARGET DEG");
    return;
  }

  double output_deg = 0.0;
  if (!parseDoubleToken(value_token, &output_deg)) {
    Serial.println("invalid output angle, expected degrees like: goto joint5 30");
    return;
  }

  commandOutputAngleDegrees(index, output_deg);
}

void handleMotorCommand(int argc, char *argv[]) {
  size_t index = g_active_motor_index;
  const char *value_token = nullptr;

  if (argc == 2) {
    value_token = argv[1];
  } else if (argc == 3) {
    if (!findMotorIndexByToken(argv[1], &index)) {
      printUnknownTarget(argv[1]);
      return;
    }
    value_token = argv[2];
  } else {
    Serial.println("usage: motor DEG  or  motor TARGET DEG");
    return;
  }

  double motor_deg = 0.0;
  if (!parseDoubleToken(value_token, &motor_deg)) {
    Serial.println("invalid motor angle, expected degrees like: motor joint6 360");
    return;
  }

  commandMotorAngleDegrees(index, motor_deg);
}

void handleSyncCommand(int argc, char *argv[]) {
  if (argc != 3) {
    Serial.println("usage: sync DEG5 DEG6");
    return;
  }

  double joint5_deg = 0.0;
  double joint6_deg = 0.0;
  if (!parseDoubleToken(argv[1], &joint5_deg) || !parseDoubleToken(argv[2], &joint6_deg)) {
    Serial.println("invalid sync angles, expected: sync 20 -15");
    return;
  }

  const bool ok5 = commandOutputAngleDegrees(0, joint5_deg);
  const bool ok6 = commandOutputAngleDegrees(1, joint6_deg);
  if (ok5 && ok6) {
    Serial.printf("sync sent: joint5=%.2f deg joint6=%.2f deg\n", joint5_deg, joint6_deg);
  }
}

void handleSerialLine(char *line) {
  char *trimmed = trimWhitespace(line);
  if (*trimmed == '\0') {
    return;
  }

  char *argv[4] = {};
  const int argc = tokenize(trimmed, argv, 4);
  if (argc <= 0) {
    return;
  }

  if (equalsIgnoreCase(argv[0], "help") || equalsIgnoreCase(argv[0], "h")) {
    printHelp();
    return;
  }
  if (equalsIgnoreCase(argv[0], "list")) {
    printAllMotorStatus();
    return;
  }
  if (equalsIgnoreCase(argv[0], "use")) {
    handleUseCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "status")) {
    handleStatusCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "on")) {
    handleSimpleMotorAction(argc, argv, "on", onMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "zero")) {
    handleSimpleMotorAction(argc, argv, "zero", zeroMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "pos")) {
    handleSimpleMotorAction(argc, argv, "pos", posMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "off")) {
    handleSimpleMotorAction(argc, argv, "off", offMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "stop")) {
    handleSimpleMotorAction(argc, argv, "stop", stopMotor);
    return;
  }
  if (equalsIgnoreCase(argv[0], "speed")) {
    handleSpeedCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "goto")) {
    handleGotoCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "motor")) {
    handleMotorCommand(argc, argv);
    return;
  }
  if (equalsIgnoreCase(argv[0], "sync")) {
    handleSyncCommand(argc, argv);
    return;
  }

  Serial.printf("unknown command: %s\n", argv[0]);
  printHelp();
}

void processSerialInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      g_serial_line[g_serial_line_len] = '\0';
      handleSerialLine(g_serial_line);
      g_serial_line_len = 0;
      g_serial_line[0] = '\0';
      continue;
    }

    if (g_serial_line_len + 1 >= sizeof(g_serial_line)) {
      Serial.println("serial command too long, clearing buffer");
      g_serial_line_len = 0;
      g_serial_line[0] = '\0';
      continue;
    }

    g_serial_line[g_serial_line_len++] = ch;
    g_serial_line[g_serial_line_len] = '\0';
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 + MCP2515 LKTech joint5/joint6 dummy controller starting...");
  Serial.println("Shared bus mode: 1 Mbps, 8 MHz MCP2515 oscillator");
  Serial.println("RX filtering enabled for joint5/joint6 only");

  initializeMotorStates();

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);

  if (!configureController()) {
    Serial.println("MCP2515 setup failed; halting.");
    while (true) {
      delay(1000);
    }
  }

  printHelp();
  Serial.println();
  Serial.println("Quick start:");
  Serial.println("  on all");
  Serial.println("  goto joint5 20");
  Serial.println("  goto joint6 -20");
  Serial.println("  sync 10 -10");
}

void loop() {
  processSerialInput();
  delay(1);
}
