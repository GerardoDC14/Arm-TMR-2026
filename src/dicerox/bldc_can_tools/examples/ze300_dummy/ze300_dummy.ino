#include <Arduino.h>
#include <SPI.h>
#include <ctype.h>
#include <mcp2515.h>
#include <stdlib.h>
#include <string.h>

// ZE300 position controller.
//
// The ZE300 has an absolute multi-turn encoder that retains position across
// power cycles — no software zero-offset needed.
//
// ZE300 CAN protocol:
// - Request ID: 0x100 | device_address
// - Reply ID:   device_address
// - A3: read single-turn and multi-turn absolute angle in counts (16384 counts/rev)
// - B2: set position-mode max speed in 0.01 rpm
// - C2: absolute position in counts
// - C3: relative position in counts
// - CF: disable output / free state

static constexpr uint32_t SERIAL_BAUD        = 115200;
static constexpr uint8_t  SPI_SCK_PIN        = 18;
static constexpr uint8_t  SPI_MISO_PIN       = 19;
static constexpr uint8_t  SPI_MOSI_PIN       = 23;
static constexpr uint8_t  MCP2515_CS_PIN     = 5;
static constexpr uint8_t  MCP2515_INT_PIN    = 4;

static constexpr CAN_SPEED FIXED_CAN_SPEED   = CAN_500KBPS;
static constexpr CAN_CLOCK FIXED_CAN_CLOCK   = MCP_8MHZ;
static constexpr uint8_t  ZE300_ADDRESS      = 13;
static constexpr uint16_t COUNTS_PER_REV     = 16384;
static constexpr double   DEFAULT_SPEED_DPS  = 100.0;
static constexpr unsigned long REPLY_TIMEOUT_MS = 250;

static constexpr uint8_t CMD_READ_POS   = 0xA3;
static constexpr uint8_t CMD_SET_SPEED  = 0xB2;
static constexpr uint8_t CMD_ABS_POS    = 0xC2;
static constexpr uint8_t CMD_REL_POS    = 0xC3;
static constexpr uint8_t CMD_DISABLE    = 0xCF;

MCP2515 g_mcp2515(MCP2515_CS_PIN);
char     g_line[128]   = {};
size_t   g_line_len    = 0;
int32_t  g_last_counts = 0;
uint32_t g_speed_centirpm = 0;

uint16_t requestId() { return static_cast<uint16_t>(0x100 | ZE300_ADDRESS); }
uint16_t replyId()   { return ZE300_ADDRESS; }

int32_t readI32LE(const uint8_t *d, uint8_t o) {
  return static_cast<int32_t>(
    static_cast<uint32_t>(d[o]) | (static_cast<uint32_t>(d[o+1]) << 8) |
    (static_cast<uint32_t>(d[o+2]) << 16) | (static_cast<uint32_t>(d[o+3]) << 24));
}
uint16_t readU16LE(const uint8_t *d, uint8_t o) {
  return static_cast<uint16_t>(d[o]) | (static_cast<uint16_t>(d[o+1]) << 8);
}

double countsToDeg(int32_t counts) {
  return static_cast<double>(counts) * 360.0 / COUNTS_PER_REV;
}
int32_t degToCounts(double deg) {
  const double s = deg * COUNTS_PER_REV / 360.0;
  return static_cast<int32_t>(s >= 0.0 ? s + 0.5 : s - 0.5);
}
uint32_t dpsToCentiRpm(double dps) {
  const double s = dps / 6.0 * 100.0;
  return s <= 0.0 ? 0 : static_cast<uint32_t>(s + 0.5);
}
double centiRpmToDps(uint32_t cr) { return static_cast<double>(cr) / 100.0 * 6.0; }

void printFrame(const can_frame &f, const char *prefix) {
  Serial.printf("%s id=0x%03lX dlc=%u data=", prefix,
                static_cast<unsigned long>(f.can_id & CAN_SFF_MASK),
                static_cast<unsigned>(f.can_dlc));
  for (uint8_t i = 0; i < f.can_dlc; ++i) {
    if (i) Serial.print(" ");
    Serial.printf("%02X", f.data[i]);
  }
  Serial.println();
}

void printStatus() {
  Serial.printf("device=0x%02X req=0x%03X reply=0x%03X speed=%.2f rpm (%.2f dps) pos=%.2f deg (%ld counts)\n",
                ZE300_ADDRESS, requestId(), replyId(),
                static_cast<double>(g_speed_centirpm) / 100.0,
                centiRpmToDps(g_speed_centirpm),
                countsToDeg(g_last_counts), static_cast<long>(g_last_counts));
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  pos            read current position");
  Serial.println("  goto DEG       absolute position in degrees");
  Serial.println("  step DEG       relative motion in degrees");
  Serial.println("  speed DPS      set max speed (deg/s)");
  Serial.println("  speedrpm RPM   set max speed (motor rpm)");
  Serial.println("  off            disable motor output");
  Serial.println("  status         print status");
  Serial.println("  help           show this help");
}

bool configureController() {
  g_mcp2515.reset();

  if (g_mcp2515.setBitrate(FIXED_CAN_SPEED, FIXED_CAN_CLOCK) != MCP2515::ERROR_OK) {
    Serial.println("setBitrate failed");
    return false;
  }

  // Hardware filters: only accept frames from the ZE300 reply ID.
  // Prevents ODrive heartbeats from filling RX buffers and dropping ZE300 replies.
  const uint32_t reply = replyId();
  g_mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  g_mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  g_mcp2515.setFilter(MCP2515::RXF0, false, reply);
  g_mcp2515.setFilter(MCP2515::RXF1, false, reply);
  g_mcp2515.setFilter(MCP2515::RXF2, false, reply);
  g_mcp2515.setFilter(MCP2515::RXF3, false, reply);
  g_mcp2515.setFilter(MCP2515::RXF4, false, reply);
  g_mcp2515.setFilter(MCP2515::RXF5, false, reply);

  if (g_mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("setNormalMode failed");
    return false;
  }

  Serial.println("MCP2515 configured.");
  printStatus();
  return true;
}

bool sendFrame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  can_frame f = {};
  f.can_id  = id;
  f.can_dlc = dlc;
  memcpy(f.data, data, dlc);
  if (g_mcp2515.sendMessage(&f) != MCP2515::ERROR_OK) {
    Serial.printf("sendMessage failed id=0x%03X\n", id);
    return false;
  }
  printFrame(f, "TX");
  return true;
}

void drainRx() {
  can_frame f = {};
  while (g_mcp2515.readMessage(&f) == MCP2515::ERROR_OK) {}
}

bool waitForReply(uint8_t cmd, can_frame *out) {
  const unsigned long t0 = millis();
  can_frame f = {};
  while (millis() - t0 < REPLY_TIMEOUT_MS) {
    if (g_mcp2515.readMessage(&f) == MCP2515::ERROR_OK) {
      printFrame(f, "RX");
      if ((f.can_id & CAN_SFF_MASK) == replyId() && f.can_dlc >= 1 && f.data[0] == cmd) {
        if (out) *out = f;
        return true;
      }
    }
    delay(2);
  }
  Serial.printf("timeout cmd=0x%02X\n", static_cast<unsigned>(cmd));
  return false;
}

bool send1(uint8_t cmd, can_frame *out) {
  drainRx();
  const uint8_t payload[1] = {cmd};
  return sendFrame(requestId(), payload, 1) && waitForReply(cmd, out);
}

bool send5(uint8_t cmd, int32_t val, can_frame *out) {
  drainRx();
  const uint8_t payload[5] = {
    cmd,
    static_cast<uint8_t>(val & 0xFF),
    static_cast<uint8_t>((val >> 8) & 0xFF),
    static_cast<uint8_t>((val >> 16) & 0xFF),
    static_cast<uint8_t>((val >> 24) & 0xFF),
  };
  return sendFrame(requestId(), payload, 5) && waitForReply(cmd, out);
}

bool readPos() {
  can_frame r = {};
  if (!send1(CMD_READ_POS, &r) || r.can_dlc != 7) return false;
  const int32_t multi = readI32LE(r.data, 3);
  g_last_counts = multi;
  Serial.printf("pos single=%.2f deg  multi=%.2f deg (%ld counts)\n",
                countsToDeg(readU16LE(r.data, 1)),
                countsToDeg(multi), static_cast<long>(multi));
  return true;
}

bool setSpeed(uint32_t centirpm) {
  if (centirpm > 0x7FFFFFFF) { Serial.println("speed too large"); return false; }
  can_frame r = {};
  if (!send5(CMD_SET_SPEED, static_cast<int32_t>(centirpm), &r) || r.can_dlc != 5) return false;
  g_speed_centirpm = static_cast<uint32_t>(readI32LE(r.data, 1));
  Serial.printf("speed set: %.2f rpm (%.2f dps)\n",
                static_cast<double>(g_speed_centirpm) / 100.0,
                centiRpmToDps(g_speed_centirpm));
  return true;
}

bool gotoAbsDeg(double deg) {
  const int32_t counts = degToCounts(deg);
  Serial.printf("goto %.2f deg -> %ld counts\n", deg, static_cast<long>(counts));
  can_frame r = {};
  if (!send5(CMD_ABS_POS, counts, &r)) return false;
  if (r.can_dlc == 7) {
    g_last_counts = readI32LE(r.data, 3);
    Serial.printf("ack pos=%.2f deg (%ld counts)\n",
                  countsToDeg(g_last_counts), static_cast<long>(g_last_counts));
  }
  return true;
}

bool stepRelDeg(double deg) {
  const int32_t delta = degToCounts(deg);
  Serial.printf("step %.2f deg -> %ld counts\n", deg, static_cast<long>(delta));
  can_frame r = {};
  if (!send5(CMD_REL_POS, delta, &r)) return false;
  if (r.can_dlc == 7) {
    g_last_counts = readI32LE(r.data, 3);
    Serial.printf("ack pos=%.2f deg (%ld counts)\n",
                  countsToDeg(g_last_counts), static_cast<long>(g_last_counts));
  }
  return true;
}

char *trim(char *s) {
  while (*s && isspace((unsigned char)*s)) ++s;
  char *e = s + strlen(s);
  while (e > s && isspace((unsigned char)*(e-1))) --e;
  *e = '\0';
  return s;
}

bool parseDouble(const char *s, double *v) {
  if (!s || !s[0] || !v) return false;
  char *e = nullptr;
  *v = strtod(s, &e);
  return e != s && *e == '\0';
}

void handleLine(char *line) {
  char *t = trim(line);
  if (!*t) return;

  if (!strcmp(t, "help") || !strcmp(t, "h")) { printHelp(); return; }
  if (!strcmp(t, "status"))                   { printStatus(); return; }
  if (!strcmp(t, "pos"))                      { readPos(); return; }
  if (!strcmp(t, "off"))                      { send1(CMD_DISABLE, nullptr); return; }

  double v = 0.0;
  if (!strncmp(t, "speedrpm", 8)) {
    if (!parseDouble(trim(t + 8), &v) || v < 0.0) { Serial.println("usage: speedrpm RPM"); return; }
    setSpeed(static_cast<uint32_t>(v * 100.0 + 0.5));
    return;
  }
  if (!strncmp(t, "speed", 5)) {
    if (!parseDouble(trim(t + 5), &v) || v < 0.0) { Serial.println("usage: speed DPS"); return; }
    setSpeed(dpsToCentiRpm(v));
    return;
  }
  if (!strncmp(t, "goto", 4)) {
    if (!parseDouble(trim(t + 4), &v)) { Serial.println("usage: goto DEG"); return; }
    gotoAbsDeg(v);
    return;
  }
  if (!strncmp(t, "step", 4)) {
    if (!parseDouble(trim(t + 4), &v)) { Serial.println("usage: step DEG"); return; }
    stepRelDeg(v);
    return;
  }

  Serial.printf("unknown: %s\n", t);
  printHelp();
}

void processSerial() {
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      g_line[g_line_len] = '\0';
      handleLine(g_line);
      g_line_len = 0;
      g_line[0]  = '\0';
      continue;
    }
    if (g_line_len + 1 >= sizeof(g_line)) {
      Serial.println("line too long");
      g_line_len = 0; g_line[0] = '\0';
      continue;
    }
    g_line[g_line_len++] = ch;
    g_line[g_line_len]   = '\0';
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\nESP32 + MCP2515 ZE300 controller starting...");
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, MCP2515_CS_PIN);
  if (!configureController()) Serial.println("MCP2515 init failed");
  setSpeed(dpsToCentiRpm(DEFAULT_SPEED_DPS));
  printHelp();
}

void loop() {
  processSerial();
  delay(5);
}
