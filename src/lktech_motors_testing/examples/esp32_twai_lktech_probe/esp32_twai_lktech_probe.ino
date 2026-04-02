#include <Arduino.h>
#include "driver/twai.h"

// ESP32 built-in TWAI controller + external CAN transceiver.
// Wiring for your stated setup:
//   GPIO22 (CTX) -> transceiver TXD
//   GPIO21 (CRX) -> transceiver RXD
//   transceiver CANH -> motor CANH
//   transceiver CANL -> motor CANL
//   transceiver GND -> ESP32 GND
//
// This sketch is intentionally conservative:
// - It starts at 500 kbps
// - It accepts all CAN frames
// - It prints TWAI alerts, status, and received frames
// - It provides read-only LKTech probe commands over Serial
//
// The ESP-IDF TWAI driver API and default config macros are documented by
// Espressif here:
// https://docs.espressif.com/projects/esp-idf/en/v4.2.3/esp32/api-reference/peripherals/twai.html

static constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_22;
static constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_21;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t STATUS_PERIOD_MS = 1000;
static constexpr uint16_t LKTECH_MOTOR_ID = 15;
static constexpr uint16_t LKTECH_REQUEST_ID_BASE = 0x140;

static uint32_t g_last_status_ms = 0;

uint32_t lktechRequestId(uint8_t motor_id) {
  return LKTECH_REQUEST_ID_BASE + motor_id;
}

void printFrame(const twai_message_t &message) {
  Serial.printf(
      "RX id=0x%03lX ext=%d rtr=%d dlc=%d data=",
      static_cast<unsigned long>(message.identifier),
      message.extd ? 1 : 0,
      message.rtr ? 1 : 0,
      message.data_length_code);

  for (int i = 0; i < message.data_length_code; ++i) {
    Serial.printf("%02X", message.data[i]);
    if (i + 1 < message.data_length_code) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void printStatus() {
  twai_status_info_t status_info;
  if (twai_get_status_info(&status_info) != ESP_OK) {
    Serial.println("status: failed to read TWAI status");
    return;
  }

  Serial.printf(
      "status state=%d tx_err=%lu rx_err=%lu msgs_to_tx=%lu msgs_to_rx=%lu tx_failed=%lu bus_error_count=%lu arb_lost=%lu\n",
      static_cast<int>(status_info.state),
      static_cast<unsigned long>(status_info.tx_error_counter),
      static_cast<unsigned long>(status_info.rx_error_counter),
      static_cast<unsigned long>(status_info.msgs_to_tx),
      static_cast<unsigned long>(status_info.msgs_to_rx),
      static_cast<unsigned long>(status_info.tx_failed_count),
      static_cast<unsigned long>(status_info.bus_error_count),
      static_cast<unsigned long>(status_info.arb_lost_count));
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h  - help");
  Serial.println("  r  - send LKTech read multi-loop angle request (0x92)");
  Serial.println("  a  - send LKTech read single-loop angle request (0x94)");
  Serial.println("  s  - send LKTech read state 2 request (0x9C)");
  Serial.println("  p  - print current TWAI status immediately");
  Serial.println("  b  - initiate TWAI bus recovery");
}

bool sendStandardFrame(uint32_t arbitration_id, const uint8_t data[8]) {
  twai_message_t message = {};
  message.identifier = arbitration_id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  memcpy(message.data, data, 8);

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    Serial.printf("TX failed id=0x%03lX err=0x%X\n",
                  static_cast<unsigned long>(arbitration_id),
                  static_cast<unsigned>(err));
    return false;
  }

  Serial.printf("TX id=0x%03lX data=", static_cast<unsigned long>(arbitration_id));
  for (int i = 0; i < 8; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < 8) {
      Serial.print(" ");
    }
  }
  Serial.println();
  return true;
}

void sendReadMultiLoopAngle() {
  const uint8_t payload[8] = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(LKTECH_MOTOR_ID), payload);
}

void sendReadSingleLoopAngle() {
  const uint8_t payload[8] = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(LKTECH_MOTOR_ID), payload);
}

void sendReadState2() {
  const uint8_t payload[8] = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendStandardFrame(lktechRequestId(LKTECH_MOTOR_ID), payload);
}

void handleSerialCommand(char command) {
  switch (command) {
    case 'h':
    case '?':
      printHelp();
      break;
    case 'r':
      sendReadMultiLoopAngle();
      break;
    case 'a':
      sendReadSingleLoopAngle();
      break;
    case 's':
      sendReadState2();
      break;
    case 'p':
      printStatus();
      break;
    case 'b': {
      esp_err_t err = twai_initiate_recovery();
      Serial.printf("bus recovery requested err=0x%X\n", static_cast<unsigned>(err));
      break;
    }
    case '\n':
    case '\r':
      break;
    default:
      Serial.printf("unknown command '%c'\n", command);
      printHelp();
      break;
  }
}

void handleAlerts() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) != ESP_OK || alerts == 0) {
    return;
  }

  Serial.printf("alert flags=0x%08lX", static_cast<unsigned long>(alerts));
  if (alerts & TWAI_ALERT_RX_DATA) Serial.print(" RX_DATA");
  if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.print(" TX_SUCCESS");
  if (alerts & TWAI_ALERT_TX_FAILED) Serial.print(" TX_FAILED");
  if (alerts & TWAI_ALERT_ERR_PASS) Serial.print(" ERR_PASS");
  if (alerts & TWAI_ALERT_BUS_ERROR) Serial.print(" BUS_ERROR");
  if (alerts & TWAI_ALERT_BUS_OFF) Serial.print(" BUS_OFF");
  if (alerts & TWAI_ALERT_BUS_RECOVERED) Serial.print(" BUS_RECOVERED");
  if (alerts & TWAI_ALERT_ARB_LOST) Serial.print(" ARB_LOST");
  Serial.println();
}

void handleReceive() {
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    printFrame(message);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println("ESP32 TWAI LKTech probe starting...");
  Serial.printf("TWAI TX pin=%d RX pin=%d bitrate=500000\n",
                static_cast<int>(TWAI_TX_PIN),
                static_cast<int>(TWAI_RX_PIN));

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  g_config.tx_queue_len = 8;
  g_config.rx_queue_len = 32;
  g_config.alerts_enabled =
      TWAI_ALERT_RX_DATA |
      TWAI_ALERT_TX_SUCCESS |
      TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_ERR_PASS |
      TWAI_ALERT_BUS_ERROR |
      TWAI_ALERT_BUS_OFF |
      TWAI_ALERT_BUS_RECOVERED |
      TWAI_ALERT_ARB_LOST;

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed err=0x%X\n", static_cast<unsigned>(err));
    return;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed err=0x%X\n", static_cast<unsigned>(err));
    return;
  }

  Serial.println("TWAI started.");
  printHelp();
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    handleSerialCommand(static_cast<char>(Serial.read()));
  }

  handleAlerts();
  handleReceive();

  const uint32_t now = millis();
  if (now - g_last_status_ms >= STATUS_PERIOD_MS) {
    g_last_status_ms = now;
    printStatus();
  }

  delay(10);
}
