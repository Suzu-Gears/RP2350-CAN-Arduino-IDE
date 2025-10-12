#include <Adafruit_MCP2515.h>

#define XL2515_SPI_PORT SPI1
#define XL2515_SCLK_PIN 10
#define XL2515_MOSI_PIN 11
#define XL2515_MISO_PIN 12
#define XL2515_CS_PIN 9
#define XL2515_INT_PIN 8

#define CAN_BAUDRATE (1000000)

const float P_GAIN = 100.0f;              // Pゲイン
const float TARGET_RPS = 10.0f;           // 目標回転数
const int16_t CURRENT_LIMIT_MA = 10000;   // 電流指令値の制限 (mA) (-10A to +10A)
const uint32_t MOTOR_COMMAND_ID = 0x200;  // 指令を送信するCAN ID

Adafruit_MCP2515 CAN(XL2515_CS_PIN, &XL2515_SPI_PORT);

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(115200);
  SPI1.setSCK(XL2515_SCLK_PIN);
  SPI1.setTX(XL2515_MOSI_PIN);
  SPI1.setRX(XL2515_MISO_PIN);
  pinMode(XL2515_CS_PIN, OUTPUT);
  pinMode(XL2515_INT_PIN, INPUT);

  CAN.begin(CAN_BAUDRATE);
}

void loop() {
  int packetSize = CAN.parsePacket();
  if (!CAN.packetRtr()) {
    uint8_t rxMsg[packetSize] = { 0 };
    for (int i = 0; i < packetSize && CAN.available(); i++) {
      rxMsg[i] = CAN.read();
    }

    int16_t rotorPositionRaw = (int16_t)(rxMsg[0] << 8 | rxMsg[1]);
    int16_t rpm = (int16_t)(rxMsg[2] << 8 | rxMsg[3]);
    int16_t actualTorqueCurrent = (int16_t)(rxMsg[4] << 8 | rxMsg[5]);
    // rxMsg[6] and rxMsg[7] are Null

    float rotorDegree = fmap(rotorPositionRaw, 0, 8192.0f, 0, 360.0f);
    float rps = rpm / 60.0f;

    float error = TARGET_RPS - rps;
    float targetCurrent = P_GAIN * error;
    int16_t commandCurrent = constrain(targetCurrent, -CURRENT_LIMIT_MA, CURRENT_LIMIT_MA);

    uint8_t txMsg[8] = { 0 };
    txMsg[0] = commandCurrent >> 8;  // モーターid1 電流値上位バイト8ビット
    txMsg[1] = commandCurrent;       // モーターid1 電流値下位バイト8ビット
    txMsg[2] = commandCurrent >> 8;  // モーターid2 電流値上位バイト8ビット
    txMsg[3] = commandCurrent;       // モーターid2 電流値下位バイト8ビット
    txMsg[4] = commandCurrent >> 8;  // モーターid3 電流値上位バイト8ビット
    txMsg[5] = commandCurrent;       // モーターid3 電流値下位バイト8ビット
    txMsg[6] = commandCurrent >> 8;  // モーターid4 電流値上位バイト8ビット
    txMsg[7] = commandCurrent;       // モーターid4 電流値下位バイト8ビット
    CAN.beginPacket(MOTOR_COMMAND_ID);
    for (int i = 0; i < 8; i++) {
      CAN.write(txMsg[i]);
    }
    CAN.endPacket();

    Serial.print("ID: 0x");
    Serial.print(CAN.packetId(), HEX);
    Serial.print(", Angle: ");
    Serial.print(rotorDegree);
    Serial.print(" deg, RPS: ");
    Serial.print(rps);
    Serial.print(", Command_mA: ");
    Serial.print(commandCurrent);
    Serial.print(", Actual_mA: ");
    Serial.print(actualTorqueCurrent);
    Serial.println();
  }
  delay(1);
}
