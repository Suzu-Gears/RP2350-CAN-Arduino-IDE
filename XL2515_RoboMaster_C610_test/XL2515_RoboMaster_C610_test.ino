#include <Adafruit_MCP2515.h>

#define XL2515_SPI_PORT SPI1
#define XL2515_SCLK_PIN 10
#define XL2515_MOSI_PIN 11
#define XL2515_MISO_PIN 12
#define XL2515_CS_PIN 9
#define XL2515_INT_PIN 8

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

Adafruit_MCP2515 mcp(XL2515_CS_PIN, &XL2515_SPI_PORT);

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t speedControllerID = 1;
float rps = 0;
int16_t rotorPosition = 0;
float rotorDegree = 0;
int16_t id = 0;
int16_t actualTorqueCurrent = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // SPI1ピンの初期化
  SPI1.setSCK(XL2515_SCLK_PIN);
  SPI1.setTX(XL2515_MOSI_PIN);
  SPI1.setRX(XL2515_MISO_PIN);
  pinMode(XL2515_CS_PIN, OUTPUT);
  pinMode(XL2515_INT_PIN, INPUT);

  Serial.println("MCP2515 Receiver test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while (1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {
  int packetSize = mcp.parsePacket();
  uint8_t buf[8] = { 0 };
  if (packetSize == 8 && !mcp.packetRtr()) {
    id = mcp.packetId();
    int i = 0;
    while (mcp.available() && i < 8) {
      buf[i++] = mcp.read();
    }
    rotorPosition = (int16_t)(buf[0] << 8 | buf[1]);
    rotorDegree = fmap(rotorPosition, 0, 8192, 0, 360);
    rps = (int16_t)(buf[2] << 8 | buf[3]) / 60.0f;
    actualTorqueCurrent = (int16_t)(buf[4] << 8 | buf[5]);
    // buf[6], buf[7]は未使用
  }

  float Kp = 100;
  float rpsRef = 10;
  float curRef = Kp * (rpsRef - rps);
  int curRefI = constrain(curRef, -10000, 10000);

  uint8_t txBuf[8] = { 0 };
  txBuf[0] = curRefI >> 8;
  txBuf[1] = curRefI & 0xFF;
  // 残りは0のまま
  mcp.beginPacket(0x200);
  for (int i = 0; i < 8; i++) {
    mcp.write(txBuf[i]);
  }
  mcp.endPacket();

  Serial.print("ID: 0x");
  Serial.print(id, HEX);
  Serial.print(", Angle: ");
  Serial.print(rotorDegree);
  Serial.print(", RPS: ");
  Serial.print(rps);
  Serial.print(", RefCurrent: ");
  Serial.print(curRefI);
  Serial.print(", TorqueCurrent: ");
  Serial.print(actualTorqueCurrent);
  Serial.println();

  delay(1);
}
