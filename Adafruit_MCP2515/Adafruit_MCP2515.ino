/*
 * Adafruit MCP2515 FeatherWing CAN Receiver Example
 */


#include <Adafruit_MCP2515.h>

#define XL2515_SPI_PORT spi1
#define XL2515_SCLK_PIN 10
#define XL2515_MOSI_PIN 11
#define XL2515_MISO_PIN 12
#define XL2515_CS_PIN 9
#define XL2515_INT_PIN 8

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

Adafruit_MCP2515 mcp(XL2515_CS_PIN, &SPI1);

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
  // try to parse packet
  int packetSize = mcp.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (mcp.packetExtended()) {
      Serial.print("extended ");
    }

    if (mcp.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(mcp.packetId(), HEX);

    if (mcp.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(mcp.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (mcp.available()) {
        Serial.print((char)mcp.read());
      }
      Serial.println();
    }

    Serial.println();
  }
}
