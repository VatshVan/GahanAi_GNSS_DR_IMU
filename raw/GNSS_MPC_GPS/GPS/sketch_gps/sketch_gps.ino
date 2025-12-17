#include <SPI.h>

const int CS_PIN = 8; // Your CS Pin

void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  
  Serial.println("--- MANUAL SPI TEST ---");
  
  // Test: Reset the chip
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xC0); // MCP2515 Reset Command
  digitalWrite(CS_PIN, HIGH);
  delay(10);
  
  // Test: Read Status Register
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xA0); // Read Status Command
  byte status = SPI.transfer(0x00); // Read byte
  digitalWrite(CS_PIN, HIGH);
  
  Serial.print("Status Register Read: 0x");
  Serial.println(status, HEX);
  
  if (status == 0xFF || status == 0x00) {
    Serial.println("❌ FAILURE: Chip is not responding (Check Wiring/Power)");
  } else {
    Serial.println("✅ SUCCESS: Chip is ALIVE (Library config was wrong)");
  }
}

void loop() {}