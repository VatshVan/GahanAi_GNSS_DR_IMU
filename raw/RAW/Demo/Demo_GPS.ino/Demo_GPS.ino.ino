#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h> // Uses the installed 'Hideaki Tai' library

// --- WIFI SETTINGS ---
const char* ssid = "ESP32_ROBOT";
const char* password = "password123";

WiFiServer imuServer(8888);
WiFiServer gpsServer(8889);
WiFiClient imuClient;
WiFiClient gpsClient;

// --- SENSOR PINS ---
// GPS (Serial2)
#define RXD2 16
#define TXD2 17

// --- IMU OBJECTS ---
// FIXED: Removed <TwoWire> because MPU9250 is already defined as that type
MPU9250 IMU1;
MPU9250 IMU2;

// --- TIMING ---
unsigned long lastImuTime = 0;
const int imuInterval = 10; // 10ms = 100Hz

void setup() {
  Serial.begin(9600); // Change from 115200 to 9600
  Wire.begin();

  // 1. Setup WiFi
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP Started. Connect PC to: "); Serial.println(ssid);
  Serial.print("ESP32 IP Address: "); Serial.println(IP);
  
  // 2. Start Servers
  imuServer.begin();
  gpsServer.begin();

  // 3. Setup GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // 4. Setup IMUs (Hideaki Tai Library Syntax)
  // 0x68 = AD0 Low (GND), 0x69 = AD0 High (3.3V)
  // Note: .setup() returns true on success
  if (!IMU1.setup(0x68)) { 
    Serial.println("IMU1 (0x68) Failed");
  } else {
    Serial.println("IMU1 (0x68) Connected");
  }

  if (!IMU2.setup(0x69)) { 
    Serial.println("IMU2 (0x69) Failed");
  } else {
    Serial.println("IMU2 (0x69) Connected");
  }
}

void loop() {
  // --- A. HANDLE CLIENT CONNECTIONS ---
  if (imuServer.hasClient()) {
    if (imuClient) imuClient.stop();
    imuClient = imuServer.available();
    Serial.println("IMU Client Connected!");
  }
  if (gpsServer.hasClient()) {
    if (gpsClient) gpsClient.stop();
    gpsClient = gpsServer.available();
    Serial.println("GPS Client Connected!");
  }

  // --- B. GPS PASSTHROUGH (Port 8889) ---
  while (Serial2.available() && gpsClient.connected()) {
    char c = Serial2.read();
    gpsClient.write(c);
  }

  // --- C. IMU STREAMING (Port 8888) ---
  unsigned long now = millis();
  if (now - lastImuTime >= imuInterval) {
    lastImuTime = now;
    
    // Read Sensors
    IMU1.update();
    IMU2.update();

    if (imuClient.connected()) {
      String data = "";
      
      // --- IMU 1 (Front) ---
      // Library returns G's, multiply by 9.81 for m/s^2
      data += String(IMU1.getAccX() * 9.81, 2) + ",";
      data += String(IMU1.getAccY() * 9.81, 2) + ",";
      data += String(IMU1.getAccZ() * 9.81, 2) + ",";
      // Library returns Degrees/s, no conversion needed
      data += String(IMU1.getGyroX(), 2) + ","; 
      data += String(IMU1.getGyroY(), 2) + ",";
      data += String(IMU1.getGyroZ(), 2) + ",";
      
      // --- IMU 2 (Rear) ---
      data += String(IMU2.getAccX() * 9.81, 2) + ",";
      data += String(IMU2.getAccY() * 9.81, 2) + ",";
      data += String(IMU2.getAccZ() * 9.81, 2) + ",";
      data += String(IMU2.getGyroX(), 2) + ",";
      data += String(IMU2.getGyroY(), 2) + ",";
      data += String(IMU2.getGyroZ(), 2);
      
      imuClient.println(data);
    }
  }
}