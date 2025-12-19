#include <WiFi.h>
#include <Wire.h>
#include <MPU9250.h> // Bolder Flight Systems MPU9250

// --- WIFI SETTINGS ---
const char* ssid = "ESP32_ROBOT";  // Name of the WiFi network ESP will create
const char* password = "password123"; // Password (min 8 chars)

WiFiServer imuServer(8888);
WiFiServer gpsServer(8889);
WiFiClient imuClient;
WiFiClient gpsClient;

// --- SENSOR PINS ---
// GPS (Serial2)
#define RXD2 16
#define TXD2 17
// IMU (I2C)
MPU9250 IMU1(Wire, 0x68);
MPU9250 IMU2(Wire, 0x69);

// --- TIMING ---
unsigned long lastImuTime = 0;
const int imuInterval = 10; // 10ms = 100Hz

void setup() {
  Serial.begin(115200);

  // 1. Setup WiFi (Access Point Mode)
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP Started. Connect PC to: "); Serial.println(ssid);
  Serial.print("ESP32 IP Address: "); Serial.println(IP);
  
  // 2. Start Servers
  imuServer.begin();
  gpsServer.begin();

  // 3. Setup GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // 4. Setup IMUs
  Wire.begin();
  if (IMU1.begin() < 0) Serial.println("IMU1 Failed");
  if (IMU2.begin() < 0) Serial.println("IMU2 Failed");
}

void loop() {
  // --- A. HANDLE CLIENT CONNECTIONS ---
  if (imuServer.hasClient()) {
    if (imuClient) imuClient.stop(); // Disconnect old client
    imuClient = imuServer.available();
    Serial.println("IMU Client Connected!");
  }
  if (gpsServer.hasClient()) {
    if (gpsClient) gpsClient.stop(); // Disconnect old client
    gpsClient = gpsServer.available();
    Serial.println("GPS Client Connected!");
  }

  // --- B. GPS PASSTHROUGH (Port 8889) ---
  // Read from GPS Hardware -> Send to WiFi Client
  while (Serial2.available() && gpsClient.connected()) {
    char c = Serial2.read();
    gpsClient.write(c);
  }

  // --- C. IMU STREAMING (Port 8888) ---
  // Send at ~100Hz
  unsigned long now = millis();
  if (now - lastImuTime >= imuInterval) {
    lastImuTime = now;
    
    // Read Sensors
    IMU1.readSensor();
    IMU2.readSensor();

    if (imuClient.connected()) {
      // Format: ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2
      // Note: Bolder Flight returns m/s^2 and rad/s.
      // Your Python script expects m/s^2 and DEGREES/s (it converts to rads later).
      // We must convert Rad->Deg here to match your Python expectations.
      
      String data = "";
      // IMU 1
      data += String(IMU1.getAccelX_mss(), 2) + ",";
      data += String(IMU1.getAccelY_mss(), 2) + ",";
      data += String(IMU1.getAccelZ_mss(), 2) + ",";
      data += String(IMU1.getGyroX_rads() * 57.2958, 2) + ","; // Convert to Deg
      data += String(IMU1.getGyroY_rads() * 57.2958, 2) + ",";
      data += String(IMU1.getGyroZ_rads() * 57.2958, 2) + ",";
      
      // IMU 2
      data += String(IMU2.getAccelX_mss(), 2) + ",";
      data += String(IMU2.getAccelY_mss(), 2) + ",";
      data += String(IMU2.getAccelZ_mss(), 2) + ",";
      data += String(IMU2.getGyroX_rads() * 57.2958, 2) + ",";
      data += String(IMU2.getGyroY_rads() * 57.2958, 2) + ",";
      data += String(IMU2.getGyroZ_rads() * 57.2958, 2); // No comma at end
      
      imuClient.println(data);
    }
  }
}