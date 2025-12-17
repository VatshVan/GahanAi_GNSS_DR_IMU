#include <Wire.h>

// --- HARDWARE CONFIG ---
#define IMU1_ADDR 0x68
#define IMU2_ADDR 0x69
#define SDA_PIN 21
#define SCL_PIN 22
#define FSYNC_PIN 4

// MPU9250 Registers
#define PWR_MGMT_1    0x6B
#define CONFIG        0x1A
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define ACCEL_XOUT_H  0x3B

// Scales
const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;

void setup() {
  Serial.begin(115200);
  while (Serial.available()) Serial.read();

  pinMode(FSYNC_PIN, OUTPUT);
  digitalWrite(FSYNC_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  initIMU(IMU1_ADDR);
  initIMU(IMU2_ADDR);

  delay(2000);   // Let ESP32 boot messages finish

//  Serial.println("ACC1_X,ACC1_Y,ACC1_Z,GYR1_X,GYR1_Y,GYR1_Z,ACC2_X,ACC2_Y,ACC2_Z,GYR2_X,GYR2_Y,GYR2_Z");
}

void loop() {
  // FSYNC pulse (hardware sync)
  digitalWrite(FSYNC_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(FSYNC_PIN, LOW);

  delay(2);

  float ax1, ay1, az1, gx1, gy1, gz1;
  float ax2, ay2, az2, gx2, gy2, gz2;

  readIMU(IMU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);
  readIMU(IMU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);

  // One line per sample (Serial Plotter friendly)
  Serial.printf(
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    ax1, ay1, az1, gx1, gy1, gz1,
    ax2, ay2, az2, gx2, gy2, gz2
  );

  delay(8); // ~100 Hz total loop
}

// ---------------- IMU FUNCTIONS ----------------

void initIMU(uint8_t address) {
  writeRegister(address, PWR_MGMT_1, 0x00);
  delay(10);

  // Enable FSYNC (EXT_SYNC_SET = 1), DLPF = 1
  writeRegister(address, CONFIG, 0x08 | 0x01);

  writeRegister(address, ACCEL_CONFIG, 0x00); // ±2g
  writeRegister(address, GYRO_CONFIG, 0x00);  // ±250 dps
}

void readIMU(uint8_t address,
             float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {

  int16_t rax, ray, raz, rgx, rgy, rgz;

  Wire.beginTransmission(address);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)14);

  if (Wire.available() == 14) {
    rax = Wire.read() << 8 | Wire.read();
    ray = Wire.read() << 8 | Wire.read();
    raz = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // temperature
    rgx = Wire.read() << 8 | Wire.read();
    rgy = Wire.read() << 8 | Wire.read();
    rgz = Wire.read() << 8 | Wire.read();

    ax = (float)rax / ACC_SCALE * 9.81;
    ay = (float)ray / ACC_SCALE * 9.81;
    az = (float)raz / ACC_SCALE * 9.81;
    gx = (float)rgx / GYRO_SCALE;
    gy = (float)rgy / GYRO_SCALE;
    gz = (float)rgz / GYRO_SCALE;
  }
}

void writeRegister(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
