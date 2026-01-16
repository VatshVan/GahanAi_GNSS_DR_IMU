#include <Wire.h>
#include <math.h>

// --- HARDWARE CONFIG ---
#define IMU1_ADDR 0x68
#define IMU2_ADDR 0x69
#define MAG_ADDR  0x0C  // Fixed Address of AK8963 Magnetometer

#define SDA_PIN 21
#define SCL_PIN 22
#define FSYNC_PIN 4

// MPU9250 Registers
#define PWR_MGMT_1    0x6B
#define INT_PIN_CFG   0x37  // Bypass Enable Configuration
#define CONFIG        0x1A
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define ACCEL_XOUT_H  0x3B

// AK8963 (Magnetometer) Registers
#define MAG_CNTL1     0x0A
#define MAG_HXL       0x03

// Scales
const float ACC_SCALE  = 16384.0;
const float GYRO_SCALE = 131.0;
const float MAG_SCALE  = 0.15; // approx uT per LSB

// Mag Calibration (You must calibrate these for your specific cart!)
float mag_bias_x = 0, mag_bias_y = 0; 

void setup() {
  Serial.begin(115200);
  while (Serial.available()) Serial.read();

  pinMode(FSYNC_PIN, OUTPUT);
  digitalWrite(FSYNC_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Initialize Accelerometer & Gyro
  initIMU(IMU1_ADDR);
  initIMU(IMU2_ADDR);

  // Initialize Magnetometers (Complex due to address conflict)
  initMagMultiplexed();

  delay(1000);
}

void loop() {
  // 1. FSYNC pulse
  digitalWrite(FSYNC_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(FSYNC_PIN, LOW);

  // 2. Read ACCEL & GYRO (Standard)
  float ax1, ay1, az1, gx1, gy1, gz1;
  float ax2, ay2, az2, gx2, gy2, gz2;
  
  readAccelGyro(IMU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);
  readAccelGyro(IMU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);

  // 3. Read MAGNETOMETERS (Multiplexed)
  float mx1, my1, mz1;
  float mx2, my2, mz2;
  
  readMagMultiplexed(mx1, my1, mz1, mx2, my2, mz2);

  // 4. Calculate Heading (Basic atan2)
  // We average the two mags for robustness
  float mx_avg = (mx1 + mx2) / 2.0;
  float my_avg = (my1 + my2) / 2.0;
  
  // Calculate Heading in Degrees
  // Note: For a golf cart, X is usually Forward, Y is Right.
  // Standard atan2(y, x) gives angle from East. Adjust as per mounting.
  float heading = atan2(my_avg, mx_avg) * 180.0 / PI;
  
  if (heading < 0) heading += 360.0; // Normalize 0-360

  // 5. Send Data (13th Value is Heading)
  Serial.printf(
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
    "%.2f\n", // <--- Added Heading here
    ax1, ay1, az1, gx1, gy1, gz1,
    ax2, ay2, az2, gx2, gy2, gz2,
    heading
  );

  delay(8); 
}

// ---------------- HELPERS ----------------

void initIMU(uint8_t address) {
  writeRegister(address, PWR_MGMT_1, 0x00);
  delay(10);
  writeRegister(address, CONFIG, 0x01);      // DLPF ~184Hz
  writeRegister(address, ACCEL_CONFIG, 0x00); // ±2g
  writeRegister(address, GYRO_CONFIG, 0x00);  // ±250 dps
}

void enableBypass(uint8_t address, bool enable) {
  uint8_t val = enable ? 0x02 : 0x00; // Bit 1 is BYPASS_EN
  writeRegister(address, INT_PIN_CFG, val);
}

void initMagMultiplexed() {
  // Init Mag 1
  enableBypass(IMU1_ADDR, true);
  enableBypass(IMU2_ADDR, false); // Block IMU2
  delay(10);
  writeRegister(MAG_ADDR, MAG_CNTL1, 0x16); // Continuous mode 2 (100Hz), 16-bit
  
  // Init Mag 2
  enableBypass(IMU1_ADDR, false); // Block IMU1
  enableBypass(IMU2_ADDR, true);
  delay(10);
  writeRegister(MAG_ADDR, MAG_CNTL1, 0x16);

  // Close both gates
  enableBypass(IMU2_ADDR, false);
}

void readMagMultiplexed(float &mx1, float &my1, float &mz1,
                        float &mx2, float &my2, float &mz2) {
  uint8_t raw[7];

  // --- READ MAG 1 ---
  enableBypass(IMU1_ADDR, true);
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)7);
  if(Wire.available() == 7) {
    for(int i=0; i<7; i++) raw[i] = Wire.read();
    // Mag is Little Endian!
    int16_t rmx = raw[1] << 8 | raw[0];
    int16_t rmy = raw[3] << 8 | raw[2];
    int16_t rmz = raw[5] << 8 | raw[4];
    mx1 = (float)rmx * MAG_SCALE;
    my1 = (float)rmy * MAG_SCALE;
    mz1 = (float)rmz * MAG_SCALE;
  }
  enableBypass(IMU1_ADDR, false);

  // --- READ MAG 2 ---
  enableBypass(IMU2_ADDR, true);
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)7);
  if(Wire.available() == 7) {
    for(int i=0; i<7; i++) raw[i] = Wire.read();
    int16_t rmx = raw[1] << 8 | raw[0];
    int16_t rmy = raw[3] << 8 | raw[2];
    int16_t rmz = raw[5] << 8 | raw[4];
    mx2 = (float)rmx * MAG_SCALE;
    my2 = (float)rmy * MAG_SCALE;
    mz2 = (float)rmz * MAG_SCALE;
  }
  enableBypass(IMU2_ADDR, false);
}

void readAccelGyro(uint8_t address,
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
    Wire.read(); Wire.read(); 
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