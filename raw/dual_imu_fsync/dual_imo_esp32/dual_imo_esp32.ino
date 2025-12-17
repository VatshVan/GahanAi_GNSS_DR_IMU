// #include <Wire.h>

// // --- HARDWARE CONFIG ---
// #define IMU1_ADDR 0x68
// #define IMU2_ADDR 0x69
// #define SDA_PIN 21
// #define SCL_PIN 22
// #define FSYNC_PIN 4  // The Pin connected to FSYNC on both IMUs

// // MPU9250 Registers
// #define PWR_MGMT_1    0x6B
// #define CONFIG        0x1A
// #define ACCEL_CONFIG  0x1C
// #define GYRO_CONFIG   0x1B
// #define ACCEL_XOUT_H  0x3B

// // Scales
// const float ACC_SCALE = 16384.0;
// const float GYRO_SCALE = 131.0; 

// void setup() {
//  Serial.begin(115200);
 
//  // 1. Setup Master Clock Pin
//  pinMode(FSYNC_PIN, OUTPUT);
//  digitalWrite(FSYNC_PIN, LOW); // Start Low

//  // 2. Start I2C
//    Wire.begin(SDA_PIN, SCL_PIN);
//  Wire.setClock(400000); 

//  // 3. Init Sensors with FSYNC Enabled
//  initIMU(IMU1_ADDR);
//  initIMU(IMU2_ADDR);

//  Serial.println("Dual IMU (Hardware Synced with FSYNC using PID waves) Started");
// }

// void loop() {
//  // --- STEP 1: THE MASTER TRIGGER ---
//  // We pulse the pin. Both IMUs capture "Now".
//  digitalWrite(FSYNC_PIN, HIGH);
//  delayMicroseconds(50); // Short pulse
//  digitalWrite(FSYNC_PIN, LOW);

//  // --- STEP 2: WAIT FOR DATA CONVERSION ---
//  // The IMU needs about 1ms to process the sampled data into the registers
//  delay(2); 

//  // --- STEP 3: READ SEQUENTIALLY ---
//  // Even though we read them one after another, the data represents 
//  // the exact moment of the FSYNC pulse we sent above.
//  printSensorData("ACC1", "GYR1", IMU1_ADDR);
//  printSensorData("ACC2", "GYR2", IMU2_ADDR);

//  // Loop Rate Control (Total delay ~10ms for 100Hz)
//  delay(8); 
// }

// void initIMU(uint8_t address) {
//  writeRegister(address, PWR_MGMT_1, 0x00); // Wake up
//  delay(10);

//  // --- CRITICAL: ENABLE FSYNC ---
//  // Register 0x1A (CONFIG)
//  // Bit 5-3: EXT_SYNC_SET. 
//  // Setting it to 1 (0x01 << 3 = 0x08) maps FSYNC pin to TEMP_OUT_L[0] bit.
//  // This effectively locks the internal sampling loop to the external pin.
//  // DLPF_CFG = 1 (bits 2-0) gives ~184Hz bandwidth
//  writeRegister(address, CONFIG, 0x08 | 0x01); 

//  writeRegister(address, ACCEL_CONFIG, 0x00); // 2g
//  writeRegister(address, GYRO_CONFIG, 0x00);  // 250dps
// }

// void printSensorData(String accHeader, String gyrHeader, uint8_t address) {
//  int16_t ax, ay, az, gx, gy, gz;

//  Wire.beginTransmission(address);
//  Wire.write(ACCEL_XOUT_H);
//  Wire.endTransmission(false);
//  Wire.requestFrom(address, (uint8_t)14);

//  if (Wire.available() == 14) {
//    ax = Wire.read() << 8 | Wire.read();
//    ay = Wire.read() << 8 | Wire.read();
//    az = Wire.read() << 8 | Wire.read();
//    int16_t temp = Wire.read() << 8 | Wire.read(); 
//    gx = Wire.read() << 8 | Wire.read();
//    gy = Wire.read() << 8 | Wire.read();
//    gz = Wire.read() << 8 | Wire.read();

//    Serial.print(accHeader); Serial.print(",");
//    Serial.print((float)ax / ACC_SCALE * 9.81, 3); Serial.print(",");
//    Serial.print((float)ay / ACC_SCALE * 9.81, 3); Serial.print(",");
//    Serial.println((float)az / ACC_SCALE * 9.81, 3);

//    // Format: "GYR,gx,gy,gz"
//    Serial.print(gyrHeader); Serial.print(",");
//    Serial.print((float)gx / GYRO_SCALE, 3); Serial.print(",");
//    Serial.print((float)gy / GYRO_SCALE, 3); Serial.print(",");
//    Serial.println((float)gz / GYRO_SCALE, 3);
//  }
// }

// void writeRegister(uint8_t address, uint8_t reg, uint8_t data) {
//  Wire.beginTransmission(address);
//  Wire.write(reg);
//  Wire.write(data);
//  Wire.endTransmission();
// }




















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
  Serial.print(ax1, 3); Serial.print(",");
  Serial.print(ay1, 3); Serial.print(",");
  Serial.print(az1, 3); Serial.print(",");
  Serial.print(gx1, 3); Serial.print(",");
  Serial.print(gy1, 3); Serial.print(",");
  Serial.print(gz1, 3); Serial.print(",");

  Serial.print(ax2, 3); Serial.print(",");
  Serial.print(ay2, 3); Serial.print(",");
  Serial.print(az2, 3); Serial.print(",");
  Serial.print(gx2, 3); Serial.print(",");
  Serial.print(gy2, 3); Serial.print(",");
  Serial.println(gz2, 3);

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
