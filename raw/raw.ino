#include <Wire.h>
// #include "I2C.h" // REMOVED: This library is not needed as Wire is used below

#define MPU9250_IMU_ADDRESS 0x68
#define MPU9250_MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18

#define TEMPERATURE_OFFSET 21
#define INTERVAL_MS_PRINT 10
#define G 9.80665

// --- Global Structs ---
struct gyroscope_raw {
  int16_t x, y, z;
} gyroscope;

struct accelerometer_raw {
  int16_t x, y, z;
} accelerometer;

struct magnetometer_raw {
  int16_t x, y, z;
  struct {
    int8_t x, y, z;
  } adjustment;
} magnetometer;

struct temperature_raw {
  int16_t value;
} temperature;

struct {
  struct {
    float x, y, z;
  } accelerometer, gyroscope, magnetometer;
  float temperature;
} normalized;

unsigned long lastPrintMillis = 0;

// --- Function Prototypes ---
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
void I2Cread(uint8_t address, uint8_t subAddress, uint8_t nBytes, uint8_t* data);
void readRawImu();
void readRawMagnetometer();
bool isMagnetometerReady();
void setMagnetometerAdjustmentValues();
void normalize(gyroscope_raw& g);
void normalize(accelerometer_raw& a);
void normalize(magnetometer_raw& m);
void normalize(temperature_raw& t);

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  Serial.begin(115200);

  // Configure gyroscope range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  
  // Configure accelerometer range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);

  // Set sample rate divider
  I2CwriteByte(MPU9250_IMU_ADDRESS, 25, 0); 

  // Configure DLPF
  I2CwriteByte(MPU9250_IMU_ADDRESS, 26, 0x01);

  // Enable Bypass Mode for Magnetometer access
  I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02);
  
  // Disable interrupt
  I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x00);

  // Get Magnetometer factory sensitivity adjustment values
  setMagnetometerAdjustmentValues();

  // Start magnetometer - 100Hz continuous mode, 16-bit
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x16); 
}

void loop()
{
  unsigned long currentMillis = millis();

  readRawImu();
  normalize(gyroscope);
  normalize(accelerometer);
  normalize(temperature);

  static uint8_t magCounter = 0;
  if (++magCounter >= 10) { // Check mag every ~100ms (since loop is fast)
    if (isMagnetometerReady()) {
      readRawMagnetometer();
      normalize(magnetometer);
    }
    magCounter = 0;
  }

  if (currentMillis - lastPrintMillis >= INTERVAL_MS_PRINT) {
    Serial.print("TEMP:\t");
    Serial.print(normalized.temperature, 2);
    Serial.println(" C");

    Serial.print("GYR:\t");
    Serial.print(normalized.gyroscope.x, 3); Serial.print("\t");
    Serial.print(normalized.gyroscope.y, 3); Serial.print("\t");
    Serial.print(normalized.gyroscope.z, 3);
    Serial.println();

    Serial.print("ACC:\t");
    Serial.print(normalized.accelerometer.x, 3); Serial.print("\t");
    Serial.print(normalized.accelerometer.y, 3); Serial.print("\t");
    Serial.print(normalized.accelerometer.z, 3);
    Serial.println();

    Serial.print("MAG:\t");
    Serial.print(normalized.magnetometer.x, 3); Serial.print("\t");
    Serial.print(normalized.magnetometer.y, 3); Serial.print("\t");
    Serial.print(normalized.magnetometer.z, 3);
    Serial.println();
    Serial.println("-----------------------------");

    lastPrintMillis = currentMillis;
  }
}

// --- Logic Implementations ---

void readRawImu() {
  uint8_t buffer[14];
  // Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
  I2Cread(MPU9250_IMU_ADDRESS, 0x3B, 14, buffer);

  accelerometer.x = (int16_t)((buffer[0] << 8) | buffer[1]);
  accelerometer.y = (int16_t)((buffer[2] << 8) | buffer[3]);
  accelerometer.z = (int16_t)((buffer[4] << 8) | buffer[5]);

  temperature.value = (int16_t)((buffer[6] << 8) | buffer[7]);

  gyroscope.x = (int16_t)((buffer[8] << 8) | buffer[9]);
  gyroscope.y = (int16_t)((buffer[10] << 8) | buffer[11]);
  gyroscope.z = (int16_t)((buffer[12] << 8) | buffer[13]);
}

bool isMagnetometerReady() {
  uint8_t st1;
  I2Cread(MPU9250_MAG_ADDRESS, 0x02, 1, &st1);
  return (st1 & 0x01); // Check DRDY bit
}

void readRawMagnetometer() {
  uint8_t buffer[7];
  // Read 7 bytes starting from HXL (0x03) to ST2 (0x09)
  I2Cread(MPU9250_MAG_ADDRESS, 0x03, 7, buffer);

  // Check if magnetic sensor overflow set, if not then report data
  if (!(buffer[6] & 0x08)) { 
    // Mag data is Little Endian
    magnetometer.x = (int16_t)((buffer[1] << 8) | buffer[0]);
    magnetometer.y = (int16_t)((buffer[3] << 8) | buffer[2]);
    magnetometer.z = (int16_t)((buffer[5] << 8) | buffer[4]);
  }
}

void setMagnetometerAdjustmentValues() {
  uint8_t buffer[3];
  
  // Set Fuse ROM access mode
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x1F);
  delay(10);
  
  // Read ASAX, ASAY, ASAZ
  I2Cread(MPU9250_MAG_ADDRESS, 0x10, 3, buffer);
  
  magnetometer.adjustment.x = buffer[0];
  magnetometer.adjustment.y = buffer[1];
  magnetometer.adjustment.z = buffer[2];
  
  // Power down mode before switching back to continuous
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x00);
}

// --- Normalization Functions ---

void normalize(gyroscope_raw& g) {
  // Scale for 1000DPS: 32.8 LSB/(deg/s)
  normalized.gyroscope.x = g.x / 32.8;
  normalized.gyroscope.y = g.y / 32.8;
  normalized.gyroscope.z = g.z / 32.8;
}

void normalize(accelerometer_raw& a) {
  // Scale for 2G: 16384 LSB/g -> Output in m/s^2
  // If you want Gs, remove the * G
  float scale = 16384.0;
  normalized.accelerometer.x = (a.x / scale) * G;
  normalized.accelerometer.y = (a.y / scale) * G;
  normalized.accelerometer.z = (a.z / scale) * G;
}

void normalize(temperature_raw& t) {
  normalized.temperature = ((float)t.value) / 333.87 + 21.0;
}

void normalize(magnetometer_raw& m) {
  // Sensitivity: 0.15 uT/LSB (for 16-bit)
  // Adjustment formula: H_adj = H_raw * (((ASA-128)*0.5)/128 + 1)
  float scale = 0.15;
  
  float adjX = (((float)m.adjustment.x - 128.0) * 0.5) / 128.0 + 1.0;
  float adjY = (((float)m.adjustment.y - 128.0) * 0.5) / 128.0 + 1.0;
  float adjZ = (((float)m.adjustment.z - 128.0) * 0.5) / 128.0 + 1.0;

  normalized.magnetometer.x = m.x * scale * adjX;
  normalized.magnetometer.y = m.y * scale * adjY;
  normalized.magnetometer.z = m.z * scale * adjZ;
}

// --- I2C Helper Definitions (Correct for Wire.h) ---

void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void I2Cread(uint8_t address, uint8_t subAddress, uint8_t nBytes, uint8_t* data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false); // Restart
  Wire.requestFrom(address, nBytes);
  
  uint8_t i = 0;
  while (Wire.available() && i < nBytes) {
    data[i++] = Wire.read();
  }
}