// -------------------------------------------------
// Copyright (c) 2022 HiBit <https://www.hibit.dev>
// Modified for 100+ Hz operation
// -------------------------------------------------

#include "Wire.h"
#include "I2C.h"

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

// Changed to 10ms for 100 Hz
#define INTERVAL_MS_PRINT 10

#define G 9.80665

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

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // Set I2C to 400kHz (Fast Mode)
  Serial.begin(115200);

  // Configure gyroscope range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  
  // Configure accelerometer range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);

  // Set sample rate divider for faster IMU readings
  // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
  // For 1kHz gyro rate, SMPLRT_DIV = 0 gives 1kHz, = 9 gives 100Hz
  I2CwriteByte(MPU9250_IMU_ADDRESS, 25, 0); // SMPLRT_DIV = 0 for max rate

  // Configure DLPF (Digital Low Pass Filter) for bandwidth
  // DLPF_CFG = 1: Gyro 184Hz, Accel 184Hz, Temp 188Hz, Delay 2.9ms
  I2CwriteByte(MPU9250_IMU_ADDRESS, 26, 0x01);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02);
  
  // Disable interrupt - we'll poll instead for faster operation
  I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x00);

  setMagnetometerAdjustmentValues();

  // Start magnetometer - 100Hz continuous mode
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x16); // 0x16 = 100Hz continuous in 16-bit
}

void loop()
{
  unsigned long currentMillis = millis();

  // Read IMU data directly without waiting for ready flag
  readRawImu();
  normalize(gyroscope);
  normalize(accelerometer);
  normalize(temperature);

  // Read magnetometer less frequently (it's slower anyway)
  // Check every 10th iteration or so
  static uint8_t magCounter = 0;
  if (++magCounter >= 10) {
    if (isMagnetometerReady()) {
      readRawMagnetometer();
      normalize(magnetometer);
    }
    magCounter = 0;
  }

  // Print at defined interval
  if (currentMillis - lastPrintMillis >= INTERVAL_MS_PRINT) {
    Serial.print("TEMP:\t");
    Serial.print(normalized.temperature, 2);
    Serial.print("\xC2\xB0");
    Serial.print("C");
    Serial.println();

    Serial.print("GYR (");
    Serial.print("\xC2\xB0");
    Serial.print("/s):\t");
    Serial.print(normalized.gyroscope.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.z, 3);
    Serial.println();

    Serial.print("ACC (m/s^2):\t");
    Serial.print(normalized.accelerometer.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.z, 3);
    Serial.println();

    Serial.print("MAG (");
    Serial.print("\xce\xbc");
    Serial.print("T):\t");
    Serial.print(normalized.magnetometer.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.z, 3);
    Serial.println();

    Serial.println();

    lastPrintMillis = currentMillis;
  }
}

void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void I2Cread(uint8_t address, uint8_t subAddress, uint8_t nBytes, uint8_t* data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);      // Send restart
  Wire.requestFrom(address, nBytes); // Request nBytes from slave
  
  uint8_t i = 0;
  while (Wire.available()) {
    data[i++] = Wire.read();        // Read bytes into array
  }
}