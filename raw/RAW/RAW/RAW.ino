#include <Wire.h>

// --- CONFIGURATION ---
#define IMU1_ADDR 0x68
#define IMU2_ADDR 0x69
#define SDA_PIN 21
#define SCL_PIN 22
#define FSYNC_PIN 4  

// Scales
#define ACC_SCALE 16384.0   
#define GYRO_SCALE 131.0    
#define G_FORCE 9.80665     

// Storage for Offsets
float ax1_offset=0, ay1_offset=0, az1_offset=0, gx1_offset=0, gy1_offset=0, gz1_offset=0;
float ax2_offset=0, ay2_offset=0, az2_offset=0, gx2_offset=0, gy2_offset=0, gz2_offset=0;

void setup() {
  Serial.begin(115200);
  pinMode(FSYNC_PIN, OUTPUT); digitalWrite(FSYNC_PIN, LOW);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); 

  initIMU(IMU1_ADDR);
  initIMU(IMU2_ADDR);

  Serial.println("Keep sensor STILL for 3 seconds to Calibrate...");
  delay(1000);
  calibrateSensors(); // CALCULATE THE "ZERO" ERROR
  Serial.println("Calibration Done. Streaming corrected data...");
}

void loop() {
  // SYNC
  digitalWrite(FSYNC_PIN, HIGH); delayMicroseconds(50); digitalWrite(FSYNC_PIN, LOW);
  delay(2); 

  // READ CORRECTED DATA
  readAndPrint(IMU1_ADDR, "ACC1", "GYR1", 1);
  readAndPrint(IMU2_ADDR, "ACC2", "GYR2", 2);

  delay(8);
}

// -------------------------------------------------------------------------

void calibrateSensors() {
  long i1[6] = {0}, i2[6] = {0}; // Accumulators
  int samples = 200;

  for(int i=0; i<samples; i++) {
    int16_t b[6]; 
    readRaw(IMU1_ADDR, b); i1[0]+=b[0]; i1[1]+=b[1]; i1[2]+=b[2]; i1[3]+=b[3]; i1[4]+=b[4]; i1[5]+=b[5];
    readRaw(IMU2_ADDR, b); i2[0]+=b[0]; i2[1]+=b[1]; i2[2]+=b[2]; i2[3]+=b[3]; i2[4]+=b[4]; i2[5]+=b[5];
    delay(5);
  }

  // Average the noise
  ax1_offset = i1[0]/(float)samples; ay1_offset = i1[1]/(float)samples; az1_offset = i1[2]/(float)samples;
  gx1_offset = i1[3]/(float)samples; gy1_offset = i1[4]/(float)samples; gz1_offset = i1[5]/(float)samples;

  ax2_offset = i2[0]/(float)samples; ay2_offset = i2[1]/(float)samples; az2_offset = i2[2]/(float)samples;
  gx2_offset = i2[3]/(float)samples; gy2_offset = i2[4]/(float)samples; gz2_offset = i2[5]/(float)samples;

  // For Z-Axis, we EXPECT 1G (16384), so we don't subtract it all, we subtract the DIFFERENCE
  // Actually, simplest is to just remove bias from Gyro and Accel X/Y. 
  // We leave Accel Z offset alone usually, but here we will just zero-center everything 
  // and handle gravity in the driver? No, let's just correct Gyro for now as it's the biggest issue.
  
  // Correction: We ONLY remove offset from GYRO. 
  // Removing Accel offset is dangerous if the table isn't perfectly flat (it "bakes in" the tilt).
  // Let's stick to GYRO calibration (Stop the spinning).
  ax1_offset=0; ay1_offset=0; az1_offset=0;
  ax2_offset=0; ay2_offset=0; az2_offset=0;
}

void readRaw(uint8_t address, int16_t* data) {
  Wire.beginTransmission(address); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)14);
  if (Wire.available()==14) {
    data[0] = Wire.read()<<8|Wire.read(); data[1] = Wire.read()<<8|Wire.read(); data[2] = Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read(); // Skip Temp
    data[3] = Wire.read()<<8|Wire.read(); data[4] = Wire.read()<<8|Wire.read(); data[5] = Wire.read()<<8|Wire.read();
  }
}

void readAndPrint(uint8_t address, String ah, String gh, int id) {
  int16_t d[6];
  readRaw(address, d);

  float ax, ay, az, gx, gy, gz;
  
  // Apply Offsets
  if(id==1) {
    ax = d[0]-ax1_offset; ay = d[1]-ay1_offset; az = d[2]-az1_offset;
    gx = d[3]-gx1_offset; gy = d[4]-gy1_offset; gz = d[5]-gz1_offset;
  } else {
    ax = d[0]-ax2_offset; ay = d[1]-ay2_offset; az = d[2]-az2_offset;
    gx = d[3]-gx2_offset; gy = d[4]-gy2_offset; gz = d[5]-gz2_offset;
  }

  Serial.print(ah); Serial.print(": ");
  Serial.print(ax / ACC_SCALE * G_FORCE, 3); Serial.print(" ");
  Serial.print(ay / ACC_SCALE * G_FORCE, 3); Serial.print(" ");
  Serial.println(az / ACC_SCALE * G_FORCE, 3);

  Serial.print(gh); Serial.print(": ");
  Serial.print(gx / GYRO_SCALE, 3); Serial.print(" ");
  Serial.print(gy / GYRO_SCALE, 3); Serial.print(" ");
  Serial.println(gz / GYRO_SCALE, 3);
}

void initIMU(uint8_t address) {
  writeReg(address, 0x6B, 0x00); delay(10);
  writeReg(address, 0x1A, 0x08); 
  writeReg(address, 0x1C, 0x00); 
  writeReg(address, 0x1B, 0x00); 
}

void writeReg(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(data); Wire.endTransmission();
}
