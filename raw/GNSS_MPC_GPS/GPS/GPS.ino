#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial gpsSerial(3, 4); // RX, TX
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(115200);  // Here4 default baud
  Serial.println("Here4 test");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Lon: ");
    Serial.println(gps.location.lng(), 6);
  }
  Serial.println(gps.location.isUpdated());
  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Lon: ");
    Serial.println(gps.location.lng(), 6);
  }
}
