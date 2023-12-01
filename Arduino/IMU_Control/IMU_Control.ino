#include <Arduino_LSM9DS1.h>
float x,y,z;
String str;

void setup() {
  Serial.begin(9600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 
}

void loop() {
  IMU.readMagneticField(x, y, z); // Read IMU DATA
  str = String(x) + ","+String(y)+String(z);
  Serial.println(str);
}
