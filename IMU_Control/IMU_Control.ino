#include <Arduino_LSM9DS1.h>

// Magnetometer range is set at [-400, +400] uT +/-0.014 uT.

int myPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
float x, y, z;
float cal_x, cal_y, cal_z;
float u, v, w;

float theta;
float mags;
float thresh = 0.5;
int heading;

String str;

void setup() {
  Serial.begin(9600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 
}

void loop() {
  IMU.readMagneticField(x, y, z); // Read IMU DATA
  // cal_x = x - 25.63;`
  // cal_y = y - 8.78;
  // cal_z = z - (-28.82);
  // float h = (atan2(cal_y, cal_x) * 180) / PI;
  // theta = atan2(z, -x);           // Calculate Heading Angle 
  // Serial.println(h);
  Serial.print(String(x) + ", ");
  Serial.print(String(y) + ", ");
  Serial.println(String(z));
  mags = sqrt(pow(x,2)+pow(y,2)); // Calculate Magnitude 

  // if(mags > thresh)
  // {
  //   heading = map(theta / 3.14, -1.125, 1.125, 0, 8);
  // } else {
  //   heading = - 1;
  // }

  // if (heading == 8) heading = 0;

  
  // for (int i = 0; i <= 7; i++) 
  // {
  //   if (i == heading)
  //   {
  //     digitalWrite(myPins[i], HIGH);
  //   } else {
  //     digitalWrite(myPins[i], LOW);
  //   }
  // }

  //str = String(x) + ","+String(y)+String(z);
  //Serial.println(str);
}
