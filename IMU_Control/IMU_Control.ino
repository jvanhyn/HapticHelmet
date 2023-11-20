#include <Arduino_LSM9DS1.h>
// Magnetometer range is set at [-400, +400] uT +/-0.014 uT.

int myPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
float x, y, z;
float u, v, w;

float theta;
float mags;
float thresh = 0.5;
int heading;

void setup() {
  Serial.begin(98600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 

  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT); // Initiate Digital Output Pins 

  pinMode(A0, INPUT); 
  pinMode(A1, INPUT);
}

void loop() {
  IMU.readMagneticField(x, y, z); // Read IMU DATA

  theta = atan2(x, -y);           // Calculate Heading Angle 
  mags = sqrt(pow(x,2)+pow(y,2)); // Calculate Magnitude 

  if(mags > thresh)
  {
    heading = map(theta / 3.14, -1.125, 1.125, 0, 8);
  } else {
    heading = - 1;
  }

  if (heading == 8) heading = 0;

  
  for (int i = 0; i <= 7; i++) 
  {
    if (i == heading)
    {
      digitalWrite(myPins[i], HIGH);
    } else {
      digitalWrite(myPins[i], LOW);
    }
  }

}
