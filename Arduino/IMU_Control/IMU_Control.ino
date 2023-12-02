#include <Arduino_LSM9DS1.h>
float mx,my,mz;
float ax,ay,az;

float mx_cal,my_cal,mz_cal;
float ax_cal,ay_cal,az_cal;
float ALPHA,BETA,GAMMA;


String str;

void setup() {
  Serial.begin(9600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 
}

void loop() {
  IMU.readMagneticField(mx, my, mz); // Read IMU DATA
  IMU.readAcceleration(ax, ay, az); // Read IMU DATA

  mx_cal = mx - 0;
  my_cal = my - 0;
  mz_cal = mz - 0;

  ax_cal = ax - 0;
  ay_cal = ay - 0;
  az_cal = az - 0;

  ALPHA = atan2(ax,ay)
  BETA = atan2(ax,az)
  GAMMA = atan2()


 
}
