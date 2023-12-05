#include <Arduino_LSM9DS1.h>
float mx,my,mz;
float ax,ay,az;
float x,y,z;
float cal_mx,cal_my,cal_mz;
float u,v,w;

float alpha,beta;
String str;
String str2;
String str3;


void setup() {
  Serial.begin(9600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 
  x = 0;
  y = 0;
  z = 0;
}

void loop() {
  //IMU.readMagneticField(mx, my, mz); // Read IMU DATA

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  
  // cal_mx = mx - (25.63);
  // cal_my = my - (8.78);
  // cal_mz = mz - (-28.82);

  e1Angle(ax,ay,az);
  //str = String(cal_mx)+","+String(cal_my)+","+String(cal_mz);
  // str2 = String(ax)+","+String(ay)+","+String(az);
  // Serial.println(str2);

  str3 = String(u)+","+String(v)+","+String(w);
  Serial.println(str3);
 
}


void e1Angle(float x,float y,float z){
alpha = atan2(y,x);
u = x*cos(-alpha)-y*sin(-alpha);
v = x*sin(-alpha)+y*cos(-alpha);
w = z;
beta = atan2(w,u);
u = u*cos(-beta)-w*sin(-beta);
v = v;
w = u*sin(-beta)+w*cos(-beta);
}

// float rotate(float x, float y,float z,float alpha,float beta){
// int u1 = x*cos(-alpha)-y*sin(-alpha);
// int v1 = x*sin(-alpha)+y*cos(-alpha);
// int w1 = z;
// u = u1*cos(-beta)-w1*sin(-beta);
// v = v1;
// w = u1*sin(-beta)+w1*cos(-beta);
// }


