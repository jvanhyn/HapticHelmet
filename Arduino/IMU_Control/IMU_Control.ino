#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
//#include <MPUOrientation.h>

Madgwick filter;

float ax,ay,az,mx,my,mz,gx,gy,gz;
const float sensorRate = 104.00;
int i = 1;
float roll,pitch;
String str;

void setup() {
  Serial.begin(9600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 
  filter.begin(sensorRate);
}

void loop() {

  IMU.readAcceleration(ax, ay, az);
  IMU.readMagneticField(mx, my, mz);
  IMU.readGyroscope(gz,gy,gz);
  
  // filter.updateIMU(gx,gy,gz,ax,ay,az);

  // roll = filter.getRoll();
  // pitch = filter.getPitch();


  str = String(-ay)+","+String(ax)+","+String(az)+","+String(-gy)+","+String(gx)+","+String(gz)+","+ String(mz)+","+String(mx)+","+String(my);
  Serial.println(str);
  // mx = mx-cal_x;
  // my = my-cal_y;
  // mz = mz-cal_z;

  // q = ax;
  // r = ay;
  // s = az;

  // // fusion.update(gx,gy,gz,ax,ay,az); 

  // // Serial.print( fusion.pitch() );
  // // Serial.print( " " );
  // // Serial.print( fusion.yaw() );
  // // Serial.print( " " );
  // // Serial.print( fusion.roll() );
  // // Serial.println();

  // alpha = atan2(r,q);

  // u = q*cos(-alpha)-r*sin(-alpha);
  // v = q*sin(-alpha)+r*cos(-alpha);
  // w = s;

  // beta = atan2(w,u);

  // x = u*cos(-beta)-w*sin(-beta);
  // y = v;
  // z = u*sin(-beta)+w*cos(-beta);

  // q = mx;
  // r = my;
  // s = mz;

  // u = q*cos(-alpha)-r*sin(-alpha);
  // v = q*sin(-alpha)+r*cos(-alpha);
  // w = s;

  // x = u*cos(-beta)-w*sin(-beta);
  // y = v;
  // z = u*sin(-beta)+w*cos(-beta);

  // theta = atan2(y,x);
  // alpha = alpha/PI*180;
  // beta = beta/PI*180;
  // theta = theta/PI*180;

  // str = String(alpha)+","+String(beta)+","+String(theta);
  // Serial.println(str);
 
}


