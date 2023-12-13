#include <Arduino_LSM9DS1.h>

/*
VARIABLES FOR BOARD SETUP
*/
// Pins corresponding to motors in the headband
const int myPins[] = { 2, 3, 4, 5, 6, 9,10,11 };

/*
VARIABLES FOR READING IMU DATA 
*/
// Calibration Constants
const float xcal = 8.2400;
const float ycal = 7.8950;
const float zcal = 8.4100;

// Raw IMU DATA
static float ax,ay,az,mx,my,mz,gx,gy,gz;

/*
VARIABLES FOR FILTERING IMU DATA 
*/
#define Kp 50.0
#define Ki 0.0

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};

// vector to hold Euler angles
static float ee[3] = {0,0,0};

// Time Keeping
static float deltat = 0;  //loop time in seconds
unsigned long now = 0, last = 0; //micros() timers for AHRS loop

// True Compass Heading
static double theta;
static double zero;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.begin(); // Start IMU 
  

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  delay(2000);
  // Serial.println("Zeroing");
  for (int i = 0; i < 70; i++) {
    zeroIMU();
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  updateIMU();
  // Serial.println(theta);
}


/*
	This Code block deals with reading IMU DATA


*/

void zeroIMU() // Updates IMU variables, deltat, q, ee, and theta
{ 
 
  // Serial.println("zeroing the IMU...");
 if (IMU.gyroscopeAvailable()) {
   IMU.readGyroscope(gx, gy, gz);
 }
 if (IMU.accelerationAvailable()) {
   IMU.readAcceleration(ax, ay, az);
 }
 IMU.readMagneticField(mx, my, mz);
 mx = mx-xcal;
 my = my-ycal;
 mz = mz-zcal;
 now = micros();
 deltat = (now - last) * 1.0e-6; //seconds since last update
 last = now; 
MahonyQuaternionUpdate(-ax,ay,az,-gx/180*PI,gy/180*PI,gz/180*PI,mx,my,mz,deltat);
 
 quat2eul();
 zero = ee[2] * 180 / PI;
 if (zero < 0) {
  zero = 360 + zero;
 }
//  Serial.println(zero);
//  Serial.println("Successfully zeroed the IMU!");
}

void updateIMU() // Updates IMU variables, deltat, q, ee, and theta
{ 
 if (IMU.gyroscopeAvailable()) {
   IMU.readGyroscope(gx, gy, gz);
 }
 if (IMU.accelerationAvailable()) {
   IMU.readAcceleration(ax, ay, az);
 }
 IMU.readMagneticField(mx, my, mz);
 mx = mx-xcal;
 my = my-ycal;
 mz = mz-zcal;
 now = micros();
 deltat = (now - last) * 1.0e-6; //seconds since last update
 last = now; 
MahonyQuaternionUpdate(-ax,ay,az,-gx/180*PI,gy/180*PI,gz/180*PI,mx,my,mz,deltat);
 
 quat2eul();
 theta = ee[2] * 180 / PI;
 
 theta = theta - zero;
 
 if (theta < 0) {
  theta = 360 + theta;
 }
// Serial.println(theta);
//  Serial.println(zero);
}


void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) // Updates q
{
 // Vector to hold integral error for Mahony method
 static float eInt[3] = {0.0, 0.0, 0.0};
   // short name local variable for readability
 float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
 float norm;
 float hx, hy, hz;  //observed West horizon vector W = AxM
 float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
 float ex, ey, ez;
 float pa, pb, pc;


 // Auxiliary variables to avoid repeated arithmetic
 float q1q1 = q1 * q1;
 float q1q2 = q1 * q2;
 float q1q3 = q1 * q3;
 float q1q4 = q1 * q4;
 float q2q2 = q2 * q2;
 float q2q3 = q2 * q3;
 float q2q4 = q2 * q4;
 float q3q3 = q3 * q3;
 float q3q4 = q3 * q4;
 float q4q4 = q4 * q4;


 // Measured horizon vector = a x m (in body frame)
 hx = ay * mz - az * my;
 hy = az * mx - ax * mz;
 hz = ax * my - ay * mx;
 // Normalise horizon vector
 norm = sqrt(hx * hx + hy * hy + hz * hz);
 if (norm == 0.0f) return; // Handle div by zero


 norm = 1.0f / norm;
 hx *= norm;
 hy *= norm;
 hz *= norm;


 // Estimated direction of Up reference vector
 ux = 2.0f * (q2q4 - q1q3);
 uy = 2.0f * (q1q2 + q3q4);
 uz = q1q1 - q2q2 - q3q3 + q4q4;


 // estimated direction of horizon (West) reference vector
 wx = 2.0f * (q2q3 + q1q4);
 wy = q1q1 - q2q2 + q3q3 - q4q4;
 wz = 2.0f * (q3q4 - q1q2);


 // Error is the summed cross products of estimated and measured directions of the reference vectors
 // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.


 ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
 ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
 ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
 if (Ki > 0.0f)
 {
   eInt[0] += ex;      // accumulate integral error
   eInt[1] += ey;
   eInt[2] += ez;
   // Apply I feedback
   gx += Ki * eInt[0];
   gy += Ki * eInt[1];
   gz += Ki * eInt[2];
 }


 // Apply P feedback
 gx = gx + Kp * ex;
 gy = gy + Kp * ey;
 gz = gz + Kp * ez;


//update quaternion with integrated contribution
// small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);


 // Normalise quaternion
 norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
 norm = 1.0f / norm;
 q[0] = q1 * norm;
 q[1] = q2 * norm;
 q[2] = q3 * norm;
 q[3] = q4 * norm;


}

void quat2eul() // Updates ee
{
   ee[0] = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
   ee[1] = asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
   ee[2] = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}
