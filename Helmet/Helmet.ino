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



/*
VARIABLES READ FROM UNITY
*/
// UNITY PARSING VARS
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
bool newData = false;


// Read Values from Unity 
float dist = 1000; // Distance from player to next waypoint
int dir = 0;	// Turn direction at waypoint
int unity_heading = 0; // Player in game heading
float old_dist = 1000;


// // Virtual Environment
// int num_checkpoints;
// float checkpoint_ds[] = {200, 600, 200, 300, 400, 300, 200, 100};

/*
Motor Control Variables
*/
bool state[] = {false,false,false,false,false,false,false,false};
static float threshold = 1;  // Change this for how close user has to be
float buzz_freq = 100;    // buzz frequency at standard distance
float buzz_dir;
float std_d = 10;      // standard distance is 10

/*
  Main control loop logic vars
*/
bool new_check = true;
float buzz_thresh;



/*
 FUNCTIONS TO DEAL WITH THE ACTUAL BUZZING OF THE ARDUINO


POINT uses the global buzz_dir angle and converts that to an a quadrant on the helmet to buzz and then makes it buzz
*/
void point() {
  //  Serial.println(buzz_dir);
  int motorNumber = int(floor(buzz_dir/360 * 8));
  if (motorNumber == 8) {
    motorNumber = 0;
  }
  for (int i = 0; i < 8; i++) {
    state[i] = false;
  }
  state[motorNumber] = true;
//   for (int i = 0; i < 8; i++) {
//     Serial.print(state[i]);
//     Serial.print(", ");
//   }
//  Serial.println("");
  
}


// Tells motors to vibrate
void updateVibration(){
  for (int i = 0; i < 8; i++) {
    if (state[i]) {
      digitalWrite(myPins[i], HIGH);
    } else {
      digitalWrite(myPins[i], LOW);
    }
  }
}


// Obtain the angle with respect to the physical headset for the motors to buzz based on the direction of the next checkpoint and where the user is looking
void buzzDir() {
 // get the desired direction of turn from the next checkpoint (0 is straight and positive is anti-clockwise)
  float imu_head = theta;   // imu heading
//  if (imu_head < 0) {
//    // heading is from 0 straight ahead and positive left but negative right toward +/- 180
//    // we want 0 to 360 degree heading so change the right hand side from 0 to -180 to 360 to 180
//    imu_head = 360 + imu_head;
//  }
  // float buzz_dir_360 = theta;
  float buzz_dir_360 = 90 - dir - unity_heading - imu_head;  // account for the actual heading of the user (where they are looking) to buzz the correct angle for turn
  buzz_dir_360 = ((int) buzz_dir_360) % 360;
  if (buzz_dir_360 < 0) {
    buzz_dir_360 = 360 + buzz_dir_360;
  }
 
  buzz_dir = buzz_dir_360;
//  if (buzz_dir > 180) {
//    buzz_dir = buzz_dir - 360;
//  }
//  Serial.println(buzz_dir);
}


void buzz(int dt, int n) {
 for (int i = 0; i < n; i++) {
   updateVibration();
   delay(dt);
   off();
   delay(dt);
 }
//  off();
//  delay(100);
}


void on() {
 for (int i = 0; i <= 7; i++) {
   digitalWrite(myPins[i], HIGH);
 }
}


void off() {
 for (int i = 0; i <= 7; i++) {
   digitalWrite(myPins[i], LOW);
  //  state[i] = false;
 }
}







/*
	This Code block deals with reading IMU DATA


*/
void zeroIMU() // Updates IMU variables, deltat, q, ee, and theta
{ 
 
  Serial.println("zeroing the IMU...");
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
 Serial.println(zero);
 Serial.println("Successfully zeroed the IMU!");
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
 theta = ((int)theta) % 360;
 if (theta < 0) {
  theta = 360 + theta;
 }
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







/*
	This Code block deals with grabbing and parsing data from Unity
*/


void recvWithStartEndMarkers() {
   static boolean recvInProgress = false;
   static byte ndx = 0;
   char startMarker = '(';
   char endMarker = ')';
   char rc;


   while (Serial.available() > 0 && newData == false) {
       rc = Serial.read();


       if (recvInProgress == true) {
           if (rc != endMarker) {
               receivedChars[ndx] = rc;
               ndx++;
               if (ndx >= numChars) {
                   ndx = numChars - 1;
               }
           }
           else {
               receivedChars[ndx] = '\0'; // terminate the string
               recvInProgress = false;
               ndx = 0;
               newData = true;
           }
       }


       else if (rc == startMarker) {
           recvInProgress = true;
       }
   }
}


//============


void parseData() {      // split the data into its parts


   char * strtokIndx; // this is used by strtok() as an index


   strtokIndx = strtok(tempChars,",");      // get the first part - the string
   dist = atof(strtokIndx);     // convert this part to a float

   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   dir = atof(strtokIndx);     // convert this part to a float


   strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
   unity_heading = atof(strtokIndx);     // convert this part to a float


}


// Just prints the data that we parsed from unity
void showParsedData() {
   Serial.print("Dist: ");
   Serial.print(dist);
   Serial.print("  ");
   Serial.print("Dir: ");
   Serial.print(dir);
   Serial.print("  ");
   Serial.print("User Angle: ");
   Serial.print(dir);
}



/*
	This code block is for the main loop
*/


void setup() {
  // Serial.begin(98600); // Start Serial Communitcation
  Serial.begin(9600);
  IMU.begin(); // Start IMU 
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(12, OUTPUT);

  // pinMode(3, OUTPUT);

  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT); // Initiate Digital Output Pins 

  // Set the number and values of each checkpoint and direction for the user to follow
  // num_checkpoints = 8;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  delay(1000);
  // Serial.println("Zeroing");
  for (int i = 0; i < 70; i++) {
    zeroIMU();
  }

}

void loop() {

  // update user pos
  recvWithStartEndMarkers();
  if (newData == true) {
  
    strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
    parseData();
  //  Serial.println("received!");
    // Serial.print("Data Recived - ");
    // showParsedData();
    newData = false;
  }
  if (abs(old_dist - dist) > 40) {
    buzz_thresh = dist;
  }
  old_dist = dist;

  //Grab IMU data
  updateIMU();
  // Serial.println(theta);
  
    
  //  digitalWrite(3, HIGH);
  //  delay(200);
  //  digitalWrite(3, LOW);
  //  delay(200);
  
  
  // for (int i = 0; i < 8; i++) {
  //   // state[i] = true;
  //   // buzz(100, 1);
  //   Serial.print(state[i]);
  //   Serial.print(", ");
  // }
  // Serial.println("");

  // Serial.println(buzz_dir);
  // Serial.println(state);
  // buzz(100, 3);

  //  Serial.println(theta);

   if (dist <= buzz_thresh) {   // if the user is within a distance threshold, buzz the directions again
     buzz_thresh = buzz_thresh / 2;    // lower the threshold so that the user is not constantly buzzed with directions
     buzzDir(); // get the correct direction on the helmet to buzz to convey direction instruction
     point();
    //  buzz((buzz_freq / (std_d / dist)), 3);   // actually buzz the helmet 3 times with the correct direction and a frequency proportional to the distance to the checkpoint
    int freq = (int)(buzz_freq / (std_d / dist));
     buzz(freq, 3);
   }


}