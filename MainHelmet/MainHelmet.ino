#include <Arduino_LSM9DS1.h>
// Magnetometer range is set at [-400, +400] uT +/-0.014 uT.

int myPins[] = {5, 6, 7, 8, 9, 10, 11, 12}; // pins on arduino for controlling each motor


float dist = 0;
int dir = 0;
int unity_heading = 0;

float threshold = 1;  // Change this for how close user has to be
int num_checkpoints;
float buzz_freq = 1;    // buzz frequency at standard distance
float std_d = 100;      // standard distance is 100


// IMU DATA

// Number correponding to a motor at a specific location on the head
static double motorNumber = 0;

// Orientation Information
static double theta;


// Button press
bool buttonPress;

//Overides all actions if true
bool overide = 0;

int count = 0;
int debounce = 3;

int freq[] = {200,100,50,25};



float ax,ay,az,mx,my,mz,gx,gy,gz;
float mxcal,mycal,mzcal;
const float xcal = 8.2400;
const float ycal = 7.8950;
const float zcal = 8.4100;
String output;


#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// equler
static float ee[3] = {0,0,0};

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
//static float yaw, pitch, roll; //Euler angle output

void updateIMU() {
  if (IMU.gyroscopeAvailable()) {

    IMU.readGyroscope(gx, gy, gz);

  }

  
  if (IMU.accelerationAvailable()) {

    IMU.readAcceleration(ax, ay, az);

  }
  
  IMU.readMagneticField(mx, my, mz);
  
  mxcal = mx-xcal;
  mycal = my-ycal;
  mzcal = mz-zcal;
  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;
  
  MahonyQuaternionUpdate(-ax,ay,az,-gx/180*PI,gy/180*PI,gz/180*PI,mxcal,mycal,mzcal,deltat);
 

  quat2eul();
  theta = ee[2];
}


void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
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

void quat2eul() {
    ee[0] = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
    ee[1] = asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
    ee[2] = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

// END IMU DATA

/*
  FUNCTIONS TO DEAL WITH THE ACTUAL BUZZING OF THE ARDUINO

POINT uses the global buzz_dir angle and converts that to an a quadrant on the helmet to buzz and then makes it buzz
*/
void point() {
    motorNumber = int(buzz_dir/PI * 4.5) + 4;
    if (motorNumber == 8) {
      motorNumber = 0;
    }

  bool state[] = {false,false,false,false,false,false,false,false};
  if(motorNumber>=0){
  state[(int)motorNumber] = true;
  }

  updateVibration(state);
}

void updateVibration(bool arr[]){
  for (int i = 0; i <= 7; i++) {
    if (arr[i]) {
      digitalWrite(myPins[i], HIGH);
    } else {
      digitalWrite(myPins[i], LOW);
    }
  }
}

//

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing


boolean newData = false;

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

//============

void showParsedData() {
    Serial.print("Dist: ");
    Serial.print(dist);
    Serial.print("  ");
    Serial.print("Dir: ");
    Serial.print(dir);
    Serial.print("  ");
    Serial.print("User Angle: ");
    Serial.print(unity_heading));
}

// END UNITY PARSING

// Checkpoint checkpoints[];
// User user;


void setup() {
  // Serial.begin(98600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 

  Serial.begin(9600);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT); // Initiate Digital Output Pins 

  // Set the number and values of each checkpoint and direction for the user to follow
  num_checkpoints = 8;

  int[] checkpoint_ds = {200, 600, 200, 300, 400, 300, 200, 100};
  // checkpoints = {{{250, 150}, 270}, {{250, 750}, 90}, {{450, 750}, 90}, {{450, 450}, 270}, {{850, 450}, 270}}, {{850, 750}, 270}, {{750, 750}, 0};
  // initialize the user
  // user = {{0, 0}, 0};

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

}



// Obtain the angle with respect to the physical headset for the motors to buzz based on the direction of the next checkpoint and where the user is looking
void buzzDir(turn_ang, u_heading, imu_heading) {

  // get the desired direction of turn from the next checkpoint (0 is straight and positive is anti-clockwise)
  imu_head = theta;   // imu heading
  if (imu_head < 0) {
    // heading is from 0 straight ahead and positive left but negative right toward +/- 180
    // we want 0 to 360 degree heading so change the right hand side from 0 to -180 to 360 to 180
    imu_head = 360 + head; 
  }
  buzz_dir = dir - unity_heading - imu_head;  // account for the actual heading of the user (where they are looking) to buzz the correct angle for turn
  // return buzz_dir;
}

void loop() {

  // UNITY HANDLED CHECKPOINTS
  for (int i = 0; i < num_checkpoints, i++) {   // loop through all checkpoints
    
    buzz_thresh = checkpoint_ds[i];   // get the initial distance to the current checkpoint
    while (dist > threshold) {  // while the user has not reached the current checkpoint, keep updating and sending directions
      // update user pos
      recvWithStartEndMarkers();
      if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        // Serial.print("Data Recived - ");
        // showParsedData();
        newData = false;
      }

      //Grab IMU data
      updateIMU();

      if (dist <= buzz_thresh) {   // if the user is within a distance threshold, buzz the directions again
        buzz_thresh = buzz_thresh / 2;    // lower the threshold so that the user is not constantly buzzed with directions
        buzzDir()) // get the correct direction on the helmet to buzz to convey direction instruction

        point();
        buzz(buzz_dir, buzz_freq / (std_d / dist));   // actaully buzz the helmet 3 times with the correct direction and a frequency proportional to the distance to the checkpoint
      }
    }
  }

}
