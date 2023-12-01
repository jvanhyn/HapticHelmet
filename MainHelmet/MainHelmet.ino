#include <Arduino_LSM9DS1.h>
// Magnetometer range is set at [-400, +400] uT +/-0.014 uT.

int myPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
float x, y, z;
float u, v, w;

float theta;
float mags;
float thresh = 0.5;
int heading;

float user_x = 0;
float user_y = 0;
float dist = 0;
int dir = 0;
float threshold = 0.5;  // Change this for how close user has to be
int num_checkpoints;
float buzz_freq = 1;    // buzz frequency at standard distance
float d;
float std_d = 100;      // standard distance is 100



typedef struct
{
    float x;
    float y;
} Coord;

typedef struct
{
    Coord pos;
    int dir;
} Checkpoint;

typedef struct
{
    Coord pos;
    flaot ang;
} User;


Checkpoint checkpoints[];
User user;


void setup() {
  Serial.begin(98600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 

  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT); // Initiate Digital Output Pins 

  pinMode(A0, INPUT); 
  pinMode(A1, INPUT);

  num_checkpoints = 5;
  checkpoints = {{2, 1, 1}, {4, 1, 2}, {4, 4, 2}, {2, 4, 1}, {2, 8, 0}};
  user = {0, 0};
}

float dist(User user, Checkpoint check) {
  ch_x = check.pos.x;
  ch_y = check.pos.y;
  u_x = user.pos.x;
  u_y = user.pos.y;

  d = sqrt(pow((ch_x - u_x), 2) + pow((ch_x - u_y), 2));
  return d;
}

bool reached(User you, Checkpoint check) {

  d = dist(you, check);
  if (d < dist_threshold) {
    return true;
  } else {
    return false;
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  // Get heading from the IMU
  // TODO
  IMU.readMagneticField(x, y, z); // Read IMU DATA
  heading = (atan2(y,x) * 180) / PI;

  // TODO
  // Parse serial string read into user pos

  // Temporary values assuming they are parsed later

  Checkpoint curr;
  for (int i = 0; i < num_checkpoints, i++) {
    curr = checkpoints[i];
    buzz_thresh = dist(user, curr);
    while (!(reached(user, curr))) {
      // update user pos

      d = dist(user, curr);
      if (d <= buzz_thresh) {
        buzz_thresh = buzz_thresh / 2;
        buzz(buzz_freq / (std_d / d)));
      }
    }
  }

  



}
