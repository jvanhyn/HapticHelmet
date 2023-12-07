#include <Arduino_LSM9DS1.h>
// Magnetometer range is set at [-400, +400] uT +/-0.014 uT.

int myPins[] = {5, 6, 7, 8, 9, 10, 11, 12}; // pins on arduino for controlling each motor

// initialize values for IMU readings
float x, y, z;
float u, v, w;

// Variables related to positioning and navigation
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
    float ang;
} User;


Checkpoint checkpoints[];
User user;


void setup() {
  Serial.begin(98600); // Start Serial Communitcation
  IMU.begin(); // Start IMU 

  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT); // Initiate Digital Output Pins 

  pinMode(A0, INPUT); 
  pinMode(A1, INPUT);

  // Set the number and values of each checkpoint and direction for the user to follow
  num_checkpoints = 7;
  checkpoints = {{{250, 150}, 270}, {{250, 750}, 90}, {{450, 750}, 90}, {{450, 450}, 270}, {{850, 450}, 270}}, {{850, 750}, 270}, {{750, 750}, 0};
  // initialize the user
  user = {{0, 0}, 0};
}

// function to return the distance between the user and the next chckpoint
float dist(User user, Checkpoint check) {
  ch_x = check.pos.x;
  ch_y = check.pos.y;
  u_x = user.pos.x;
  u_y = user.pos.y;

  // simple euclidean distance calculator
  d = sqrt(pow((ch_x - u_x), 2) + pow((ch_x - u_y), 2));
  return d; // return distance
}

// function to check whether the user has "reached" a checkpoint
// return true if the user is within a certain distance threshold of the checkpoint, false otherwise
bool reached(User you, Checkpoint check) {

  d = dist(you, check);   // check distance between user and the checkpoint
  if (d < dist_threshold) {
    return true;
  } else {
    return false;
  }

}

// Obtain the angle with respect to the physical headset for the motors to buzz based on the direction of the next checkpoint and where the user is looking
int buzzDir(Checkpoint ch, heading) {

  dir = ch.dir;   // get the desired direction of turn from the next checkpoint (0 is straight and positive is anti-clockwise)
  head = heading;
  if (head < 0) {
    // heading is from 0 straight ahead and positive left but negative right toward +/- 180
    // we want 0 to 360 degree heading so change the right hand side from 0 to -180 to 360 to 180
    head = 360 + head; 
  }
  buzz_dir = dir - head;  // account for the actual heading of the user (where they are looking) to buzz the correct angle for turn
  return buzz_dir;
}

void loop() {
  // put your main code here, to run repeatedly:

  // TODO
  // Parse serial string read into user pos

  Checkpoint curr;
  for (int i = 0; i < num_checkpoints, i++) {   // loop through all checkpoints
    curr = checkpoints[i];  // obtain the current checkpoint the user is trying to get to
    buzz_thresh = dist(user, curr);   // get the initial distance to the current checkpoint
    while (!(reached(user, curr))) {  // while the user has not reached the current checkpoint, keep updating and sending directions
      // update user pos

      // update the user heading from IMU
      IMU.readMagneticField(x, y, z); // Read IMU DATA
      heading = (atan2(y,x) * 180) / PI;

      d = dist(user, curr);     // update the current distance of the user to the checkpoint
      if (d <= buzz_thresh) {   // if the user is within a distance threshold, buzz the directions again
        buzz_thresh = buzz_thresh / 2;    // lower the threshold so that the user is not constantly buzzed with directions
        buzz_dir = buzzDir(curr, heading) // get the correct direction on the helmet to buzz to convey direction instruction
        buzz(buzz_dir, buzz_freq / (std_d / d)));   // actaully buzz the helmet 3 times with the correct direction and a frequency proportional to the distance to the checkpoint
      }
    }
  }

  



}
