// Includes
#include <math.h>

// Parameter that define what environment to render
//#define ENABLE_VIRTUAL_WALL
#define ENABLE_MASS_SPRING_DAMPER

#define E_Value 2.71828     // Value for e

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

double rs = 0.073152;   //[m]
double rp = 0.004191;   //[m]

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // Position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// Force output variables
double force = 0;                 // Force at the handle
double Tp = 0;                    // Torque of the motor pulley
double duty = 0;                  // Duty cylce (between 0 and 255)
unsigned int output = 0;          // Output command to the motor

// *************** Parameter for the haptic rendering *******************
// Parameter for virtual wall
double x_wall = 0.005;                  // Position of the virtual wall
double k_wall = 1000;                     // Maximum stiffness of the virtual wall
double b_wall = 1;


// Parameter for mass spring damper simulation
double m_msd = 2;        // mass of the mass spring damper simulation in kg
double b_msd = 1;        // damping coefficient of the mass spring damper simulation in Ns/m
double k_msd = 50;       // spring coefficient of the mass spring damper simulation in N/m
double k_user = 1000;     // spring coefficient linking the user's


// position with the position of the mass
double dt = 0.0006;      // time step used for simulation
double x_msd = 0;        // position of the mass
double v_msd = 0;        // velocity of the mass
double a_msd = 0;        // acceleration of the mass
double xi_msd = 0.005;   // initial position of the mass
double vi_msd = 0;       // initial velocity of the mass
double ai_msd = 0;       // initial acceleration of the mass

double f_wall_msd = 0;
double b_wallcont=0;

int counter = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup()
{
  // Set up serial communication
  Serial.begin(115200);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{

  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {       // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
  updatedPos = rawPos + flipNumber * OFFSET; // need to update pos based on what most recent offset is


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Define kinematic parameters you may need
  double rh = 0.075;   //[m]

  // Step B.1
  //Serial.println(updatedPos);

  // Step B.6
  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double theta_s = .0106 * updatedPos - 9.0251; // m = 0.0131 [deg/pos], b = -8.504 [deg]

  // Step B.7
  // Compute the position of the handle based on the angle of the sector pulley
  xh = rh * (theta_s * 3.14159 / 180);

  // Step B.8
  // Serial.println(xh, 4); // [min, max] --> [-0.053, 0.053]

  // Calculate the velocity of the handle
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9 * dxh + 0.1 * dxh_prev;

  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;

  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;

  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  //Serial.println(xh);

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

  // Forces algorithms
#ifdef ENABLE_VIRTUAL_WALL
  if (xh > x_wall)
  {
    force = k_wall*(xh-x_wall);
  }
  else
  {
    force = 0; 
  }

  //using this counter is optional, just limits how many times we print to Processing (increasing baud rate can also help if need be)
  if (counter >= 100) {
    counter = 0;
    Serial.println(xh, 4); //print to processing
  } else {
    counter++;s
  }
#endif 


#ifdef ENABLE_MASS_SPRING_DAMPER
  // Calculate the relevant state at this time step
  v_msd = v_msd + a_msd * dt;
  x_msd = x_msd + v_msd * dt;
  // double x_wall_msd = 0.045;

  // calculate force and a_msd 
  // Determine whether the user's position is in front
  if (xh > x_msd)
  {
    force = k_wall*(xh-x_msd) + b_wall*(v_msd+dxh); 
    
    //a_msd = force/m_msd  - b_msd/m_msd*v_msd - k_msd/m_msd*x_msd;
  }
  else
  {
    //double f_wall_msd = 0;
    //if (x_msd > 0.045) {
    //  f_wall_msd = k_wall * (x_msd - k_wall);
    //}
    force = 0;
    //a_msd = - b_msd/m_msd*v_msd - k_msd/m_msd*x_msd;
    // a_msd = - b_msd/m_msd*v_msd - k_msd/m_msd*x_msd + f_wall_msd / m_msd;
  }

  
    if (x_msd > 0.02) {
      f_wall_msd = k_wall * (x_msd - 0.02);
      if (v_msd > 0) {
        b_wallcont = 3;
      }
      else{
        b_wallcont = 0;
      }
    }
    else
    {
      f_wall_msd = 0;
      b_wallcont = 0;
    }
  a_msd = force/m_msd - b_msd/m_msd*v_msd - k_msd/m_msd*x_msd - f_wall_msd / m_msd - b_wallcont/m_msd*v_msd;

  //using this counter is optional, just limits how many times we print to Processing (increasing baud rate can also help if need be)
  if (counter >= 100) {
    counter = 0;
    //print to processing
    Serial.print(xh, 4);
    Serial.print(' ');
    Serial.println(x_msd, 4);

  } else {
    counter++;
  }
#endif


  //Step C.2
  Tp = rp / rs * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force
  // Serial.println(Tp, 4);

  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction for motor torque
  if (force > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
