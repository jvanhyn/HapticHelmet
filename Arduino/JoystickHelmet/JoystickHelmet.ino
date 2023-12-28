//#define BUZZ
//#define PRESS


// Pins corresponding to motors in the headband
int myPins[] = { 2, 3, 4, 5, 6, 7, 8, 9 };

// Motor output state
static bool state[] = { false, false, false, false, false, false, false, false };

// Number correponding to a motor at a specific location on the head
double motorNumber = 0;

// Voltage Inputs
double Vy;
double Vx;
double Vbutton;

// Orientation Information
double u;
double v;
double theta;
double mag;

// Joystick Sensetivity
double thresh = 0.5;





void setup() {
  Serial.begin(9600);
  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}

void loop() {
  Vy = analogRead(A0);       // Read the Joystick y pos
  Vx = analogRead(A1);       // Read the Joystick x pos
  Vbutton = analogRead(A2);  // Read Joystick button

  u = (Vy) / 1023 * 2 - 1;  // Convert (Vx,Vy) to a vector inside the unit circle
  v = (Vx) / 1023 * 2 - 1;


  point(u,v);
  updateVibration();
  Serial.println(motorNumber);
 
}

void point(double x, double y) {
  theta = atan2(x, -y) / 3.14;  // find the angle of the Joystick from vertial
  mag = sqrt(pow(x, 2) + pow(y, 2));

  if (mag > thresh) {
    motorNumber = int(theta * 4.5) + 4;
    if (motorNumber == 8) {
      motorNumber = 0;
    }
  } else {
    motorNumber = -1;
  }

  for (int i = 0; i <= 7; i++) {
    state[i] = false;
  }

  if (motorNumber >= 0) {
    state[(int)motorNumber] = true;
  }

  if (motorNumber >= 0) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}



void updateVibration() {
  for (int i = 0; i <= 7; i++) {
    if (state[i]) {
      digitalWrite(myPins[i], HIGH);
    } else {
      digitalWrite(myPins[i], LOW);
    }
  }
}

// void on() {
//   for (int i = 0; i <= 7; i++) {
//     digitalWrite(myPins[i], HIGH);
//   }
// }

// void off() {
//   for (int i = 0; i <= 7; i++) {
//     digitalWrite(myPins[i], LOW);
//   }
// }
