#define BUZZ
//#define Joystick

// Pins corresponding to motors in the headband
int myPins[] = { 2, 3, 4, 5, 6, 7, 8, 9 };

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

// Button press
bool buttonPress;

//Overides all actions if true
bool overide = 0;

int count = 0;
int debounce = 10;

int freq[] = {200,100,50,25};

int j=0;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i <= 7; i++) pinMode(myPins[i], OUTPUT);

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

  if(j<3){
  for(int i = 0; i < 3;i++){
  on();
  delay(freq[j]);
  off();
  delay(freq[j]);
  }
  delay(3000);
  j++;
  }

  // if (Vbutton < 1) {
  //   count++;
  // } else {
  //   count = 0;
  // }

  // if (count > debounce) {
  //   buttonPress = true;
  // } else {
  //   buttonPress = false;
  // }

  // if (buttonPress) {
  //   overide = true;
  //   on();
  // } else {
  //   overide = false;
  //   off();
  // }

#ifdef Joystick
  if (!overide) {
    point(u, v);
  }
#endif
}

void point(double x, double y) {
  theta = -atan2(x, -y) / 3.14;  // find the angle of the Joystick from vertial
  mag = sqrt(pow(x, 2) + pow(y, 2));

  if (mag > thresh) {
    motorNumber = int(theta * 4.5) + 4;
    if (motorNumber == 8) {
      motorNumber = 0;
    }
  } else {
    motorNumber = -1;
  }

  Serial.print(theta);
  Serial.print(",");
  Serial.print(motorNumber);
  Serial.print(",");
  Serial.println(mag);

  for (int i = 0; i <= 7; i++) {
    if (i == motorNumber) {
      digitalWrite(myPins[i], HIGH);
    } else {
      digitalWrite(myPins[i], LOW);
    }
  }
}

void on() {
  for (int i = 0; i <= 7; i++) {
    digitalWrite(myPins[i], HIGH);
  }
}

void off() {
  for (int i = 0; i <= 7; i++) {
    digitalWrite(myPins[i], LOW);
  }
}

void buzz(int dt) {
  for (int i = 0; i > 3; i++) {
    on();
    delay(dt);
    off();
    delay(dt);
  }
}
