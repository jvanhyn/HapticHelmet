int myPins[] = { 2, 3, 4, 5, 6, 7, 8, 9 };

int point(double x, double y) {
  float theta = -atan2(x, -y) / 3.14;  // find the angle of the Joystick from vertial
  float mag = sqrt(pow(x, 2) + pow(y, 2));
  return theta;
}

bool updateArr(float theta){
  
    float motorNumber = int(theta * 4.5) + 4;
    if (motorNumber == 8) {
      motorNumber = 0;
    }

  bool arr[] = {false,false,false,false,false,false,false,false};
  return arr;
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

void buzz(bool arr[],int dt) {
  for (int i = 0; i < 3; i++) {
    updateVibration(arr);
    delay(dt);
    off();
    delay(dt);
  }
  delay(100);
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