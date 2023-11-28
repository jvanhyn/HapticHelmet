#include <Arduino_LSM9DS1.h>

double Vy;
double Vx;
double theta;
bool yes;
double vibe = 0;

int Pins[] = {5,6,7,8,9,10,11,12};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  for(int i = 0; i <= 7; i++){
  pinMode(Pins[i],INPUT);
  }
  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

}

void loop() {
  Vy = ((double)analogRead(A0))/1023*2-1;
  Vx = ((double)analogRead(A1))/1023*2-1;

  theta = -atan2(Vx,-Vy)/3.14*4.5;
  yes = sqrt(pow(Vx,2)+pow(Vy,2)) > 0.5;

  if(yes){
    vibe = trunc(theta)+4;
    if(vibe == 8){
      vibe = 0;
    }
  } else {
    vibe = -1;
  }

  
  for(int i = 0; i <= 7; i++){
    if(i == vibe){
    pinMode(Pins[i],OUTPUT);
    } else {
    pinMode(Pins[i],INPUT);
    }
    }
  
}
