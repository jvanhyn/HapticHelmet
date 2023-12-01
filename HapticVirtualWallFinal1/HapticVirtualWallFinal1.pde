// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
 Last Modified by Tania Morimoto 2/14/2023:
  - more detailed comments/instructions
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 38400 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

Serial myPort; 
String nstr = null;

float xwall = 0.005;
float xmin = -0.05;
float xmax = 0.05;

int rxwall = 0;
int rxh = 0;

float inByte = 0;
float lastByte = 0;
String input_str;
float xh;

Wall wall = new Wall();
User user = new User();

void setup () {
  size(600, 400);        
  println(Serial.list());

  myPort = new Serial(this, Serial.list()[1], 115200);  
  myPort.bufferUntil('\n');
  
  rxwall = (int)map(xwall,xmin,xmax,-width/2,width/2);
  
  wall.center();
  user.center();
  wall.place(rxwall,0);
}

void draw () {
  // everything happens in the serialEvent()\
  background(0);           //uncomment if you want to control a ball
  //stroke(127,34,255);     //stroke color
  //strokeWeight(4);        //stroke wider
  frameRate(60);
  
  rxh = (int)map(xh,xmin,xmax,-width/2,width/2);
  println(rxh);
  wall.display();
  user.display();
  if(rxh>rxwall){
    user.place(rxwall,0);
  }else{
    user.place(rxh,0);
  }
}

void serialEvent (Serial myPort) {
    input_str = myPort.readStringUntil('\n');
    input_str = input_str.trim();
    if(input_str != nstr){
      xh = float(input_str);
    }

}
