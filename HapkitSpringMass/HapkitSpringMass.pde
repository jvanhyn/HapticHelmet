import processing.serial.*;

Serial myPort; 
String nstr = null;

float xwall = 0.045;
float xmin = -0.05;
float xmax = 0.05;

int wposx = 0;
int wposy = 0;
int hposx = 0;
int hposy = 0;

int rxwall = 0;
int rxh = 0;
int rx_msd;

float xh;
float x_msd;

float inByte = 0;
float lastByte = 0;
String input_str;


Mass mass = new Mass();
User user = new User();
Wall wall = new Wall();
//Spring spring = new Spring();

void setup () {
  size(600, 400);        
  println(Serial.list());

  myPort = new Serial(this, Serial.list()[1], 115200);  
  myPort.bufferUntil('\n');
  
  rxwall = (int)map(xwall,xmin,xmax,-width/2,width/2);
  
  mass.center();
  user.center();
  wall.center();
  //spring.center();
  wall.place(rxwall,0);
  mass.place(0,0);
  user.place(0,0);
  //spring.place(mass.x,rxwall,0);

}

void draw(){
  background(0);
  mass.display();
  user.display();
  //spring.display();
  
  
  println(xh + " " +x_msd);
  rxh = (int)map(xh,xmin,xmax,-(width)/2,(width)/2)+mass.size/2;
  rx_msd = (int)map(x_msd,xmin,xmax,-(width)/2,(width)/2)+mass.size/2;
  
  int spring_width = rxwall - rx_msd;
  int spring_center = rx_msd + (spring_width / 2);
  //println(rx_msd, rxwall, spring_center, spring_width);
  
  //line(rx_msd,height/2, rxwall ,height/2);
  
  if(rx_msd + mass.size >(rxwall)){
    mass.place(rxwall - mass.size,0);
    rx_msd = rxwall - mass.size;
  }else{
    mass.place(rx_msd,0);
  }
  
  rect(rx_msd + rxwall + mass.size + wall.w + 20, height/2-5, abs(rxwall - rx_msd - mass.size + wall.w), 10);
  
  if(rxh>rx_msd){
    user.place(rx_msd,0);
  }else{
    user.place(rxh,0);
  }
  

  //mass.place(rx_msd,0);

  wall.display();

}

void serialEvent (Serial myPort) {
    input_str = myPort.readStringUntil('\n');
    input_str = input_str.trim();
    if(input_str != nstr){
      xh = float(input_str.split(" ",2)[0]);
      x_msd = float(input_str.split(" ",2)[1]);
    }

}
