public class Mass {
  int y0 = 0;
  int x0 = 0;
  
  int y = 0;
  int x = 0;
 
  int size = 100;
  
  public void center(){
    x0 = width/2-size/2;
    y0 = height/2-size/2;
    x = x0;
    y = y0;
  }

  public void move(int dx, int dy){
    y -= dy;
    x += dx;
  }
  
   public void place(int x, int y){
    this.y = y+y0;
    this.x = x+x0+size/2 ;
  }
 
  public void display(){
    square(x,y,size);
  }
}
