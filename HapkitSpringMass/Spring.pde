public class Spring{
  int y0 = 0;
  int x0 = 0;
  int y = 0;
  int x = 0;
  int h = 10;
  int w = 300;
  
  public void center(){
    x0 = width/2-w/2;
    y0 = height/2-h/2;
    x = x0;
    y = y0;
  }

  public void move(int dx, int dy){
    y -= dy;
    x += dx;
  }
  
   public void place(int x1, int x2, int y){
    this.y = y+y0;
    this.x = x+x0 + (x1+x2)/2;
    this.w = x2 - x1;
  }
 
  public void display(){
    rect(x,y,w,h);
  }
}
