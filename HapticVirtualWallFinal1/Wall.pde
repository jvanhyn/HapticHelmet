public class Wall{
  int y0 = 0;
  int x0 = 0;
  int y = 0;
  int x = 0;
  int h = 300;
  int w = 10;
  
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
  
   public void place(int x, int y){
    this.y = y+y0;
    this.x = x+x0+w/2;
  }
 
  public void display(){
    rect(x,y,w,h);
  }
}
