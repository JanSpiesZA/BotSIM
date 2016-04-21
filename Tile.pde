class Tile
{
  int gravity = -1;    //Number of points in tile
  color gravityCol = color(gravity);
  //int[] arrayXY = {0,0};
  //int[] worldXY = {0,0};
  
  Tile()
  {
    gravityCol = color(150,200,150);
  }
  
  void clearGravity()
  {
    gravity = -1;
    gravityCol = color(150,200,150);
  }
  
  void update()
  {    
    switch(gravity)
    {
      case -1:
      {
        gravityCol = color(150,200,150);
        break;
      }
      case 1:
      {
        gravityCol = color(200,150,150);
        break;
      }
    }
  }
}