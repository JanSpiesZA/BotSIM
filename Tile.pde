class Tile
{  
  int gravity = -1;    //-1 is an unoccupied tile
                      
  color gravityCol = color(gravity);
  //Tile type variable to distinguish between different types of obstacles in the occupancy grid
  //0 - Unassigned cell
  //1 - Permanent map obstacle
  //2 - user added obstacle
  //3 - kinect obstacle
  int tileType = 0;    
  
  
  
  Tile()
  {
    gravityCol = color(150,200,150);
    tileType = 0;
  }
  
  void clearGravity()
  {
    switch (tileType)
    {
      case 3:
      {
        gravity = -1;
        gravityCol = color(150,200,150);
        break;
      }
      
      case 1:
      {        
      }
    }
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
      case 0:
      {
        gravityCol = color(200,150,150);
        break;
      }
      
      case 2:
      {
        gravityCol = color(70,130,180);    //steelblue for user obstacles
        break;
      }
      
      case 3:
      {
        gravityCol = color(0,191,255);    //deepskyblue for kinect obstacles
        break;
      }
    }
  }
}