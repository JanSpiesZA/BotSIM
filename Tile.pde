class Tile
{  
  int gravity = 0;                          
  color gravityCol;
  
  //Tile type variable to distinguish between different types of obstacles in the occupancy grid
  //0 - Unassigned cell
  //1 - Permanent map obstacle
  //2 - user added obstacle
  //3 - kinect obstacle
  String tileType = "UNASSIGNED";    
  
  PVector field;
  PVector tilePos;
  float force = 0.0;
  
  
  
  Tile()
  {
    gravityCol = color(150,200,150);
    tileType = "UNASSIGNED";
    field = new PVector();
    tilePos = new PVector();
  }
  
  Tile(int _tileX, int _tileY)
  {
    gravityCol = color(150,200,150);
    tileType = "UNASSIGNED";
    field = new PVector();
    tilePos = new PVector();
    tilePos.x = _tileX;
    tilePos.y = _tileY;
  }
  
  void clearGravity()
  {    
  }
  
  void update()
  {    
    switch(tileType)
    {
      case "UNASSIGNED":
      {
        gravity = 0;
        gravityCol = color(150,200,150);
        break;
      }
      case "MAP":
      {
        gravity = 255;        
        calcField();
        gravityCol = color(200,150,150);
        break;
      }
      
      case "USER":
      {
        gravity = 255;        
        calcField();        
        gravityCol = color(70,130,180);    //steelblue for user obstacles
        break;
      }
      
      case "KINECT":
      {        
        gravityCol = color(0,191,255);    //deepskyblue for kinect obstacles
        break;
      }
    }
  }


  void drawTileForce()
  {
    //Draws a flowfield indicator
    stroke(0);
    line (tilePos.x, tilePos.y, tilePos.x + field.x, tilePos.y + field.y);    
  }


  //Calculates the force flow vector based on the position and distance of the robot and the gravity of tha specific tile
  void calcField()
  {
    field.x = myRobot.location.x - tilePos.x;
    field.y = myRobot.location.y - tilePos.y;
    float distance = PVector.dist(myRobot.location, tilePos);
    field.normalize();
      
    force = 100 / pow(distance,2); 
    
    field.mult(force);
    field.mult(gravity);         
  }

}