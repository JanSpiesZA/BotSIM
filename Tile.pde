class Tile
{  
  int gravity = 0;                          
  color gravityCol;
  
  //Tile type variable to distinguish between different types of obstacles in the occupancy grid
  //0 - Unassigned cell
  //1 - Permanent map obstacle
  //2 - user added obstacle
  //3 - kinect obstacle
  int tileType = 0;    
  
  PVector field;
  PVector tilePos;
  float force = 0.0;
  
  
  
  Tile()
  {
    gravityCol = color(150,200,150);
    tileType = 0;
    field = new PVector();
    tilePos = new PVector();
  }
  
  Tile(int _tileX, int _tileY)
  {
    gravityCol = color(150,200,150);
    field = new PVector();
    tilePos = new PVector();
    tilePos.x = _tileX;
    tilePos.y = _tileY;
  }
  
  void clearGravity()
  {
    switch (tileType)
    {
      case 3:
      {
        gravity = -1;
        //force = 0.0;
        field.mult(0);        
        tileType = 0;
        break;
      }
    }
  }
  
  void update()
  {    
    switch(tileType)
    {
      case 0:
      {
        gravity = -1;
        gravityCol = color(150,200,150);
        break;
      }
      case 1:
      {
        gravity = 1;        
        calcField();
        gravityCol = color(200,150,150);
        break;
      }
      
      case 2:
      {
        gravity = 1;        
        calcField();        
        gravityCol = color(70,130,180);    //steelblue for user obstacles
        break;
      }
      
      case 3:
      {
        gravity = 0;
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
      
    force = 1000 / pow(distance,2); 
    
    field.mult(force);
    field.mult(gravity);         
  }

}