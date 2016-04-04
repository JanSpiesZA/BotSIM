class Robot{
  float x = random (0, screenSizeX);  //x-pos of robot
  float y = random (0, screenSizeY);  //y-pos of robot
  float heading = random (0, 2*PI);
  float robotDiameter = 45 * scaleFactor;  //diameter of chassis
  float noseLength = diameter/2;
  float maxSpeed = 1 * scaleFactor;
  float maxTurnRate = 5 * scaleFactor;
  float [] state = {x, y, heading};
  boolean collisionFlag = false;
  String nodeType = "";    //ROBOT or PARTICLE
  float prob = 1.0;
  
  Robot (String _nodeType)
  {   
    nodeType = _nodeType;
  }
  
  void set(float tempX, float tempY, float tempHeading)
  {
    x = tempX;
    y = tempY;
    heading = tempHeading;
    state[0] = x;
    state[1] = y;
    state[2] = heading;
  }
  
//Draws the robot() and plots the sensors positions
  void display()
  {    
    //Displays the heading of the robot as a line from the centerpoint of the robot into the same direction as the heading of the robot
    //if (collisionFlag)
    //{
    //  stroke (255,0,0);
    //} 
    //else
    //{
    //  stroke (0);
    //}
    
    switch (nodeType)
    {
      case "ROBOT":
        stroke(0);
        fill(0,255,0);    
        ellipse(x, y, robotDiameter, robotDiameter);         
        textAlign(CENTER, CENTER);
        textSize(10);
        fill(0);
        //text(dist, xPos, yPos-10);   
        
        //Displays position of sensors on robot chassis
        //Sensor data is translated into global coords an then plotted as global coords
        float x_glob = 0.0;
        float y_glob = 0.0;    
        for (int i=0; i < numSensors; i++)
        {
          transRot(x, y, heading, sensorX[i], sensorY[i]);    //Takes the sensor's x,y and plot it in the global frame
          ellipse(x_temp, y_temp,3,3);
        }
        
        break;
      
      case "PARTICLE":
        stroke(255,0,0);
        fill(255,0,0);
        ellipse(x, y, prob*10, prob*10);
        //ellipse(xPos, yPos, 5, 5);
        textAlign(CENTER, CENTER);
        fill(0);
        //text(prob, xPos,yPos-10);
        //print(prob+",");
        break;
    } 
    stroke(0);    
    float noseX = x + noseLength * cos(heading);
    float noseY = y + noseLength * sin(heading);
    line (x, y, noseX, noseY);
  }
  

//Moves the robot  
  void move(float turnAngle, float distance)
  { 
    heading += turnAngle;  //Add the turnAngle value to the current heading
    if (heading >= (2*PI)) heading -= (2*PI);
    if (heading <= (-2*PI)) heading += (2*PI);    
    
    float newX = x + distance * cos(heading);
    float newY = y + distance * sin(heading);
    x = newX;
    y = newY;
    
    state[0] = x;
    state[1] = y;
    state[2] = heading;    
  }
  
}