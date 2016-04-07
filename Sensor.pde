class Sensor
{
  float sensorXPos = 0.0;      //x position relative to robot centre point
  float sensorYPos = 0.0;      //y position relative to robot center point
  float sensorHAngle = 0.0;    //Angle of sensor in horizontal plane relative to robot chassis. IE rotated left or right
  float sensorVAngle = 0.0;    //Angle of sensor in vertical plane relative to robot chassis. IE tilted up or down
  float sensorGain = 1.0;      //Gains used to indicate importance of sensor
  float sensorObstacleDist = 0.0;  //Distance from THIS sensor to obstacle
  float sensorMaxDetect = 200.0;
  float sensorMinDetect = 2.0;
  float sensorNoise = 5.0;
  
  Sensor(float _sensorXPos, float _sensorYPos, float _sensorHAngle)
  {
    sensorXPos = _sensorXPos;
    sensorYPos = _sensorYPos;
    sensorHAngle = _sensorHAngle;
  }
  
  //Displays a sensor based on the reference X, Y and heading values
  void display(float _refXPos, float _refYPos, float _refHeading)
  {        
    transRot(_refXPos, _refYPos, _refHeading, sensorXPos, sensorYPos);    //Takes the sensor's x,y and plot it in the global frame    
    ellipse(x_temp, y_temp,3,3);
  }  
  
  //Senses if an obstacle is within its cone of detection
  void sense(float _refXPos, float _refYPos, float _refHeading)
  {
    boolean obstacleFlag = false;    
    float obstacleX = 0.0;
    float obstacleY = 0.0; 
     
    fill(255);
    stroke(1);
    
    sensorObstacleDist = sensorMinDetect; //myrobot.diameter/2 + 1;    //Set starting point of collision detect to 1 pixel wider than the radius of the robot    
    obstacleFlag = false;
     
    while ((obstacleFlag == false) && (sensorObstacleDist < sensorMaxDetect))
    {
      //translates and rotates sensor position and heading onto robot frame
      transRot(sensorXPos, sensorYPos, sensorHAngle, sensorObstacleDist, 0);    //Converts distance to sensor frame
      transRot(_refXPos, _refYPos, _refHeading, x_temp, y_temp);
      
      obstacleX = x_temp;
      obstacleY = y_temp;      
      
      color col = img.get (int(x_temp), int(y_temp));    //Test pixel colour to determine if there is an obstacle
      if (red(col) == 0)
        obstacleFlag = true;
      sensorObstacleDist += 1;      
    }
    
    //Add noise to the sensor distance
    sensorObstacleDist += randomGaussian() * sensorNoise;
        
    transRot(sensorXPos, sensorYPos, sensorHAngle, sensorObstacleDist, 0);    //Converts distance to sensor frame
    transRot(_refXPos, _refYPos, _refHeading, x_temp, y_temp);
    ellipse (x_temp, y_temp, 10*scaleFactor,10*scaleFactor);
    
    if (sensorObstacleDist <= safeDistance) myRobot.collisionFlag = true;      //Set collision flag when any sensor is too close to obstacle
  }
}