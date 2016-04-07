class Robot{
  float x = random (0, screenSizeX);  //x-pos of robot
  float y = random (0, screenSizeY);  //y-pos of robot
  float heading = random (0, 2*PI);
  float robotDiameter = 0; //diameter; * scaleFactor;  //diameter of chassis
  float noseLength = diameter/2;
  float maxSpeed = 1 * scaleFactor;
  float maxTurnRate = 5 * scaleFactor;
  float [] state = {x, y, heading};
  boolean collisionFlag = false;
  String nodeType = "";    //ROBOT or PARTICLE
  float prob = 1.0;
  float noiseForward = 0.0;
  float noiseTurn = 0.0;
  float noiseSense = 1.0;  
 
  ArrayList<Sensor> sensors = new ArrayList<Sensor>();
  
  //Instantiates robot as either a ROBOT or a PARTICLE - rules applly differently to the two
  //Supplies a diameter of the robot in cm's for visualization  
  Robot (String _nodeType, float _diameter)
  {   
    nodeType = _nodeType;   
    robotDiameter = _diameter;
  }
  
  Robot (String _nodeType)
  {   
    nodeType = _nodeType;
  }
  
  void addSensor(float _sensorXPos, float _sensorYPos, float _sensorHAngle)
  {
    sensors.add(new Sensor(_sensorXPos, _sensorYPos, _sensorHAngle));
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
  
  
  //Sets movement, sensor and turn noise to robot model
  void setNoise(float _noiseForward, float _noiseTurn, float _noiseSense)
  {
    noiseForward = _noiseForward;
    noiseTurn = _noiseTurn;
    noiseSense = _noiseSense;
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
        
        //Displays position of sensors on robot chassis
        //Sensor data is translated into global coords an then plotted as global coords
        float x_glob = 0.0;
        float y_glob = 0.0;
        fill(255);
        
        //Plots the sensors position on the robot avatar
        for (int i=0; i < numSensors; i++)
        {
          fill(255,0,0);
          transRot(x, y, heading, sensorX[i], sensorY[i]);    //Takes the sensor's x,y and plot it in the global frame
          ellipse(x_temp, y_temp,3,3);
        }
        
        //Displays sensor from ArrayList on robot avatar
        for (int k = 0; k < sensors.size(); k++)
        {
          fill(255,0,0);          
          sensors.get(k).display(x,y,heading);          
        }
        
        break;
      
      case "PARTICLE":
        stroke(255,0,0);
        fill(255,0,0);
        ellipse(x, y, max(1,prob*10), max(1,prob*10));  //Shows a small red dot where the head of the particle is else proportionate to the probability        
        textAlign(CENTER, CENTER);
        fill(0);        
        break;
    } 
    stroke(0);    
    float noseX = x + noseLength * cos(heading);
    float noseY = y + noseLength * sin(heading);
    line (x, y, noseX, noseY);
  }
  

  //Moves the robot  
  void move(float turnAngle, float _forward)
  { 
    heading += turnAngle + randomGaussian() * noiseTurn;  //Add the turnAngle value to the current heading
    if (heading >= (2*PI)) heading -= (2*PI);
    if (heading <= (-2*PI)) heading += (2*PI);    
    
    float distance = _forward + randomGaussian() * noiseForward;
    float newX = x + distance * cos(heading);
    float newY = y + distance * sin(heading);
    x = newX;
    y = newY;
    
    state[0] = x;
    state[1] = y;
    state[2] = heading;   
    
    
    //Allows PARTICLES to live in a continuous world
    if (nodeType == "PARTICLE")
    {
      if (x > screenSizeX) x =- screenSizeX;
      if (x < 0) x += screenSizeX;
      if (y > screenSizeY) y =- screenSizeY;
      if (y < 0) y += screenSizeY;
    }
  }
  
  //Calcualtes distances to obstacles for each sensor in the sensor array   
  void sense()
  {
    for (int k = 0; k < sensors.size(); k++)
    {
      sensors.get(k).sense(x,y,heading);
    }
  }  
  
  //Calculates the probability of how closely a particle's measurements to an obstacle coresponds with that of the robot.
  //A prob value is calculated for each sensors distance which is multiplied to all other probabilities of the specific particle
  //Uses a gausian with:
  //  mu     - Particle's measured distance to a obstacle
  //  sigma  - Particle's measurement noise
  //  x      - Robot's distance measurement of the same sensor  
  void measureProb()
  {        
    prob = 1.0;        //Set probability to maximum value
    float probActual =1.0;
    
    for (int k = 0; k < sensors.size(); k++)
    { 
      float mu = sensors.get(k).sensorObstacleDist;
      float sigma = sensors.get(k).sensorNoise;
      float x = myRobot.sensors.get(k).sensorObstacleDist;      

      prob *= exp(- (pow(mu - x, 2) / pow(sigma,2)/2.0) / sqrt(2*PI * pow(sigma,2)));      
    }    
    //println(prob);
  }
  
}