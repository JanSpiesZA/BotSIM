//All distances are measured and listed in cm's unless specified differently


//Actual distance of measured on ground, measured in cm's
float worldMapScaleX = 1000; //3737;      //To be used as the actual distance of the world map x axis, measured in cm
float worldMapScaleY = 1000; //1137;

float screenSizeX = 500;
float screenSizeY = screenSizeX * (worldMapScaleY/worldMapScaleX);  //Scale the y size according to ratio between worldMapScaleX an Y

float scaleFactor = screenSizeX / worldMapScaleX;

boolean makingProgress = true;    //Indicates if progress towards the goal is being made
boolean wallDetect = false;

Robot myRobot;          //Creat a myRobot instance
float diameter = 45.0; 

final int maxParticles = 0;
Robot[] particles = new Robot[maxParticles];
final float noiseForward = 1.0;            //global Noisevalues used to set the noise values in the praticles
final float noiseTurn = 0.1;
final float noiseSense = 5.0;

float moveSpeed = 0;                    //Globals used to define speeds and turn angle
float moveAngle = 0;

float turnGain = 0.1;
float moveGain = 0.01;
float blendGain = 0.5;      //Gain used when blending the AO and GTG vectors;
float normaliseGain = 100.0;

float safeZone = 20.0;          //Safe area around target assumed the robot reached its goal;
int safeDistance = 40;      //If sensor measured distance is less than this value, the robot is too close to an obstacle
float distanceFromWall = 50.0;    //Distance that must be maintained when following the wall



//This section must be removed when only sensor class is used
float[] sensorX =   {0.0, cos(PI/8*3)* diameter/2, cos(PI/8*2)*diameter/2, cos(PI/8)*diameter/2, diameter/2, cos(PI/8)*diameter/2, cos(PI/4)*diameter/2, cos(PI/8*3)*diameter/2, 0.0};      //Array containing all the sensors X values in the robot frame
float[] sensorY =   {-(diameter/2), -sin(PI/8*3)* diameter/2, -sin(PI/8*2)*diameter/2, -sin(PI/8)*diameter/2, 0.0, sin(PI/8)*diameter/2, sin(PI/4)*diameter/2, sin(PI/8*3)*diameter/2, diameter/2};
float[] sensorPhi = {-PI/2, -PI/8*3, -PI/8*2, -PI/8, 0.0, PI/8, PI/4, PI/8*3, PI/2};
float[] sensorGains = {1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.0};    //Gains used to indicate imprtance of sensor values
//This section must be removed when only sensor class is used



float[] vectorAO_GTG = {0.0, 0.0};    //x and y values for avoid obstacle and go-to-goal combined vector
PVector vectorAOGTG = new PVector();
float[] vectorAO = {0.0, 0.0};      //x and y values for avoid obstacle vector
PVector vectorAvoidObstacles = new PVector();
float[] vectorGTG = {0.0, 0.0};      //x and y values for vector go-to-goal
PVector vectorGoToGoal = new PVector();
float[] vectorWall = {0.0, 0.0};      //x and y values representing the vector of a piece of wall for follow wall procedure
float[] vectorWallDist = {0.0, 0.0};  //x and y values for a line perpendicular to the wall vector
float[] vectorAwayFromWall = {0.0, 0.0};  //x and y values for vector pointing away from the wall 
float[] vectorFollowWall = {0.0, 0.0};    //Vector pointing in the direction the robot must move when following the wall

int numSensors = sensorX.length;    //Determines the amount of sensor elements present
int numSensors2 = 9;
float[] sensorObstacleDist = new float[numSensors];
int minDetectDistance = 0;        //Closer than this value and the sensors do not return valid data
float maxDetectDistance = 200.0;

float goalX = screenSizeX / 2;            //Goal's X and Y coordinates, set up by clicking with the mouse on the screen
float goalY = screenSizeY / 2;
float startX = 0;          //Starting point for straight line to goal used by Bug algorithm families
float startY = 0;
float x_vector_avoid = 0.0;
float y_vector_avoid = 0.0;
float phi_avoid = 0.0;
float errorAngle = 0.0;
float[] progressPoint = {10.0, 10.0};
float[] closest1 = {0.0, 0.0};
float[] closest2 = {0.0, 0.0};

int stateVal = 0;      //Values used to indicate which state the robot is currently in

boolean showVal = false;
boolean step = true;

//Measurement of tiles to be used for occupancy grid in cm's scaled to represented size in real world
int tileSize = int(50 * scaleFactor);                            
int maxTilesX = int(screenSizeX/tileSize);
int maxTilesY = int(screenSizeY/tileSize);
Tile tile[][] = new Tile[maxTilesX][maxTilesY];

void setup()
{

  myRobot = new Robot("ROBOT", diameter);        //Create a new robot object
  myRobot.set(screenSizeX/2, screenSizeY/2, -PI/2);
    

  //Add sensors to the robot object 
  for (int k=0; k<numSensors2; k++)
  { 
    myRobot.addSensor(0, 0, -PI/2 + PI/(numSensors-1)*k);   
    myRobot.sensors.get(k).sensorMinDetect = minDetectDistance;
  }  

  //Sets up a 2D array which will hold the world Tiles
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      tile[x][y] = new Tile();
    }
  }  

  for (int i = 0; i < maxParticles; i++)
  {
    particles[i] = new Robot("PARTICLE");  
    particles[i].set(screenSizeX/2, screenSizeY/2, -PI/2);
    particles[i].setNoise(noiseForward, noiseTurn, noiseSense);    //Add noise to newly created particle

    for (int k = 0; k < numSensors2; k++)
    {
      particles[i].addSensor(0, 0, -PI/2 + PI/(numSensors2-1)*k);
    }
  }
  
  applyScale();    //Applies the scale to all physical quantities

  surface.setResizable(true);
  surface.setSize(int(screenSizeX), int(screenSizeY));

  //Change particle x and y values to prevent them from being inside walls
  //for (int i=0; i < maxParticles; i++)
  //{  
  //  color col = img.get (int(particles[i].x) ,int(particles[i].y));    //Test pixel colour to determine if there is an obstacle
  //  if (red(col) == 0)
  //  {
  //    while(red(col) == 0)    
  //    {
  //      particles[i].x = random (0, screenSizeX);
  //      particles[i].y = random (0, screenSizeY);
  //      col = img.get (int(particles[i].x) ,int(particles[i].y));    //Test pixel colour to determine if there is an obstacle
  //    }
  //  }      
  //}
}

void draw()
{

  if (showVal)
  {
    for (int k=0; k<numSensors; k++) print(int(myRobot.sensors.get(k).sensorObstacleDist)+"\t");
    println("\nState: "+stateVal+", CollisionFlag: "+myRobot.collisionFlag);
    println();

    for (int k = 0; k< maxParticles; k++) 
    {
      for (int i = 0; i < numSensors; i++)
      {        
        print (int(particles[k].sensors.get(i).sensorObstacleDist)+"\t");
      }
      print ("PROB: "+ particles[k].prob);
      println();
    }
    println();
    showVal = false;
  }

  if (step)
  {
    //background(img);                                  //Make the background the orginal map image    
    //background(255);
    drawTiles();
    drawTarget();
    PlotRobot();

    //detectObstacle();        //Detects distance to obstacle not using the sensor class  

    myRobot.sense();          //Makes use of sensor class to detect obstacles

    for (int k = 0; k < maxParticles; k++)
    {    
      particles[k].sense();
      particles[k].measureProb();
    }

    // updateParticles(); 

    calcProgressPoint();

    // resample();



    step = false;
  }


  //calcVecAO();       //Calculates the avoid obstacle vector;
  
  
  
  vectorAvoidObstacles = calcVectorAvoidObstacles(); 
  vectorGoToGoal = calcVectorGoToGoal();
  //vectorAOGTG = vectorAvoidObstacles;
  vectorAOGTG = PVector.add(vectorGoToGoal, vectorAvoidObstacles);
  //println(vectorGoToGoal+" : "+vectorAvoidObstacles+" : "+vectorAOGTG);
  
  
  
  calcVecGTG();
  //calcVecAO_GTG();    //Calculates vector after blending Go-To-Goal and Avoid_Obstacle;
  //estimateWall();    //Estimates the distance to the wall using closest sesnors to the wall  
  dispVectors();      //Displays different vectors, ie: Go-To-Goal, Avoid Obstacle, etc
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void drawTiles()
{
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      stroke(0);        //Lines between tiles are black
      strokeWeight(1);  //Stroke weight makes the lines very light
      fill(tile[x][y].gravityCol);
      rect(x*tileSize, y*tileSize, tileSize, tileSize);  //Draws a rectangle to indicate the tile
    }
  }
}

//###############################################################################################
void updateParticles()
{
  //Update particle movement
  for (int i = 0; i < maxParticles; i++)
  {
    particles[i].move(moveAngle, moveSpeed);
  }

  //Display updated particles
  for (int i=0; i < maxParticles; i++)
  {
    particles[i].display();
  }
}

//###############################################################################################
void applyScale()
{
  myRobot.robotDiameter *= scaleFactor;
  myRobot.noseLength *= scaleFactor;
  myRobot.maxSpeed *= scaleFactor;
  myRobot.maxTurnRate *= scaleFactor;
  //minDetectDistance *= scaleFactor;        //Closer than this value and the sensors do not return valid data
  //maxDetectDistance *= scaleFactor;  
  safeZone *= scaleFactor;          //Safe area around target assumed the robot reached its goal;
  safeDistance *= scaleFactor;    
  distanceFromWall *= scaleFactor;

  //Apply the scalefactor to the position of each of the sensors
  //sensorObstacleDist[i] will automatically be less since the map being measured is smaller
  for (int i = 0; i < numSensors; i++)
  {
    sensorX[i] *= scaleFactor;
    sensorY[i] *= scaleFactor;
  }

  //Applies scale factor to sensors on the robot
  //---should probably be moved to the robot or sensor display function
  for (int k = 0; k < myRobot.sensors.size(); k++)
  {
    myRobot.sensors.get(k).sensorXPos *= scaleFactor;
    myRobot.sensors.get(k).sensorYPos *= scaleFactor;
    myRobot.sensors.get(k).sensorMaxDetect *= scaleFactor;
  }
}
//###############################################################################################
void PlotRobot()
{
  float difference = 0.0;

  float deltaX = goalX - myRobot.x;
  float deltaY = goalY - myRobot.y;  
  float targetAngle = atan2(deltaY, deltaX);    
  float distanceToTarget = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

  float phi_GTG = calcGoalAngle(vectorGTG[0], vectorGTG[1]);
  float phi_AO = calcGoalAngle(vectorAO[0], vectorAO[1]); 
  float phi_AO_GTG = calcGoalAngle(vectorAO_GTG[0], vectorAO_GTG[1]);
  float phi_FW = calcGoalAngle(vectorFollowWall[0], vectorFollowWall[1]);


  switch (stateVal)
  {
  case 0:    //Stop State / Arrived at Goal
    if (distanceToTarget > safeZone) 
    {
      stateVal = 1;
    } else
    {
      distanceToTarget = 0;
    }
    break;

  case 1:    //Go straight to goal
    //calcErrorAngle(phi_GTG);

    if (distanceToTarget <= safeZone)   //Robot is close enough to goal, stop robot
    {
      stateVal = 0;
    }

    if (myRobot.collisionFlag)
    {
      stateVal = 2;
    }

    if (!myRobot.collisionFlag) 
    {
      calcErrorAngle(phi_GTG);
    }

    if ((!makingProgress) && (myRobot.collisionFlag))
    {
      stateVal = 3;
    }
    break;

  case 2:    //Avoid obstacle state
    calcErrorAngle(phi_AO);

    if (myRobot.collisionFlag)
    {
      stateVal = 1;
      myRobot.collisionFlag = false;
    }
    break;      

  case 3:
    calcErrorAngle(phi_FW);
    //makingProgress = true;
    if (makingProgress) stateVal = 1;
    break;
  } 

  //calcErrorAngle(phi_FW); 

  //Stop the robot when it is close enough to the target area
  //if (distanceToTarget <= safeZone) distanceToTarget = 0;  

  //float errorAngle = difference;
  //println("Length of vector: ",distanceToTarget, " : ",stateVal);
  //println (myRobot.heading, "\t", errorAngle,"\t",phi_AO);
  //println("Collisionflag :",collisionFlag, " Making Progress :", makingProgress);


  moveAngle = min (myRobot.maxTurnRate, (turnGain * errorAngle));  //P controller to turn towards goal
  moveSpeed = min (myRobot.maxSpeed, (moveGain * (distanceToTarget))); 
  myRobot.move(moveAngle, moveSpeed);  
  myRobot.display();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//  Calculates a progress point towards the goal.
//  If the robot's distance-to-goal is less than the progress point distance-to-goal,
//      the robot progresses, else do Wall Following

void calcProgressPoint()
{
  float oldDist = sqrt(pow(goalX - progressPoint[0], 2) + pow(goalY - progressPoint[1], 2));    //Calculates the straight line distance to the goal
  float newDist = sqrt(pow(goalX - myRobot.state[0], 2) + pow(goalY - myRobot.state[1], 2));

  if (newDist <= oldDist)
  {
    progressPoint[0] = myRobot.state[0];
    progressPoint[1] = myRobot.state[1];
    makingProgress = true;
  } else
  {
    makingProgress = false;
  }

  strokeWeight(5);
  stroke(0, 255, 255);
  ellipse(progressPoint[0], progressPoint[1], 5, 5);
  strokeWeight(1);
  stroke(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// This function takes the angle towards the destination point and calclates the folowing:
//    ??1) Converts it from an atan2 angle into a real world angle
//    2) Calculates the difference between the robot's heading and the goal angle
//    3) Converts this difference into the robot's local frame in order to determine left and right turns
// Based on Games Programming: Methods and How to's - Dr James Jordaan Revision 4.1 p196
void calcErrorAngle (float goalAngle)
{  
  //if (goalAngle < 0) goalAngle += (2*PI);        //Change goal angle from atan2 to radians (global frame)
  errorAngle = goalAngle - myRobot.heading;
  if (errorAngle < -PI) errorAngle += (2*PI);
  if (errorAngle > PI) errorAngle -= (2*PI);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void estimateWall()
{ 
  int c1 = 0;
  int c2 = 1;

  //Step 1:
  //  Determine the two smallest values from the left sensors  
  for (int i = 0; i <= ceil(numSensors/2); i++)
  {
    if (sensorObstacleDist[i] < sensorObstacleDist[c1]) c1 = i;
  }

  for (int i = 0; i <= ceil(numSensors/2); i++)
  {
    if ((sensorObstacleDist[i] < sensorObstacleDist[c2]) && (sensorObstacleDist[i] > sensorObstacleDist[c1])) c2 = i;
  }
  if (c1 == c2) c2 = c1 + 1;

  int leftC1 = min(c1, c2);
  int leftC2 = max(c1, c2); 

  c1 = leftC1;
  c2 = leftC2;

  //step 2:
  //  Convert the distances to points in the global frame
  //  Create a wall vector and normalise it
  if ((c1 >= 0) & (c2 >= 0))
  { 
    PVector returnVal = transRot (sensorX[c1], sensorY[c1], sensorPhi[c1], sensorObstacleDist[c1], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.x, myRobot.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    closest1[0] = returnVal.x;
    closest1[1] = returnVal.y;

    returnVal = transRot (sensorX[c2], sensorY[c2], sensorPhi[c2], sensorObstacleDist[c2], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.x, myRobot.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    closest2[0] = returnVal.x;
    closest2[1] = returnVal.y;

    ellipse (closest1[0], closest1[1], 20, 20);
    ellipse (closest2[0], closest2[1], 20, 20);
    //line (closest1[0], closest1[1], closest2[0], closest2[1]);

    vectorWall[0] = closest2[0] - closest1[0];
    vectorWall[1] = closest2[1] - closest1[1];

    float n = sqrt(pow(vectorWall[0], 2)+pow(vectorWall[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector  
    vectorWall[0] = normaliseGain * vectorWall[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
    vectorWall[1] = normaliseGain * vectorWall[1]/n;  

    //Step 3
    //  Compute a vector perpendicular with the wall pointing from the center of the robot to the wall vector    
    //  http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point

    float k = ((closest2[1] - closest1[1]) * (myRobot.x - closest1[0]) - (closest2[0] - closest1[0])*(myRobot.y - closest1[1])) / (pow(closest2[1]-closest1[1], 2) + pow(closest2[0]-closest1[0], 2));
    vectorWallDist[0] = myRobot.x - k *(closest2[1] - closest1[1]);
    vectorWallDist[1] = myRobot.y + k *(closest2[0] - closest1[0]);

    vectorWallDist[0] -= myRobot.x;
    vectorWallDist[1] -= myRobot.y;    

    n = sqrt(pow(vectorWallDist[0], 2)+pow(vectorWallDist[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector
    /*  
     vectorWallDist[0] = normaliseGain * vectorWallDist[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
     vectorWallDist[1] = normaliseGain * vectorWallDist[1]/n;  
     */
    //Step 4
    //  Determines vector pointing away from the wall and weighted with distanceFromWall    
    for (int i = 0; i < 2; i++)
    {
      vectorAwayFromWall[i] =  vectorWallDist[i] - distanceFromWall * vectorWallDist[i]/n;
    }


    //Step 5
    //  Calculates vectorFollowWall used to point the robot in the right direction when following the wall 
    for (int i=0; i < 2; i++)
    {
      vectorFollowWall[i] = vectorWall[i] + vectorAwayFromWall[i];
    }
  }
} 

/////////////////////////////////////////////////////////////////////////////////////////////////
void detectObstacle()
{ 
  boolean obstacleFlag = false;
  float detectDistance = 0.0;
  float obstacleX = 0.0;
  float obstacleY = 0.0; 

  fill(255);
  stroke(1);
  for (int i = 0; i < numSensors; i++)
  {
    sensorObstacleDist[i] = 2; //myRobot.diameter/2 + 1;    //Set starting point of collision detect to 1 pixel wider than the radius of the robot    
    obstacleFlag = false;

    while ((obstacleFlag == false) && (sensorObstacleDist[i] < maxDetectDistance))
    {
      PVector returnVal = transRot(myRobot.x, myRobot.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
      obstacleX = returnVal.x + sensorObstacleDist[i] * cos(myRobot.heading + sensorPhi[i]);
      obstacleY = returnVal.y + sensorObstacleDist[i] * sin(myRobot.heading + sensorPhi[i]);   
      color col = get (int(obstacleX), int(obstacleY));    //Test pixel colour to determine if there is an obstacle
      if (col == color(200, 150, 150))                //Test to see if tile is an obstacle
        obstacleFlag = true;
      sensorObstacleDist[i] += 1;
    }
    if (sensorObstacleDist[i] <= safeDistance) myRobot.collisionFlag = true;      //Set collision flag when any sensor is too close to obstacle
    ellipse (obstacleX, obstacleY, 10*scaleFactor, 10*scaleFactor);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//Calculates a vector which points away from obstacles using the distance vectors from all the sensors to obstacles 
void calcVecAO()
{
  float x_vector = 0.0;
  float y_vector = 0.0;
  float x1_vector = 0.0;
  float y1_vector = 0.0;

  vectorAO[0] = 0.0;
  vectorAO[1] = 0.0;

  for (int i=0; i < numSensors; i++)
  {
    PVector returnVal = transRot(myRobot.x, myRobot.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
    x1_vector = returnVal.x; //x_temp;
    y1_vector = returnVal.y; //y_temp;

    returnVal = transRot (sensorX[i], sensorY[i], sensorPhi[i], sensorObstacleDist[i], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.x, myRobot.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    x_vector = returnVal.x - x1_vector;
    y_vector = returnVal.y - y1_vector;

    //Add all the vectors together and multiply with vector gains
    vectorAO[0] += (x_vector * sensorGains[i]);
    vectorAO[1] += (y_vector * sensorGains[i]);  

    //line (myRobot.x, myRobot.y, x_temp, y_temp);
  }
  float n = sqrt(pow(vectorAO[0], 2)+pow(vectorAO[1], 2));    //Calculates the normalisation factor for the AvoidObstacle vector

  vectorAO[0] = 100 * vectorAO[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorAO[1] = 100 * vectorAO[1]/n;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
PVector calcVectorAvoidObstacles()
{
  PVector tempCoords = new PVector();  
  PVector location = new PVector(myRobot.x, myRobot.y);
  PVector result = new PVector();

  for (int k = 0; k < myRobot.sensors.size(); k++)
  {
    //TransRotate sensor distance value to sensor frame
    tempCoords = transRot(myRobot.sensors.get(k).sensorXPos, myRobot.sensors.get(k).sensorYPos, myRobot.sensors.get(k).sensorHAngle, myRobot.sensors.get(k).sensorObstacleDist, 0);
    //TransRotate sensor distance value in sensor frame to robot frame
    tempCoords = transRot(myRobot.x, myRobot.y, myRobot.heading, tempCoords.x, tempCoords.y);

    //Calculate vector away from obstacles from p143 in Behaviour Based Robotics
    tempCoords = tempCoords.sub(location);    
    result.add(tempCoords);
  } 
  return result.normalize();
}


/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecGTG()
{
  vectorGTG[0] = goalX - myRobot.x;
  vectorGTG[1] = goalY - myRobot.y;

  float n = sqrt(pow(vectorGTG[0], 2)+pow(vectorGTG[1], 2));    //Calculates the normailsation factor for the AvoidObstacle vector
  if (n==0) n=1;
  vectorGTG[0] = 100 * vectorGTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorGTG[1] = 100 * vectorGTG[1]/n;
  
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////
PVector calcVectorGoToGoal()
{
  PVector result = new PVector();  
  result.x = goalX - myRobot.x;
  result.y = goalY - myRobot.y;
  result.normalize();  
  return result;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecAO_GTG()
{       

  for (int i = 0; i <= 1; i++)
  {
    vectorAO_GTG[i] = blendGain*vectorAO[i] + (1 - blendGain)*vectorGTG[i];
  }  

  float n = sqrt(pow(vectorAO_GTG[0], 2)+pow(vectorAO_GTG[1], 2));    //Calculates the normailsation factor for the AvoidObstacle vector

  vectorAO_GTG[0] = 100 * vectorAO_GTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorAO_GTG[1] = 100 * vectorAO_GTG[1]/n;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

float calcGoalAngle(float vectX, float vectY)
{  
  float angle = atan2 (vectY, vectX);
  //if (angle <= 0) angle += (2*PI);
  return angle;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void drawTarget()
{
  stroke(0);
  fill(255, 0, 0);
  strokeWeight(1);
  ellipse (goalX, goalY, safeZone*3, safeZone*3);  
  stroke(0);
  fill(255);
  ellipse (goalX, goalY, safeZone*2, safeZone*2);
  stroke(0);
  fill(0);
  ellipse (goalX, goalY, safeZone, safeZone);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void dispVectors()
{ 
  strokeWeight(1); 
  stroke(0, 0, 255);
  
  strokeWeight(3);
  stroke (255,0,0);
  line (myRobot.x, myRobot.y, myRobot.x + vectorAvoidObstacles.x*100, myRobot.y + vectorAvoidObstacles.y*100);
  
  stroke (0,0,255);  //BLUE
  line (myRobot.x, myRobot.y, myRobot.x + vectorGoToGoal.x*100, myRobot.y + vectorGoToGoal.y*100);
  
  stroke (0,255,0);
  line (myRobot.x, myRobot.y, myRobot.x + vectorAOGTG.x*100, myRobot.y + vectorAOGTG.y*100);
  
  
  //path.x = avoid.x*1000 + vectorGTG[0];
  //path.y = avoid.y*1000 + vectorGTG[1];
  
  //stroke(0,255,255);
  //line (location.x, location.y, location.x + path.x, location.y + path.y);

  //stroke (255,0,0); 
  //line (myRobot.x, myRobot.y, myRobot.x + vectorAO[0], myRobot.y + vectorAO[1]); //draws the Avoid Obstacle vector 
  //stroke (0,255,0);
  //line (myRobot.x, myRobot.y, myRobot.x + vectorAO_GTG[0], myRobot.y + vectorAO_GTG[1]);  //draws the AO and GTG blended vector


  //line (closest1[0], closest1[1], closest1[0] +vectorWall[0], closest1[1] + vectorWall[1]);
  //line (myRobot.x, myRobot.y, myRobot.x+vectorWallDist[0], myRobot.y+vectorWallDist[1]);
  //line (myRobot.x, myRobot.y, myRobot.x+vectorAwayFromWall[0], myRobot.y+vectorAwayFromWall[1]);
  //stroke(255,0,0);
  //line (myRobot.x, myRobot.y, myRobot.x+vectorFollowWall[0],myRobot.y+vectorFollowWall[1]);

  //stroke(0);



  //strokeWeight(1);
  //stroke (0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//Rotates and translates an X and Y coordinate onto a local frame using the local frame's X,Y and HEADING
//Returns a PVector with new x and y coordinates
PVector transRot (float x_frame, float y_frame, float phi_frame, float x_point, float y_point)
{
  float x_temp = cos(phi_frame) * x_point - sin(phi_frame)*y_point + x_frame; //Uses transformation and rotation to plot sensor gloablly 
  float y_temp = sin(phi_frame) * x_point + cos(phi_frame)*y_point + y_frame;
  PVector result = new PVector(x_temp, y_temp);
  return result;
}
//==========================================================================
//
void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();
  if (mousePressed && (mouseButton == RIGHT)) 
  {
    myRobot.heading = 0.0;
    myRobot.x = mouseX;
    myRobot.y = mouseY;
    //Resets progress point when target is moved to the current mouse position
    progressPoint[0] = mouseX;
    progressPoint[1] = mouseY;
    makingProgress = true;
  }
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{
  goalX = mouseX;
  goalY = mouseY;
  startX = myRobot.x;
  startY = myRobot.y;  
  stateVal = 1; 
  //Resets progress point when target is moved to the current robot position
  progressPoint[0] = myRobot.state[0];
  progressPoint[1] = myRobot.state[1];
  makingProgress = true;
}

void keyPressed()
{
  if (key == ' ') showVal = true;

  if (key =='s') step = true;

  //Use this key to enable or diable obstacle
  if (key == 'o')
  {
    tile[int(mouseX/tileSize)][int(mouseY/tileSize)].gravity *= -1;    
    tile[int(mouseX/tileSize)][int(mouseY/tileSize)].update();
  }
}