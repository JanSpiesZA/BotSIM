//All distances are measured and listed in cm's unless specified otherwise

PImage img;

//Actual distance of measured on ground, measured in cm's, where one pixel = 1cm???
float worldMapScaleX = 1000; //3737;      //To be used as the actual distance of the world map x axis, measured in cm
float worldMapScaleY = 1000; //1137;

float screenSizeX = 500;
float screenSizeY = screenSizeX * (worldMapScaleY/worldMapScaleX);  //Scale the y size according to ratio between worldMapScaleX an Y

float scaleFactor = screenSizeX / worldMapScaleX;

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

//safeDistance cannot be less than minDetectDistance
int safeDistance = 50;      //If sensor measured distance is less than this value, the robot is too close to an obstacle
float distanceFromWall = 50.0;    //Distance that must be maintained when following the wall


//This section must be removed when only the sensor class is used
float[] sensorX =   {0.0, cos(PI/8*3)* diameter/2, cos(PI/8*2)*diameter/2, cos(PI/8)*diameter/2, diameter/2, cos(PI/8)*diameter/2, cos(PI/4)*diameter/2, cos(PI/8*3)*diameter/2, 0.0};      //Array containing all the sensors X values in the robot frame
float[] sensorY =   {-(diameter/2), -sin(PI/8*3)* diameter/2, -sin(PI/8*2)*diameter/2, -sin(PI/8)*diameter/2, 0.0, sin(PI/8)*diameter/2, sin(PI/4)*diameter/2, sin(PI/8*3)*diameter/2, diameter/2};
float[] sensorPhi = {-PI/2, -PI/8*3, -PI/8*2, -PI/8, 0.0, PI/8, PI/4, PI/8*3, PI/2};
float[] sensorGains = {1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.0};    //Gains used to indicate imprtance of sensor values
int numSensors = sensorX.length;    //Determines the amount of sensor elements present
float[] sensorObstacleDist = new float[numSensors];

float[] vectorAO_GTG = {0.0, 0.0};    //x and y values for avoid obstacle and go-to-goal combined vector
float[] vectorAO = {0.0, 0.0};      //x and y values for avoid obstacle vector
float[] vectorGTG = {0.0, 0.0};      //x and y values for vector go-to-goal
float goalX = screenSizeX / 2;            //Goal's X and Y coordinates, set up by clicking with the mouse on the screen
float goalY = screenSizeY / 2;
//This section must be removed when only the sensor class is used




PVector vectorAOGTG = new PVector();
PVector vectorAvoidObstacles = new PVector();
PVector coordsAvoidObstacles = new PVector();    //Coords on the world frame, holding the point of the avoid obstacle vector
PVector vectorGoToGoal = new PVector();
PVector vectorBlendedAOGTG = new PVector();      //Holds the vector which is blended between AvoidObstacles an GoToGoal

float[] vectorWall = {0.0, 0.0};      //x and y values representing the vector of a piece of wall for follow wall procedure
float[] vectorWallDist = {0.0, 0.0};  //x and y values for a line perpendicular to the wall vector
float[] vectorAwayFromWall = {0.0, 0.0};  //x and y values for vector pointing away from the wall
float[] vectorFollowWall = {0.0, 0.0};    //Vector pointing in the direction the robot must move when following the wall


int numSensors2 = 9;

int minDetectDistance = 10;        //Closer than this value and the sensors do not return valid data
float maxDetectDistance = 200.0;


PVector goalXY = new PVector(screenSizeX / 2, screenSizeY / 2);       //Holds the goal's x and y coords
float startX = 0;          //Starting point for straight line to goal used by Bug algorithm families
float startY = 0;
float x_vector_avoid = 0.0;
float y_vector_avoid = 0.0;
float phi_avoid = 0.0;
float errorAngle = 0.0;

float[] closest1 = {0.0, 0.0};
float[] closest2 = {0.0, 0.0};

int stateVal = 0;      //Values used to indicate which state the robot is currently in

boolean showVal = false;
boolean step = true;

//Measurement of tiles to be used for occupancy grid in cm's scaled to represented size in real world
int tileSize = 50;
int maxTilesX = 0;
int maxTilesY = 0;
Tile tile[][];

void setup()
{
  //img = loadImage("blank.png");         //Loads image
  img = loadImage("kamer2.png");         //Loads image
  img.resize(int(screenSizeX), int(screenSizeY));
  
  tileSize *= scaleFactor;        //Aplies scale factor to the tile size
    
  maxTilesX = ceil((float(img.width)/float(tileSize)));
  maxTilesY = ceil((float(img.height)/float(tileSize)));  
  
  println("img.width : "+img.width+", img.Height: "+img.height);
  println(maxTilesX+","+maxTilesY);
  
  tile = new Tile[int(maxTilesX)][int(maxTilesY)];
  
  //Sets up a 2D array which will hold the world Tiles
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      tile[x][y] = new Tile();
    }
  }
  
  //Scans the pixels of the background image to build the occupancy grid
  img.filter(THRESHOLD);              //Convert image to greyscale
  for (int x = 0; x < screenSizeX; x++)
  {
    for (int y = 0; y < screenSizeY; y++)
    {
      color c = img.get(x,y);
      if (c == color(0))
      {         
        tile[x/tileSize][y/tileSize].gravity = 1;  
        tile[x/tileSize][y/tileSize].update();
      }      
    }
  } 

  
  //-------------------------------------------------------------------------------
  //Initialise Robot
  myRobot = new Robot("ROBOT", diameter);        //Create a new robot object
  myRobot.set(screenSizeX/2, screenSizeY/2, -PI/2);

  //Add sensors to the robot object
  for (int k=0; k<numSensors2; k++)
  {
    myRobot.addSensor(0, 0, -PI/2 + PI/(numSensors-1)*k);
    
    //myRobot.addSensor(0,0,0);
    myRobot.sensors.get(k).sensorMinDetect = minDetectDistance;
  }
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  
  //-------------------------------------------------------------------------------
  //Create particles to localise robot
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
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
  background (img);
  
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
   drawTiles();
   drawTarget();
   PlotRobot();
   calcProgressPoint();
    
   int startTime = millis();
   myRobot.sense();          //Makes use of sensor class to detect obstacles

   for (int k = 0; k < maxParticles; k++)
   {
     particles[k].sense();
     particles[k].measureProb();
   }
    
   int endTime = millis();
   println(endTime - startTime);

   updateParticles();
   resample();



   step = true;

  vectorAvoidObstacles = calcVectorAvoidObstacles();
  vectorGoToGoal = calcVectorGoToGoal();  
  //vectorAOGTG = PVector.add(vectorGoToGoal, vectorAvoidObstacles);
  vectorBlendedAOGTG = calculateVectorBlendedAOGTG();
  //println(vectorGoToGoal+" : "+vectorAvoidObstacles+" : "+vectorAOGTG);

  }
  
  //estimateWall();    //Estimates the distance to the wall using closest sesnors to the wall
  //dispVectors();      //Displays different vectors, ie: Go-To-Goal, Avoid Obstacle, etc
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//Draws the world using tiles
void drawTiles()
{
  for (int x = 0; x < maxTilesX; x++)
  {
    for (int y = 0; y < maxTilesY; y++)
    {
      stroke(0);        //Lines between tiles are black
      strokeWeight(1);  //Stroke weight makes the lines very light
      fill(tile[x][y].gravityCol,200);
      rect(x*tileSize, y*tileSize, tileSize, tileSize);  //Draws a rectangle to indicate the tile
    }
  }
}

//###############################################################################################
//Updates each particle accoridng to robot movement
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
//Applies a scale factor to all the values used according to the actual world size and the displayed screen size

//MUST BE CLEANED
void applyScale()
{
  myRobot.robotDiameter *= scaleFactor;
  myRobot.noseLength *= scaleFactor;
  myRobot.maxSpeed *= scaleFactor;
  myRobot.maxTurnRate *= scaleFactor;
  minDetectDistance *= scaleFactor;        //Closer than this value and the sensors do not return valid data
  maxDetectDistance *= scaleFactor;
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
  
  //Applies scale factor to each particle
  for (int k = 0; k < maxParticles; k++)
  {
    for (int i = 0; i < numSensors2; i++)
    {
      particles[k].sensors.get(i).sensorXPos *= scaleFactor;
      particles[k].sensors.get(i).sensorYPos *= scaleFactor;
      particles[k].sensors.get(i).sensorMaxDetect *= scaleFactor;
    }
  }
}
//###############################################################################################
//Main FSM for robot movement and decisions
void PlotRobot()
{  
  float distanceToTarget = PVector.dist(goalXY, myRobot.location);

  float phi_GTG = calcGoalAngle(vectorGoToGoal.x, vectorGoToGoal.y);
  float phi_AO = calcGoalAngle(vectorAvoidObstacles.x, vectorAvoidObstacles.y);
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

    if ((!myRobot.makingProgress) && (myRobot.collisionFlag))
    {
      stateVal = 2;
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
    if (myRobot.makingProgress) stateVal = 1;
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
  // float oldDist = sqrt(pow(goalX - progressPoint[0], 2) + pow(goalY - progressPoint[1], 2));    //Calculates the straight line distance to the goal
  // float newDist = sqrt(pow(goalX - myRobot.state[0], 2) + pow(goalY - myRobot.state[1], 2));
  float oldDist = PVector.dist(goalXY, myRobot.progressPoint);
  float newDist = PVector.dist(goalXY, myRobot.location);
  
  if (newDist <= oldDist)
  {
    myRobot.progressPoint = myRobot.location.copy();    
    myRobot.makingProgress = true;  
  } else
  {
    myRobot.makingProgress = false;    
  }

  strokeWeight(5);
  stroke(0, 255, 255);  
  ellipse(myRobot.progressPoint.x, myRobot.progressPoint.y, 5, 5);
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
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
    closest1[0] = returnVal.x;
    closest1[1] = returnVal.y;

    returnVal = transRot (sensorX[c2], sensorY[c2], sensorPhi[c2], sensorObstacleDist[c2], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
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

    float k = ((closest2[1] - closest1[1]) * (myRobot.location.x - closest1[0]) - (closest2[0] - closest1[0])*(myRobot.location.y - closest1[1])) / (pow(closest2[1]-closest1[1], 2) + pow(closest2[0]-closest1[0], 2));
    vectorWallDist[0] = myRobot.location.x - k *(closest2[1] - closest1[1]);
    vectorWallDist[1] = myRobot.location.y + k *(closest2[0] - closest1[0]);

    vectorWallDist[0] -= myRobot.location.x;
    vectorWallDist[1] -= myRobot.location.y;

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
PVector calcVectorAvoidObstacles()
{
  PVector tempCoords = new PVector();   
  PVector vectorAO = new PVector();
  
  for (int k = 0; k < myRobot.sensors.size(); k++)
  {
    //TransRotate sensor distance value to robot frame
    tempCoords = transRot(myRobot.sensors.get(k).sensorXPos, myRobot.sensors.get(k).sensorYPos, myRobot.sensors.get(k).sensorHAngle, myRobot.sensors.get(k).sensorObstacleDist, 0);
    
    //Add all the x's and y's together to get combined vector of avoid obstacles        
    vectorAO.add(tempCoords);
  }
  //vectorAO.normalize();
  //println(vectorAO.mag());
  //Transrotate avoidObstacles coords into world frame with a scaling factor
  tempCoords = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, vectorAO.x, vectorAO.y);
  
  tempCoords.x = tempCoords.x - myRobot.location.x;
  tempCoords.y = tempCoords.y - myRobot.location.y;
  
  return tempCoords;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
PVector calcVectorGoToGoal()
{
  PVector result = new PVector();
  result.x = goalX - myRobot.location.x;
  result.y = goalY - myRobot.location.y;  
  return result; //.normalize();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
PVector calculateVectorBlendedAOGTG()
{
  PVector result = new PVector();
  float dist = vectorAvoidObstacles.mag();
  float beta = 0.002;          //The smaller this value gets the smaller sigma becomes
  float sigma = 1 - exp(-beta*dist);
  PVector gtgBlend = new PVector();
  PVector aoBlend = new PVector();  
  
  PVector.mult(vectorGoToGoal,sigma, gtgBlend);
  PVector.mult(vectorAvoidObstacles, (1-sigma), aoBlend); 

  result = PVector.add(gtgBlend, aoBlend);
  return result;
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
  strokeWeight(3);
  stroke (255,0,0);  //RED  
  line (myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorAvoidObstacles.normalize().x*100, myRobot.location.y + vectorAvoidObstacles.normalize().y*100);

  stroke (0,0,255);  //BLUE
  line (myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorGoToGoal.normalize().x*100, myRobot.location.y + vectorGoToGoal.normalize().y*100);

  //stroke (0,255,0);  //GREEN
  //line (myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorAOGTG.x*100, myRobot.location.y + vectorAOGTG.y*100);
  
  stroke (0,255,255);
  line (myRobot.location.x, myRobot.location.y, myRobot.location.x + vectorBlendedAOGTG.normalize().x*100, myRobot.location.y + vectorBlendedAOGTG.normalize().y*100);


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
    myRobot.location.x = mouseX;
    myRobot.location.y = mouseY;   

    //Resets progress point when target is moved to the current mouse position    
    myRobot.progressPoint.x = mouseX;
    myRobot.progressPoint.y = mouseY;
    myRobot.makingProgress = true;
  }
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{
  goalX = mouseX;
  goalY = mouseY;
  goalXY.x = mouseX;
  goalXY.y = mouseY;

  startX = myRobot.location.x;
  startY = myRobot.location.y;

  stateVal = 1;
  
  //Resets progress point when target is moved to the current robot position  
  myRobot.progressPoint = myRobot.location;
  myRobot.makingProgress  = true;
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