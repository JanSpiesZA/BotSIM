PImage img;

float worldMapScaleX = 1000; //3737;      //To be used as the actual distance of the world map x axis, measured in cm
float worldMapScaleY = 1000; //1137;

float screenSizeX = 500;
float screenSizeY = screenSizeX * (worldMapScaleY/worldMapScaleX);  //Scale the y size according to ratio between worldMapScaleX an Y

float scaleFactor = screenSizeX / worldMapScaleX;

float ogXResolution = 100 * scaleFactor;          ///The resolution of a the grid of the occupancy map, measured in cm
float ogYResolution = 100 * scaleFactor;

int occupiedFlag = 0;            //Indicates if a grid in the occupied map must be completely covered
//boolean collisionFlag = false;
boolean makingProgress = true;    //Indicates if progress towards the goal is being made
boolean wallDetect = false;

int maxParticles = 10;
Robot[] particle = new Robot[maxParticles];
float noiseForward = 1.0;
float noiseTurn = 0.1;
float noiseSense = 5.0;

float moveSpeed = 0;
float moveAngle = 0;

Robot myrobot;
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0; //PI; //random(-PI,PI);
float turnGain = 0.1;
float moveGain = 0.01;
float blendGain = 0.5;      //Gain used when blending the AO and GTG vectors;
float normaliseGain = 100.0;

float safeZone = 0.0;          //Safe area around target assumed the robot reached its goal;
float safeDistance = 0.0;      //Closer than this value and the robot is too close to an obstacle
float distanceFromWall = 0.0;    //Distance that must be maintained when following the wall

float diameter = 45.0; 

float[] sensorX =   {0.0           , cos(PI/8*3)* diameter/2   , cos(PI/8*2)*diameter/2  , cos(PI/8)*diameter/2  , diameter/2 , cos(PI/8)*diameter/2, cos(PI/4)*diameter/2, cos(PI/8*3)*diameter/2, 0.0};      //Array containing all the sensors X values in the robot frame
float[] sensorY =   {-(diameter/2) , -sin(PI/8*3)* diameter/2  , -sin(PI/8*2)*diameter/2 , -sin(PI/8)*diameter/2 ,        0.0 , sin(PI/8)*diameter/2, sin(PI/4)*diameter/2, sin(PI/8*3)*diameter/2, diameter/2};
float[] sensorPhi = {-PI/2, -PI/8*3, -PI/8*2, -PI/8, 0.0 , PI/8, PI/4, PI/8*3, PI/2};
float[] sensorGains = {1.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.0};    //Gains used to indicate imprtance of sensor values

float[] vectorAO_GTG = {0.0, 0.0};    //x and y values for avoid obstacle and go-to-goal combined vector
float[] vectorAO = {0.0, 0.0};      //x and y values for avoid obstacle vector
float[] vectorGTG = {0.0, 0.0};      //x and y values for vector go-to-goal
float[] vectorWall = {0.0, 0.0};      //x and y values representing the vector of a piece of wall for follow wall procedure
float[] vectorWallDist = {0.0, 0.0};  //x and y values for a line perpendicular to the wall vector
float[] vectorAwayFromWall = {0.0, 0.0};  //x and y values for vector pointing away from the wall 
float[] vectorFollowWall = {0.0, 0.0};    //Vector pointing in the direction the robot must move when following the wall

int numSensors = sensorX.length;    //Determines the amount of sensor elements present
float[] sensorObstacleDist = new float[numSensors];
float minDetectDistance = 0.0;        //Closer than this value and the sensors do not return valid data
float maxDetectDistance = 0.0;

float x_temp = 0.0;        //Placeholder for temporary data from transRot function
float y_temp = 0.0;        //Placeholder for temporary data from transRot function

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

void setup()
{
  //All measurement values are in cm
  myrobot = new Robot("ROBOT");  
   
  myrobot.robotDiameter = diameter;
  robotX = screenSizeX / 2;
  robotY = screenSizeY / 2;  
  
  minDetectDistance = 10.0;        //Closer than this value and the sensors do not return valid data
  maxDetectDistance = 200.0;
  safeZone = 20.0;
  safeDistance = 20.0;    
  distanceFromWall = 50.0; 

  for (int i = 0; i < maxParticles; i++)
  {
    particle[i] = new Robot("PARTICLE");    
    particle[i].setNoise(noiseForward, noiseTurn, noiseSense);    //Add noise to newly created particle
  }  
  
  applyScale();
  
  img = loadImage("kamer2.png");
  img.resize((int)worldMapScaleX, (int)worldMapScaleY);
  img.filter(THRESHOLD);                              //Convert image to black and white
  img.resize(int(screenSizeX), int(screenSizeY));
  
  //size(int(screenSizeX), int(screenSizeY));  
  size(500,500);
  
  println (img.width);
  println (img.height);  
  
  println (numSensors);
  
  tint(255,127);
  image(img,0,0); 
  
  myrobot.set(robotX, robotY, robotTheta);  
  
  //Change particle x and y values to prevent them from being inside walls
  for (int i=0; i < maxParticles; i++)
  {  
    color col = img.get (int(particle[i].x) ,int(particle[i].y));    //Test pixel colour to determine if there is an obstacle
    if (red(col) == 0)
    {
      while(red(col) == 0)    
      {
        particle[i].x = random (0, screenSizeX);
        particle[i].y = random (0, screenSizeY);
        col = img.get (int(particle[i].x) ,int(particle[i].y));    //Test pixel colour to determine if there is an obstacle
      }
    }      
  }
}

void draw()
{
  background(img);                                  //Make the background the orginal map image
  drawTarget();
  PlotRobot();
  
  for (int i=0; i < maxParticles; i++)
  {
    particle[i].display();
  }
  updateParticles();
  
  
  calcProgressPoint();
  detectObstacle();
  calcVecAO();       //Calculates the avoid obstacle vector;
  calcVecGTG();
  calcVecAO_GTG();    //Calculates vector after blending Go-To-Goal and Avoid_Obstacle;
  estimateWall();    //Estimates the distance to the wall using closest sesnors to the wall  
  //dispVectors();      //Displays different vectors, ie: Go-To-Goal, Avoid Obstacle, etc
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
void updateParticles()
{
  for (int i = 0; i < maxParticles; i++)
  {
    particle[i].move(moveAngle, moveSpeed);    
  }
  
  
}
  
void applyScale()
{
  myrobot.robotDiameter *= scaleFactor;
  myrobot.noseLength *= scaleFactor;
  //myrobot.maxSpeed *= scaleFactor;      //I do not know why this must not be scaled???
  minDetectDistance *= scaleFactor;        //Closer than this value and the sensors do not return valid data
  maxDetectDistance *= scaleFactor;
  //robotX *= scaleFactor;
  //robotY *= scaleFactor;
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
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void PlotRobot()
{
  float difference = 0.0;
  
  float deltaX = goalX - myrobot.x;
  float deltaY = goalY - myrobot.y;  
  float targetAngle = atan2(deltaY, deltaX);    
  float distanceToTarget = sqrt(pow(deltaX,2) + pow(deltaY,2));
  
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
      }
      else
      {
        distanceToTarget = 0;
      }
    break;
    
    case 1:    //Go straight to goal
      //calcErrorAngle(phi_GTG);
      
      if (distanceToTarget <= safeZone)   //Robot is close enough to goal, stop robot
        stateVal = 0;
        
      if (myrobot.collisionFlag)
        stateVal = 2;      
      
      if (!myrobot.collisionFlag) 
        calcErrorAngle(phi_GTG);
        
      if ((!makingProgress) && (myrobot.collisionFlag))
        stateVal = 3;
    break;
    
    case 2:    //Avoid obstacle state
      calcErrorAngle(phi_AO);
      
      if(myrobot.collisionFlag)
      {
        stateVal = 1;
        myrobot.collisionFlag = false;
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
  //println (myrobot.heading, "\t", errorAngle,"\t",phi_AO);
  //println("Collisionflag :",collisionFlag, " Making Progress :", makingProgress);
  
    
  moveAngle = min (myrobot.maxTurnRate, (turnGain * errorAngle));  //P controller to turn towards goal
  moveSpeed = min (myrobot.maxSpeed ,(moveGain * (distanceToTarget))); 
  myrobot.move(moveAngle,moveSpeed);  
  myrobot.display();
  
  
  //moveSpeed = 1;
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//  Calculates a progress point towards the goal. If the progress point decreases, the robot progresses, else do Wall Following
void calcProgressPoint()
{
  float oldDist = sqrt(pow(goalX - progressPoint[0],2) + pow(goalY - progressPoint[1],2));    //Calculates the straight line distance to the goal
  float newDist = sqrt(pow(goalX - myrobot.state[0],2) + pow(goalY - myrobot.state[1],2));
  
  if (newDist <= oldDist)
  {
    progressPoint[0] = myrobot.state[0];
    progressPoint[1] = myrobot.state[1];
    makingProgress = true;
  }  
  else
  {
    makingProgress = false;
  }
  
  strokeWeight(5);
  stroke(0,255,255);
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
  errorAngle = goalAngle - myrobot.heading;
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
  
  int leftC1 = min(c1,c2);
  int leftC2 = max(c1,c2); 
  
  c1 = leftC1;
  c2 = leftC2;
  
  //step 2:
  //  Convert the distances to points in the global frame
  //  Create a wall vector and normalise it
  if ((c1 >= 0) & (c2 >= 0))
  { 
    transRot (sensorX[c1], sensorY[c1], sensorPhi[c1], sensorObstacleDist[c1], 0);    //translates obstacle distance to robot frame
    transRot (myrobot.x, myrobot.y, myrobot.heading, x_temp, y_temp);  //translates sensordata in robot frame to global frame
    closest1[0] = x_temp;
    closest1[1] = y_temp;
    
    transRot (sensorX[c2], sensorY[c2], sensorPhi[c2], sensorObstacleDist[c2], 0);    //translates obstacle distance to robot frame
    transRot (myrobot.x, myrobot.y, myrobot.heading, x_temp, y_temp);  //translates sensordata in robot frame to global frame
    closest2[0] = x_temp;
    closest2[1] = y_temp;
         
    ellipse (closest1[0], closest1[1], 20,20);
    ellipse (closest2[0], closest2[1], 20,20);
    //line (closest1[0], closest1[1], closest2[0], closest2[1]);
    
    vectorWall[0] = closest2[0] - closest1[0];
    vectorWall[1] = closest2[1] - closest1[1];
    
    float n = sqrt(pow(vectorWall[0],2)+pow(vectorWall[1],2));    //Calculates the normalisation factor for the AvoidObstacle vector  
    vectorWall[0] = normaliseGain * vectorWall[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
    vectorWall[1] = normaliseGain * vectorWall[1]/n;  
    
    //Step 3
    //  Compute a vector perpendicular with the wall pointing from the center of the robot to the wall vector    
    //  http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
    
    float k = ((closest2[1] - closest1[1]) * (myrobot.x - closest1[0]) - (closest2[0] - closest1[0])*(myrobot.y - closest1[1])) / (pow(closest2[1]-closest1[1],2) + pow(closest2[0]-closest1[0],2));
    vectorWallDist[0] = myrobot.x - k *(closest2[1] - closest1[1]);
    vectorWallDist[1] = myrobot.y + k *(closest2[0] - closest1[0]);
    
    vectorWallDist[0] -= myrobot.x;
    vectorWallDist[1] -= myrobot.y;    
    
    n = sqrt(pow(vectorWallDist[0],2)+pow(vectorWallDist[1],2));    //Calculates the normalisation factor for the AvoidObstacle vector
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
 
 for (int i = 0; i < numSensors; i++)
 {
   sensorObstacleDist[i] = 2; //myrobot.diameter/2 + 1;    //Set starting point of collision detect to 1 pixel wider than the radius of the robot    
   obstacleFlag = false;
   
   while ((obstacleFlag == false) && (sensorObstacleDist[i] < maxDetectDistance))
   {
     transRot(myrobot.x, myrobot.y, myrobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
     obstacleX = x_temp + sensorObstacleDist[i] * cos(myrobot.heading + sensorPhi[i]);
     obstacleY = y_temp + sensorObstacleDist[i] * sin(myrobot.heading + sensorPhi[i]);   
     color col = img.get (int(obstacleX), int(obstacleY));    //Test pixel colour to determine if there is an obstacle
     if (red(col) == 0)
       obstacleFlag = true;
     sensorObstacleDist[i] += 1;      
   }
   if (sensorObstacleDist[i] <= safeDistance) myrobot.collisionFlag = true;      //Set collision flag when any sensor is too close to obstacle
   ellipse (obstacleX, obstacleY, 10*scaleFactor,10*scaleFactor);
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
    transRot(myrobot.x, myrobot.y, myrobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
    x1_vector = x_temp;
    y1_vector = y_temp;
    
    transRot (sensorX[i], sensorY[i], sensorPhi[i], sensorObstacleDist[i], 0);    //translates obstacle distance to robot frame
    transRot (myrobot.x, myrobot.y, myrobot.heading, x_temp, y_temp);  //translates sensordata in robot frame to global frame
    x_vector = x_temp - x1_vector;
    y_vector = y_temp - y1_vector;
    
    //Add all the vectors together and multiply with vector gains
    vectorAO[0] += (x_vector * sensorGains[i]);
    vectorAO[1] += (y_vector * sensorGains[i]);  
    
    //line (myrobot.x, myrobot.y, x_temp, y_temp);
  }
  float n = sqrt(pow(vectorAO[0],2)+pow(vectorAO[1],2));    //Calculates the normalisation factor for the AvoidObstacle vector
  
  vectorAO[0] = 100 * vectorAO[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorAO[1] = 100 * vectorAO[1]/n;  
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecGTG()
{
  vectorGTG[0] = goalX - myrobot.x;
  vectorGTG[1] = goalY - myrobot.y;
  
  float n = sqrt(pow(vectorGTG[0],2)+pow(vectorGTG[1],2));    //Calculates the normailsation factor for the AvoidObstacle vector
  if (n==0) n=1;
  vectorGTG[0] = 100 * vectorGTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorGTG[1] = 100 * vectorGTG[1]/n;   
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void calcVecAO_GTG()
{       
    
  for (int i = 0; i <= 1; i++)
  {
    vectorAO_GTG[i] = blendGain*vectorAO[i] + (1 - blendGain)*vectorGTG[i];
  }  
  
  float n = sqrt(pow(vectorAO_GTG[0],2)+pow(vectorAO_GTG[1],2));    //Calculates the normailsation factor for the AvoidObstacle vector
  
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
  fill(255,0,0);
  strokeWeight(1);
  ellipse (goalX,goalY, safeZone*3,safeZone*3);  
  stroke(0);
  fill(255);
  ellipse (goalX,goalY, safeZone*2,safeZone*2);
  stroke(0);
  fill(0);
  ellipse (goalX,goalY, safeZone,safeZone);  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void dispVectors()
{ 
 strokeWeight(5); 
 stroke(0,0,255);
 
 line (myrobot.x, myrobot.y, goalX, goalY);          //draws a line from the robot x,y to the goal loaction 
 stroke (255,0,0); 
 line (myrobot.x, myrobot.y, myrobot.x + vectorAO[0], myrobot.y + vectorAO[1]); //draws the Avoid Obstacle vector 
 stroke (0,255,0);
 line (myrobot.x, myrobot.y, myrobot.x + vectorAO_GTG[0], myrobot.y + vectorAO_GTG[1]);  //draws the AO and GTG blended vector
 
 
 line (closest1[0], closest1[1], closest1[0] +vectorWall[0], closest1[1] + vectorWall[1]);
 line (myrobot.x, myrobot.y, myrobot.x+vectorWallDist[0], myrobot.y+vectorWallDist[1]);
 line (myrobot.x, myrobot.y, myrobot.x+vectorAwayFromWall[0], myrobot.y+vectorAwayFromWall[1]);
 stroke(255,0,0);
 line (myrobot.x, myrobot.y, myrobot.x+vectorFollowWall[0],myrobot.y+vectorFollowWall[1]);
 
 stroke(0);
 
 
 
 strokeWeight(1);
 stroke (0); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//Rotates and translates an X and Y coordinate onto a local frame using the local frame's X,Y and HEADING
//Returns x_temp and y_temp which must be allocated to other relevant variables in order to prevent overwriting of their values
void transRot (float x_frame, float y_frame, float phi_frame, float x_point, float y_point)
{
  x_temp = cos(phi_frame) * x_point - sin(phi_frame)*y_point + x_frame; //Uses transformation and rotation to plot sensor gloablly 
  y_temp = sin(phi_frame) * x_point + cos(phi_frame)*y_point + y_frame;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void convertToOG()
{
   //Draw the vertical and horisontal occupacy grid lines
  for (int cntrh = 0; cntrh < worldMapScaleX; cntrh += ogXResolution)
    line (0+cntrh,0,0+cntrh,worldMapScaleY);
  for (int cntrv = 0; cntrv < worldMapScaleY; cntrv += ogYResolution)
    line (0,0+cntrv,worldMapScaleX,0+cntrv);
  
  
  for (int cntrh = 0; cntrh < worldMapScaleX; cntrh += ogXResolution)
  {
    for (int cntrv = 0; cntrv < worldMapScaleX; cntrv += ogYResolution)
    {
      PImage imgOG = img.get(cntrh,cntrv, int(ogXResolution), int(ogYResolution));
      imgOG.loadPixels();
      
      for (int i = 0; i < (imgOG.width*imgOG.height); i++)
      {
        color c = imgOG.pixels[i];
        //float redval = red(c);
        
        if (red(c) == 0)
        {
          occupiedFlag = 1;          
        }        
      }
      if (occupiedFlag == 1)
      {
        fill(128);
        rect(cntrh,cntrv,ogXResolution,ogYResolution);
        occupiedFlag = 0;
      }
    }
  } 
}
//==========================================================================
//

void keyPressed()
{
  switch (key)
  {
    case 'q':
      scaleFactor -= 0.1;
      applyScale();
      break;
      
    case 'a':
      scaleFactor += 0.1;      
      applyScale();
      break;
  }
}


void mouseWheel(MouseEvent event) 
{
  float e = event.getCount();
  if (e == -1) scaleFactor = 0.9;
  if (e == 1) scaleFactor = 1.1;
  applyScale();
  println(e);
}

void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();
  if (mousePressed && (mouseButton == RIGHT)) 
  {
    myrobot.heading = 0.0;
    myrobot.x = mouseX;
    myrobot.y = mouseY;
    progressPoint[0] = myrobot.state[0];
    progressPoint[1] = myrobot.state[1];
  }
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{
  goalX = mouseX;
  goalY = mouseY;
  startX = myrobot.x;
  startY = myrobot.y;  
 stateVal = 1; 
}