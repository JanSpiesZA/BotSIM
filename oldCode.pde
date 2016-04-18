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
      PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
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
    PVector returnVal = transRot(myRobot.location.x, myRobot.location.y, myRobot.heading, sensorX[i], sensorY[i]);  //translates sensordata to global frame
    x1_vector = returnVal.x; //x_temp;
    y1_vector = returnVal.y; //y_temp;

    returnVal = transRot (sensorX[i], sensorY[i], sensorPhi[i], sensorObstacleDist[i], 0);    //translates obstacle distance to robot frame
    returnVal = transRot (myRobot.location.x, myRobot.location.y, myRobot.heading, returnVal.x, returnVal.y);  //translates sensordata in robot frame to global frame
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
void calcVecGTG()
{
  vectorGTG[0] = goalX - myRobot.location.x;
  vectorGTG[1] = goalY - myRobot.location.y;

  float n = sqrt(pow(vectorGTG[0], 2)+pow(vectorGTG[1], 2));    //Calculates the normailsation factor for the AvoidObstacle vector
  if (n==0) n=1;
  vectorGTG[0] = 100 * vectorGTG[0]/n;      //Multiply by 100 gain in order to control the length of the unity vectors
  vectorGTG[1] = 100 * vectorGTG[1]/n;
  
  
}