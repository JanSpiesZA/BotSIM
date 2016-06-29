
// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void drawPixels()
{
 // Get the raw depth as array of integers
 int[] depth = kinect.getRawDepth();
  
 for (int x = 0; x < kinect.width; x += skip) 
 {
   for (int y = 0; y < kinect.height; y += skip) 
   {
    int offset = x + y*kinect.width;

     // Convert kinect data to world xyz coordinate
     int rawDepth = depth[offset];
     PVector v = depthToWorld(x, y, rawDepth);
     
    v.x *= 100;      //Convert depth value from meters into mm
    v.z *= 100;
    
    v.x *= scaleFactor;  //Multiply values with scale factor 
    v.z *= scaleFactor;
         
     //transRot(robotPos.x, robotPos.y, robotPos.z, v.x, -v.z);     
     
     if (v.z > 0 && v.z < maxKinectDetectNormal*scaleFactor)    //Test for any invalid depth values      
     {
        fill(255);     
        //ellipse((x_temp*scaleFactor),y_temp*scaleFactor,5,5);
        
        ellipse(myRobot.location.x + v.x, myRobot.location.y - v.z ,5,5);
        //updateGravity(robotPos.x + v.x,robotPos.y - v.z );
        
        //transRot(robotPos.x, robotPos.y, robotPos.z, sensorX, sensorY);
        //ellipse(x_temp,y_temp,10,10);
     }
   }
 }  
}