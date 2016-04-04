class Sensor
{
  float sensorXPos = 0.0;      //x position relative to robot centre point
  float sensorYPos = 0.0;      //y position relative to robot center point
  float sensorHAngle = 0.0;    //Angle of sensor in horizontal plane relative to robot chassis. IE rotated left or right
  float sensorVAngle = 0.0;    //Angle of sensor in vertical plane relative to robot chassis. IE tilted up or down
  float sensorGain = 0.0;      //Gains used to indicate importance of sensor
  float sensorObstacleDist = 0.0;
  float sensorMaxDetect = 0.0;
  float sensorMinDetect = 0.0;
  
  Sensor()
  {
  }
  
  void display()
  {
  }
  
}