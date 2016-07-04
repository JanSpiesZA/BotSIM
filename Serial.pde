import processing.serial.*;
Serial myPort;

int lf = 10;
int cr = 13;

String inData = null;
String[] list = null;

void requestSerialPosition()
{
  println("TXING!!!");
  myPort.write("<?" + "\r");  
}

void updateRobot(float _moveAngle)
{
  println("TXING!!!");
  myPort.write("<w"+_moveAngle*1000+"\r");  
}

void parseSerialData()
{
  if (inData != null)
  {
    switch (inData.charAt(0))
    {
      case '?':
      {
        inData = inData.substring(1);
        println(inData);
        list = split(inData, ",");
        
        //Add robot real world position to inital robot position
        myRobot.location.x = (float(list[0]) + robotStart.x) * scaleFactor;
        myRobot.location.y = (float(list[1]) + robotStart.y) * scaleFactor;
        myRobot.heading = (float(list[2]) +robotStart.z) * scaleFactor;         
        break;
      }
      
      case 's':
      {
        break;
      }
    }
  }
}

void serialEvent(Serial p)
{
  inData = p.readString();
}