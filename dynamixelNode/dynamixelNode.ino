/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <DynamixelSerial2.h>

const float degUnit = 0.29;
ros::NodeHandle  nh;

std_msgs::String str_msg;
std_msgs::Float32 int_msg;
ros::Publisher axservo("axservo", &int_msg);

char hello[13] = "hello world!";
char charBuf[5];
float Position;
String tempStr;
void setup()
{
  nh.initNode();
  nh.advertise(axservo);
  Dynamixel.begin(1000000,2);
  Dynamixel.setEndless(13,OFF); 
  Dynamixel.moveSpeedRW(13,600,30);
    Dynamixel.action();
   
}

void loop()
{
   
  Position = Dynamixel.readPosition(13);  
if (Position >= 600) {
    Dynamixel.moveSpeedRW(13,500,30);
    Dynamixel.action();
} else if (Position <= 500) {
  Dynamixel.moveSpeedRW(13,600,30);
    Dynamixel.action();
}

 //  Dynamixel.action();
  //tempStr = String((Position*degUnit)-150);
  //tempStr.toCharArray(charBuf, 5);
  //str_msg.data = charBuf;
  int_msg.data = (Position*degUnit)-150;
  axservo.publish( &int_msg );
  nh.spinOnce();
  
}
