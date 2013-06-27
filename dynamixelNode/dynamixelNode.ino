/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <DynamixelSerial2.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>



/*--- typedefs ---------------------------------------*/ 

typedef enum   {
  SCAN_INIT_UP=1,
  SCAN_UP=2,
  SCAN_INIT_DOWN=3,
  SCAN_DOWN=4,
  SCAN_STOP_INIT=5,
  SCAN_STOP=6
} scanState;


scanState currentScanState =  SCAN_INIT_UP;
scanState nextScanState    =  SCAN_UP;
scanState backUPState    =  SCAN_INIT_UP;;

const float degUnit = 0.29;
const long  timerUnit = 40000; // Equals 20 Milliseconds -> Doule the Amount of Postion inforamtion than Laser scans 25Hz
const int   lowerBoundry = 650; // Equals -40 degree down
const int   upperBoundry = 374;

ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::Float32 int_msg;

char hello[13] = "hello world!";
char charBuf[5];
volatile float Position;
String tempStr;

boolean startScan = false;

void axScan( const std_msgs::Bool& bool_msg){
 startScan = bool_msg.data;  // blink the led
}

ros::Publisher axservo("axservo", &int_msg);
ros::Subscriber<std_msgs::Bool> sub("ax_scan", &axScan );

void setup()
{
  nh.initNode();
  nh.advertise(axservo);
  nh.subscribe(sub);
  Dynamixel.begin(1000000,2);
  Dynamixel.setEndless(13,OFF); 
  Dynamixel.moveSpeedRW(13,512,30);
  Dynamixel.action();
  delay(2000);
//  Position = Dynamixel.readPosition(13); 
  // Initialize timer
  //Timer1.initialize(timerUnit);
  //Timer1.attachInterrupt(getPosition);
   Position = 0;
}


void loop()
{
   
  Position = Dynamixel.readPosition(13); 
  

  
  if (!((((((Position*degUnit)-150) * 71) / 4068) < 1) && (((((Position*degUnit)-150) * 71) / 4068) > -1))) 
  {
     Position = Dynamixel.readPosition(13); 
  } else
  {
      // Convert Postion data into rad and put it into the datafield of the message
      int_msg.data = (((Position*degUnit)-150) * 71) / 4068; 
      axservo.publish( &int_msg );
      runScanStatemachine(lowerBoundry, upperBoundry, 10, Position, startScan);
  }
  

      //if ((((((Position*degUnit)-150) * 71) / 4068) < 1) && (((((Position*degUnit)-150) * 71) / 4068) > -1)) {
  delay(100);
  nh.spinOnce();

      //}

    //noInterrupts();
 
  // interrupts();
 //  Dynamixel.action();
  //tempStr = String((Position*degUnit)-150);
  //tempStr.toCharArray(charBuf, 5);
  //str_msg.data = charBuf;
  


}


// ---------- Command Statemachine for Dynamixel Commands -------------
void runScanStatemachine (long maxUP, long maxDown, int maxDelta, float fPosition, boolean bstartScan) {


switch (currentScanState) {
  case SCAN_INIT_UP:
    
      if ((fPosition <= (maxUP - maxDelta)) && bstartScan == true )  {
        Dynamixel.moveSpeed(13,maxUP,50);
        nextScanState=SCAN_UP;
      } else {
        nextScanState=SCAN_INIT_DOWN;
      }
      
      if (bstartScan == false) {
        nextScanState = SCAN_STOP;
         backUPState = SCAN_INIT_UP; 
      }
    break;
  
  case SCAN_UP:
 // Serial.print("UP :");

      if (fPosition <= (maxUP - maxDelta)) {
        
          nextScanState=SCAN_UP;
      }
      else {
        nextScanState=SCAN_INIT_DOWN;
      }
      
      if (bstartScan == false) {
        nextScanState = SCAN_STOP;
            backUPState = SCAN_UP;
      }
    break;
  
  case SCAN_INIT_DOWN:
      if (fPosition >= (maxDown + maxDelta)) {
        Dynamixel.moveSpeed(13,maxDown,50);
        nextScanState=SCAN_DOWN;
      } else {
        nextScanState=SCAN_INIT_UP;
      }
      
            if (bstartScan == false) {
              nextScanState = SCAN_STOP;
            backUPState = SCAN_INIT_DOWN;  
          }
    break;
    
    
  case SCAN_DOWN:

        if ((fPosition >= (maxDown + maxDelta)) && bstartScan == true) {
     
        nextScanState=SCAN_DOWN;
      } else {
        nextScanState=SCAN_INIT_UP;
      }
            if (bstartScan == false) {
              nextScanState = SCAN_STOP;
              backUPState = SCAN_DOWN; 
          }
    break;
    
  case SCAN_STOP_INIT:
        if (bstartScan == false) {
          Dynamixel.moveSpeed(13,fPosition,1);
          nextScanState = SCAN_STOP;
        }
        else {
         nextScanState = backUPState;
      }
    break;
    
    case SCAN_STOP:
             if (bstartScan == false) {
              nextScanState = SCAN_STOP;
        }
        else {
         nextScanState = backUPState;
      }
    
    break;
  
}
currentScanState = nextScanState;
}

