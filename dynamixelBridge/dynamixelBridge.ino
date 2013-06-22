/**********************************************************************\
* Dateiname: AX18ServoProxy.c
* Autor : Mario Grotschar
*
* Projekt : Proxy for the AX library
* Copyright (C) <<COPYRIGHT>>
*t
* Kurzbeschreibung: Beinhaltet die Funktionen zum Erzeugen und Traversieren
*					eines Labyrinth.
*
* Datum: Autor: Grund der Aenderung:
* 21.06.2013 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/

/*--- #includes of type <...> ---------------------------------------*/

#include <DynamixelSerial2.h>

/*--- #defines ---------------------------------------*/

#define DEBUG 1
#define MAXARRAYSIZE 10 // Allow a maximum of 10 parameers

// Macro for the selection of the Serial Port

#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define availableData() (Serial.available())    // Check Serial Data Available
#define readData()      (Serial.read())         // Read Serial Data
#define peekData()      (Serial.peek())         // Peek Serial Data
#define beginCom(args)  (Serial.begin(args))    // Begin Serial Comunication
#define endCom()        (Serial.end())          // End Serial Comunication

// Macro for Timing

#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// Dynamixel Constants

// Control Table Constants
const byte DXL_INITSEQ = 255;
const byte DXL_MODEL_NUMBER_L = 0;
const byte DXL_MODEL_NUMBER_H = 1;
const byte DXL_VERSION = 2;
const byte DXL_ID = 3;
const byte DXL_BAUD_RATE = 4;
const byte DXL_RETURN_DELAY_TIME = 5;
const byte DXL_CW_ANGLE_LIMIT_L = 6;
const byte DXL_CW_ANGLE_LIMIT_H = 7;
const byte DXL_CCW_ANGLE_LIMIT_L = 8;
const byte DXL_CCW_ANGLE_LIMIT_H = 9;
const byte DXL_DRIVE_MODE = 10;
const byte DXL_LIMIT_TEMPERATURE = 11;
const byte DXL_DOWN_LIMIT_VOLTAGE = 12;
const byte DXL_UP_LIMIT_VOLTAGE = 13;
const byte DXL_MAX_TORQUE_L = 14;
const byte DXL_MAX_TORQUE_H = 15;
const byte DXL_RETURN_LEVEL = 16;
const byte DXL_ALARM_LED = 17;
const byte DXL_ALARM_SHUTDOWN = 18;
const byte DXL_OPERATING_MODE = 19;
const byte DXL_DOWN_CALIBRATION_L = 20;
const byte DXL_DOWN_CALIBRATION_H = 21;
const byte DXL_UP_CALIBRATION_L = 22;
const byte DXL_UP_CALIBRATION_H = 23;
const byte DXL_TORQUE_ENABLE = 24;
const byte DXL_LED = 25;
const byte DXL_CW_COMPLIANCE_MARGIN = 26;
const byte DXL_CCW_COMPLIANCE_MARGIN = 27;
const byte DXL_CW_COMPLIANCE_SLOPE = 28;
const byte DXL_CCW_COMPLIANCE_SLOPE = 29;
const byte DXL_D_GAIN = 26;
const byte DXL_I_GAIN = 27;
const byte DXL_P_GAIN = 28;
const byte DXL_GOAL_POSITION_L = 30;
const byte DXL_GOAL_POSITION_H = 31;
const byte DXL_GOAL_SPEED_L = 32;
const byte DXL_GOAL_SPEED_H = 33;
const byte DXL_TORQUE_LIMIT_L = 34;
const byte DXL_TORQUE_LIMIT_H = 35;
const byte DXL_PRESENT_POSITION_L = 36;
const byte DXL_PRESENT_POSITION_H = 37;
const byte DXL_PRESENT_SPEED_L = 38;
const byte DXL_PRESENT_SPEED_H = 39;
const byte DXL_PRESENT_LOAD_L = 40;
const byte DXL_PRESENT_LOAD_H = 41;
const byte DXL_PRESENT_VOLTAGE = 42;
const byte DXL_PRESENT_TEMPERATURE = 43;
const byte DXL_REGISTERED_INSTRUCTION = 44;
const byte DXL_PAUSE_TIME = 45;
const byte DXL_MOVING = 46;
const byte DXL_LOCK = 47;
const byte DXL_PUNCH_L = 48;
const byte DXL_PUNCH_H = 49;
const byte DXL_SENSED_CURRENT_L = 56;
const byte DXL_SENSED_CURRENT_H = 57;

// Status Return Levels
const byte DXL_RETURN_NONE = 0;
const byte DXL_RETURN_READ = 1;
const byte DXL_RETURN_ALL = 2;

// Instruction Set
const byte DXL_PING = 1;
const byte DXL_READ_DATA = 2;
const byte DXL_WRITE_DATA = 3;
const byte DXL_REG_WRITE = 4;
const byte DXL_ACTION = 5;
const byte DXL_RESET = 6;
const byte DXL_SYNC_WRITE = 131;

// Broadcast Constant
const byte DXL_BROADCAST = 254;

// Error Codes
const byte DXL_INSTRUCTION_ERROR = 64;
const byte DXL_OVERLOAD_ERROR = 32;
const byte DXL_CHECKSUM_ERROR = 16;
const byte DXL_RANGE_ERROR = 8;
const byte DXL_OVERHEATING_ERROR = 4;
const byte DXL_ANGLE_LIMIT_ERROR = 2;
const byte DXL_INPUT_VOLTAGE_ERROR = 1;
const byte DXL_NO_ERROR = 0;

// Static parameters
const byte DXL_MIN_COMPLIANCE_MARGIN = 0;
const byte DXL_MAX_COMPLIANCE_MARGIN = 255;

const byte DXL_MIN_COMPLIANCE_SLOPE = 1;
const byte DXL_MAX_COMPLIANCE_SLOPE = 254;

// These are guesses as Dynamixel documentation doesn't have any info about it
const byte DXL_MIN_PUNCH = 0;
const byte DXL_MAX_PUNCH = 255;

const byte DXL_MAX_SPEED_TICK = 1023;                   // maximum speed in encoder units
const byte DXL_MAX_TORQUE_TICK = 1023;                  // maximum torque in encoder units

const byte KGCM_TO_NM = 0.0980665;                      // 1 kg-cm is that many N-m
const byte RPM_TO_RADSEC = 0.104719755;                 // 1 RPM is that many rad/sec
/*--- typedefs ---------------------------------------*/ 

typedef enum   {
  DECODE_INIT1=1,
  DECODE_INIT2=2,
  DECODE_COMMANDID=3,
  DECODE_LENGTH=4,
  DECODE_INSTRUCTION=5,
  DECODE_PARAMETERS=6,
  DECODE_CHECKSUM=7,
  DECODE_SENDTOAX=8
} decodeStateTX;               // ROS --> Arduino --> AX 


typedef enum   {
  ENCODE_INIT1=1,
  ENCODE_INIT2=2,
  ENCODE_COMMANDID=3,
  ENCODE_LENGTH=4,
  ENCODE_INSTRUCTION=5,
  ENCODE_PARAMETERS=6,
  ENCODE_CHECKSUM=7,
  ENCODE_SENDTOAX=8
} decodeStateRX;            // AX --> Arduino --> ROS

/*--- Declarion of variables ---------------------------------------*/

int Temperature,Voltage,Position,Error; 

// Packet fields of the command message 

byte bArrayPARAM[MAXARRAYSIZE];
byte nextCommand;
byte bServoID;
byte bInstructionNumber;
byte bInstructionLength;
byte bInstruction;
byte bChecksumRecieved;
byte bChecksumCalculated;
// Decode states

decodeStateTX currentStateTX  = DECODE_INIT1;
decodeStateTX nextStateTX     = DECODE_INIT1;

decodeStateRX currentStateRX  = ENCODE_INIT1;
decodeStateRX nextStateRX     = ENCODE_INIT1;


boolean stateAXRexiveIsRunning = false; // True if response from AX is expected



/*--- function definitions ---------------------------------------*/

void setup(){
  Dynamixel.begin(1000000,2);
  Dynamixel.setEndless(13,OFF); // Prende el Endless Mode Rotación Continua 
  Serial.begin(9600);              // Begin Serial Comunication
  delay(1000);
}

void setup (long baudDyna){
  Dynamixel.begin(baudDyna,2);  // Inicialize the servo at 1Mbps and Pin Control 2
  Serial.print(" *** Baudrate: ");
  Serial.println(baudDyna);
  Dynamixel.setEndless(13,OFF); // Prende el Endless Mode Rotación Continua
  delay(1000);
}

int pingServo (){
    for (int id = 0; id <= 255; id++) {
      Error = Dynamixel.ping(id);
      Temperature = Dynamixel.readTemperature(id); // Request and Print the Temperature
      Voltage = Dynamixel.readVoltage(id);         // Request and Print the Voltage
      Position = Dynamixel.readPosition(id);       // Request and Print the Position 
      Dynamixel.ledStatus(id,1); 
      Dynamixel.move(id,random(200,800)); 
   
       if (Error != -1) {
        Serial.print("*** ID: ");
        Serial.print(id);
        Serial.print(" Temperature: ");   // Print the variables in the Serial Monitor
        Serial.print(Temperature);
        Serial.print(" Celcius  Voltage: ");
        Serial.print(Voltage);
        Serial.print("  Volts   Position: ");
        Serial.print(Position);
        Serial.println(" of 1023 resolution");
       }
      
  }
 
  return (Error);
}

int parseCommands() {
  
}

/*--- main ---------------------------------------*/

void loop(){
  //Dynamixel.setID(11,13);

  // Dynamixel.moveSpeedRW(13, 512, 30);
  // Dynamixel.action();
  // delay(5000);
//  Dynamixel.moveSpeedRW(13,0,30);
//  Dynamixel.action();
//   delay(5000);a
//  autoSearch();
  //Dynamixel.turn(254,RIGTH,512); // Gira el servo a la Derecha a maxima velocidad

//
 if (stateAXRexiveIsRunning != true) {
     runStateMachineTX();
 } else {
     runStateMachineRX();
 }
   
     
     
     
 }

int runStateMachineRX () {
}

int runStateMachineTX () {
// ---------- Command Statemachine for Dynamixel Commands -------------

switch (currentStateTX ) {
  case DECODE_INIT1:
       
        if (getStartByte() == DXL_INITSEQ) {
                   #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);
        #endif
          nextStateTX = DECODE_INIT2;   // Wait for the second init sequence byte 0xFF.
        } else
        {
          nextStateTX = DECODE_INIT1;  // Stay in the same state an wait for a new message.
        }
       
  break;
  
  case DECODE_INIT2:
       
        if (getStartByte() == DXL_INITSEQ) {
        #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);
        #endif
          nextStateTX = DECODE_COMMANDID;   // Wait for the second init sequence byte 0xFF.
        } else
        {
          nextStateTX = DECODE_INIT1;  // Stay in the same state an wait for a new message.
        }
       
  break;
  
  case DECODE_COMMANDID:
                 
        nextCommand = getCommandByte();
        if (nextCommand != 0) {
          #ifdef DEBUG
          Serial.print("*** State: ");
          Serial.println(currentStateTX);
          #endif
          bServoID = nextCommand;
          nextStateTX = DECODE_LENGTH;   
        } else
        {
          nextStateTX = DECODE_INIT1;  // Stay in the same state an wait for a new message.
        }
  break;
  
  case DECODE_LENGTH:
         #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);
        #endif
        
        nextCommand = getCommandByte();
        if (nextCommand != 0) {                // Has to be at least 2
          bInstructionLength = nextCommand;
          nextStateTX = DECODE_INSTRUCTION;   
        } else
        {
          nextStateTX = DECODE_INIT1;  // Stay in the same state an wait for a new message.
        } 
  break;
  
  case DECODE_INSTRUCTION:
         #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);
        #endif
        
        nextCommand = getCommandByte();
        if (nextCommand != 0) {
          bInstruction = nextCommand;
          nextStateTX = DECODE_PARAMETERS;   
        } else
        {
          nextStateTX = DECODE_INIT1;  // Stay in the same state an wait for a new message.
        } 
        
   break;
  
  case DECODE_PARAMETERS:
    
        #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);
        #endif
        
        // Get N Parameters 
        // Number of Parameter = LEN - 2
        
        for (int i = 0; i < bInstructionLength - 2; i++) {
        nextCommand = getCommandByte();
        bArrayPARAM[i] = nextCommand;
        }

        nextStateTX = DECODE_CHECKSUM;   
  break;
  
  case DECODE_CHECKSUM:
        #ifdef DEBUG
        Serial.print("*** State: ");
        Serial.println(currentStateTX);      
        Serial.print("*** ID: ");
        Serial.println(bServoID);
        Serial.print("*** LEN: ");
        Serial.println(bInstructionLength);
        Serial.print("*** INS: ");
        Serial.println(bInstruction);
        Serial.println("*** Parameter: ");
        for (int i = 0; i < bInstructionLength - 2 ; i++) {
        Serial.print(bArrayPARAM[i]);
        }
        #endif
        
          nextCommand = getCommandByte();
          bChecksumRecieved = nextCommand;
          bChecksumCalculated = calculateChecksum();
          
          if (bChecksumRecieved == bChecksumCalculated) {
          // Send Command to Servo
              nextStateTX = DECODE_SENDTOAX; 
            
          } else {
           // Discard command
           nextStateTX = DECODE_SENDTOAX;  // TODO: Change to DECODE_INIT1 when checksum is calculated correctly. 
          
        }
         
  break;
  
  case DECODE_SENDTOAX:
       
       sendToAXServo();
       stateAXRexiveIsRunning = true;
       nextStateTX = DECODE_INIT1;
  break;
  
  default:
  break;
}

currentStateTX  = nextStateTX;

}

byte sendToAXServo () {
 
 switch (bInstruction) {

   case DXL_PING:
       Error = Dynamixel.ping(bServoID);
       sendData(255);
       sendData(255);
       sendData(13);
       sendData(2);
       sendData(0);
     
   break;
   
   case DXL_READ_DATA:
   break;
   
   case DXL_WRITE_DATA:
   break;
   
   case DXL_REG_WRITE:
   break;
   
   case DXL_ACTION:
   break;
   
   case DXL_RESET:
   break;
   
   case DXL_SYNC_WRITE :
   break;
   
   default:

   break;
   
 }
 
 
}

byte getStartByte () {
  byte returnCommand;
 
  if (Serial.available()) {
      returnCommand= Serial.read();
  } else {
      returnCommand = 0;
  }
  return returnCommand;
}

byte getCommandByte () {
  byte returnCommand;
 
  if (Serial.available()) {
      returnCommand= Serial.read();
  } else {
      returnCommand = 0;
  }
  return returnCommand;
}

byte calculateChecksum () {
  byte returnChecksum;
  byte parameterSum;
  
 for (int j = 0; j < bInstructionLength - 2; j++) {
   parameterSum = bArrayPARAM[j];
 }

  returnChecksum =  (~(bServoID + bInstructionLength + parameterSum))&0xFF;
  return returnChecksum;
}

void autoSearch() {

  for (int j = 0; j <= 9; j++) {
    switch(j) {
    case 0: 
       setup(9600);
      (void) pingServo();
       Dynamixel.end();
    break;
    case 1:              
      setup(19200);
      (void) pingServo();
      Dynamixel.end();
    break;
    case 2:      
      setup(57600);
      (void) pingServo();
      Dynamixel.end();
    break;
    case 3:        
      setup(115200);
      (void) pingServo();
      Dynamixel.end();
    break;
    case 4:         
      setup(200000);
      (void) pingServo();
      Dynamixel.end();
    break;
    case 5:         
      setup(250000);
      (void) pingServo();
      Dynamixel.end();
    break;
    case 6:         
      setup(400000);
     (void) pingServo();
     Dynamixel.end();
    break;
    case 7:
      setup(500000);
    (void) pingServo();
     Dynamixel.end();
    break;
    case 8:         
      setup(1000000);
      (void) pingServo();
      Dynamixel.end();
    break;
    default:   
      Serial.end();    
      continue;
    break;
    }
  }
}

