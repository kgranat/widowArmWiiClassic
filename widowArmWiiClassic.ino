/***********************************************************************************
 *  }--\     InterbotiX Robotic Arm            /--{
 *      |       Playback Code                 |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will continously playback the sequence defned in 
 *  armSequence.h . This sequence can be generated from the Arm Link Software
 *  or written by hand.
 *
 *
 *  WIRING
 *
 *    Digital Inputs
 *      Digital 2 - Button 1
 *
 *=============================================================================
 * Based upon Kurt's PX Reactor arm code.
 * https://github.com/KurtE
 * This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen Robotics:
 * http://www.trossenrobotics.com/robotic-arms.aspx
 * http://learn.trossenrobotics.com/interbotix/robot-arms
 *=============================================================================
 * 
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 ****************************************************************************************************************/

//=============================================================================
// Define Options
//=============================================================================

#define PINCHER 1
#define REACTOR 2
#define WIDOWX 3

//uncomment one of the following lines depending on which arm you want to use
//#define ARMTYPE PINCHER
//#define ARMTYPE REACTOR
#define ARMTYPE WIDOWX

#if !defined(ARMTYPE) 
   #error YOU HAVE TO SELECT THE ARM YOU ARE USING! Uncomment the correct line above for your arm
#endif

//set USE_BUTTON to FALSE if you do not have a button attached to the ArbotiX-M
#define USE_BUTTON true

//set BUTTON TRUE to HIGH for a button with built in pullup resistor like the RobotGeek Pushbutton.
//set BUTTON TRUE to LOW for a simple 2-pin pushbutton.
#define BUTTON_TRUE HIGH
#define BUTTON1_PIN 2


#define SOUND_PIN    1      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
//DYNAMIXEL Control libraries
#include <ax12.h>
#include <BioloidController.h>


#include <Wire.h>       //include the Wire/I2C Library
#include <WiiClassy.h>  //include the WiiClassy Libary
// include the RobotGeekLCD library
#include <RobotGeekLCD.h>

// create a robotgeekLCD object named 'lcd'
RobotGeekLCD lcd;

WiiClassy classy = WiiClassy(); //start an instance of the WiiClassy Library


//input control file - local
#include "Kinematics.h"

//armSequence
#include "armSequence.h"



//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);

//variables to hold the current status of the button
int buttonState1;         

volatile long lastInterruptChange;

//===================================================================================================
// Setup 
//====================================================================================================
void setup() 
{

  

  delay(100);
  classy.init();  //start classy library
  delay(100);
  classy.update();  //read data from the classic controller


  
  lcd.init(4,20);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print("Widow Arm Startup....");

  delay(1000);

  int servoReturnList[CNT_SERVOS];
  int numberOfFoundServos =   dxlScanServos(CNT_SERVOS, servoReturnList);

  


  

  lcd.setCursor(0,1);//set cursor to  column 0 row 1
    
  if(CNT_SERVOS == numberOfFoundServos )
  {  
    lcd.print("All ");
    lcd.print(numberOfFoundServos);
    lcd.print(" servos found");

    
    lcd.setCursor(0,2);//set cursor to  column 0 row 2
    
    //lcd.print("Servo Response:");
    
    lcd.setCursor(0,3);//set cursor to  column 0 row 3
    for(int i = 0; i < CNT_SERVOS; i++)
    {
      
      lcd.setCursor(i*3,2);//set cursor for each number
      lcd.print(i + 1);

      
      lcd.setCursor(i*3,3);//set cursor for each number
      lcd.print("");
      lcd.print(servoReturnList[i]);
    }
    
  }
  else
  {
    
    lcd.print("Servos missing! ");
    lcd.setCursor(0,3);//set cursor to  column 0 row 3
    for(int i = 0; i < CNT_SERVOS; i++)
    {
      
      lcd.setCursor(i*3,2);//set cursor for each number
      lcd.print(i + 1);

      
      lcd.setCursor(i*3,3);//set cursor for each number
      lcd.print("");
      lcd.print(servoReturnList[i]);
    }

    
     //number of notes, duration, frequency, duration, freuency, etc)
     MSound(3, 100, 800, 0, 600, 500, 600);
  
    while(1==1)
    {
      
    }
  }





  //initialize the Serial Port
  Serial.begin(9600);  

  //send message to serial port
  Serial.println("Interbotix Robot Arm Online.");

  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  
  // Start off to put arm to sleep...
  //PutArmToSleep();
  

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);



  


  //startup sound
  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  
  //playSequenceLeftPartOne();

  
  lcd.setCursor(0,0);
  lcd.print("Press A to Start    ");

    Serial.println(" You can now push buttons on the wii controller");
  delay(1000);
  //classy.aPressed = false;

  classy.aPressed = false; // there seems to be a bug in the library that sets some values to true on first statup, manually set it to false 

  while(classy.aPressed == false)
  {
    classy.update();  //read data from the classic controller
    heartBeat();

  }


  MoveArmToHome();  //we were able to 
  g_fArmActive = true;
  //startup sound
  MSound(6, 100, 2000, 120, 2250, 140, 2500, 160, 2000, 180, 2250, 200, 2500);
  

  
  lcd.setCursor(0,0);
  lcd.print("Arm Running         ");

}//end setup

//update the display incase of program crash. Check a last update time so that the screen is only updated periodicly.
//also update postions
void heartBeat()
{
  static long lastPrintTimeCount; // static so that this variable persists through calls
  static long lastPrintTimeCoord; // static so that this variable persists through calls
  static long count; 
  static long coordPrint;
  




  if(millis() - lastPrintTimeCount > 1000)
  {

    lcd.setCursor(16,0);
    lcd.print(count++);

    
   
  

  
    lastPrintTimeCount = millis();
  }

  

  if(millis() - lastPrintTimeCoord > 200)
  {


    switch(coordPrint){
      case 0:
        lcd.setCursor(0,1);
        lcd.print("B:");
        lcd.print(g_sBase);
        lcd.print("    ");
        break;
        
      case 1:
        lcd.setCursor(7,1); 
        lcd.print("Y:");
        lcd.print(g_sIKY);
        lcd.print("   ");
        break;
        
      case 2:
        lcd.setCursor(13,1);
        lcd.print("Z:");
        lcd.print("   ");
        lcd.print(g_sIKZ);
  
        break;
        
      case 3:
        lcd.setCursor(0,2);
        lcd.print("WA:");
        lcd.print(g_sIKGA);
        lcd.print("   ");
        break;
        
      case 4:
      
      
        lcd.setCursor(7,2);
        lcd.print("WR:");
        lcd.print(g_sWristRot);
        lcd.print("   ");
        break;
        
      case 5:
        lcd.setCursor(14,2);
        lcd.print("GR:");
        lcd.print(g_sGrip);
        lcd.print("   ");
  
        break;
        
    }

  
    coordPrint = coordPrint + 1;
    if (coordPrint > 5)
    {
      coordPrint = 0;
    }
    
    lastPrintTimeCoord = millis();
  }

  

  
}


 //last read values of analog sensors (Native values, 0-WII_JOYSTICK_MAX)
int joyXVal = 0;     //present value of the base rotation knob (analog 0)
int joyYVal = 0; //present value of the shoulder joystick (analog 1)
int joyZVal = 0;    //present value of the elbow joystick (analog 2)
int joyGAVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)
//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -spd to spd, to faciliate incremental control
float joyXMapped = 0;      //base knob value, mapped from 1-WII_JOYSTICK_MAX to BASE_MIN-BASE_MAX
float joyYMapped = 0;  //shoulder joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyZMapped = 0;     //elbow joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGAMapped = 0;     //wrist joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGripperMapped = 0;   //gripper knob  value, mapped from 1-WII_JOYSTICK_MAX to GRIPPER_MIN-GRIPPER_MAX



  float spd = 1.00;  //speed modififer, increase this to increase the speed of the movement
  float spdMod = .1;
  float maxSpd = 5;
  float minSpd = .2;
  unsigned long lastArmSpeedUpdate;
  
  int gripSpd = 10;
  int gripSpdMod = 5;
  int maxGripSpd = 100;
  int minGripSpd = 1;
  unsigned long lastGripperSpeedUpdate;
  
  int speedUpdateInterval = 100; //10hz
  
  bool lastLButtonState;
  bool lastRButtonState;


  int delayTime = 5; //milliseocnds to delay in each processAnalog function - reduce this to get full speed from the arm
  
  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }




void readClassicJoysticks()
{  
  
  classy.update();  //read data from the classic controller
   //read analog values from analog sensors
   joyXVal = classy.leftStickX;                       //read standard left stick data (0-63)
   joyYVal = classy.leftStickY;                       //read standard left stick  data (0-63)
   joyZVal = 2 * (classy.rightStickY + 1);                  //read standard right stick data (0-31),add one then,  then double it to get to 6-bit range (0-64), 
   
   joyGAVal = 2 * (classy.rightStickX + 1);                 //read standard right stick data (0-31), add one then double it to get to 6-bit range (0-64)
   joyGripperVal = 2 * (classy.rightShoulderPressure + 1);  //read standard shoulder data (0-31),add one then double it to get to 6-bit range (0-64)
   delay(delayTime);  //slow down the readings - remove


   

     
}
//
//void playSequenceMiddlePartOne()
//{
//
//}
//
//void playSequenceMiddlePartTwo()
//{
//
//}
//
//void playSequenceCornerPartOne()
//{
//
//}
//
//void playSequenceCornerPartTwo()
//
//void playSequenceRightPartOne()
//
//void playSequenceRightPartTwo()
//
//void ()
//
//void ()
//


     
 #define WII_JOYSTICK_MAX 63
  
  //generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
  #define DEADBANDLOW 30   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
  #define DEADBANDHIGH 34  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

   int leftStep = 0;
   int rightStep = 0;
   int middleStep = 0;
   int cornerStep = 0;
  
void processSequences()
{

  static bool lastLeftPressed = false;
  static bool lastRightPressed = false;
  static bool lastMiddlePressed = false;
  static bool lastCornerPressed = false;

  

  if(classy.leftDPressed )
  {
    if(lastLeftPressed == false)
    {
      
    
      if(leftStep == 0 )
      {
        MSound(1, 80, 1000);
        playSequenceLeftPartOne();
        leftStep = 1;
        cornerStep = 0;
        middleStep = 0;
        rightStep = 0;
      }

      
      else if(leftStep == 1 )
      {
        MSound(3, 80, 1000,0,500, 100, 1000);
        playSequenceLeftPartTwo();
        leftStep = 0;
      }
    }
    
    lastLeftPressed = true;
  }
  else
  {
    lastLeftPressed = false;
  }







  if(classy.rightDPressed )
  {
    if(lastRightPressed == false)
    {
      
    
      if(rightStep == 0 )
      {
        MSound(1, 200, 1000);
        playSequenceRightPartOne();
        rightStep = 1;
        cornerStep = 0;
        middleStep = 0;
        leftStep = 0;
      }

      
      else if(rightStep == 1 )
      {
        MSound(3, 200, 1000,0,500, 200, 1000);
        playSequenceRightPartTwo();
        rightStep = 0;
      }
    }
    
    lastRightPressed = true;
  }
  else
  {
    lastRightPressed = false;
  }




  if(classy.upDPressed )
  {
    if(lastMiddlePressed == false)
    {
      
    
      if(middleStep == 0 )
      {
        MSound(1, 300, 1000);
        playSequenceMiddlePartOne();
        middleStep = 1;
        cornerStep = 0;
        leftStep = 0;
        rightStep = 0;
      }

      
      else
      {
        MSound(3, 300, 1000,0,500, 300, 1000);
        playSequenceMiddlePartTwo();
        middleStep = 0;
      }
    }
    
    lastMiddlePressed = true;
  }
  else
  {
    lastMiddlePressed = false;
  }



  if(classy.downDPressed )
  {
    if(lastCornerPressed == false)
    {
      
    
      if(cornerStep == 0 )
      {
        MSound(1, 400, 1000);
        playSequenceCornerPartOne();
        cornerStep = 1;
        middleStep = 0;
        leftStep = 0;
        rightStep = 0;
      }

      
      else
      {
        MSound(3, 400, 1000,0,500, 400, 1000);
        playSequenceCornerPartTwo();
        cornerStep = 0;
      }
    }
    
    lastCornerPressed = true;
  }
  else
  {
    lastCornerPressed = false;
  }



  
}


void updateControls()
{
    static bool lastYPressed = false;
  
    readClassicJoysticks();

    processSequences();


  

    if(classy.yPressed )
    {
      if(lastYPressed == false)
      {
        MSound(1,1000,1000);
        if(g_bIKMode == IKM_CYLINDRICAL_90)
        {
          g_bIKMode = IKM_CYLINDRICAL;

          MoveArmToHome();  //we were able to 
          g_fArmActive = true;
        }
        else
        {
          g_bIKMode = IKM_CYLINDRICAL_90;

          MoveArmToHome();  //we were able to 
          g_fArmActive = true;          
          
        }

        

      }
  
  
      lastYPressed = true;
    }
    else
    {
      lastYPressed = false;
    }

  
    if(classy.homePressed == true)
    {
      MoveArmToHome();
      g_fArmActive = true;
      leftStep = 0;
      rightStep = 0;
      middleStep = 0;
      cornerStep = 0;
    }
    
    if(classy.selectPressed == true)
    {
      
      EmergencyStop();
      g_fArmActive = false;
      leftStep = 0;
      rightStep = 0;
      middleStep = 0;
      cornerStep = 0;
    }
    
    if(classy.startPressed == true)
    {
      
      PutArmToSleep();
      g_fArmActive = false;
      leftStep = 0;
      rightStep = 0;
      middleStep = 0;
      cornerStep = 0;
    }
    
  
     //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
     {
       joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0,  -spd * 10 , spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sBase += joyXMapped;
     }
  
     //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
     {
       joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       g_sIKY -= joyYMapped;
     }
  
     //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
     {
      
  
       joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       
       g_sIKZ -= joyZMapped;
     }
     
     //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
     if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
     {

        Serial.println(joyGAVal);
       joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd * 5, spd * 5); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
       //g_sIKGA += joyGAMapped;
       sWristRot += joyGAMapped;
     }

     

  
   if(classy.lzPressed)
   {
    
     sGrip = 176;
   }

   
   if(classy.rzPressed)
   {
     sGrip = 512;

   }
    

  
   if(classy.leftShoulderPressed)
   {
    
     sWristRot = 814;
   }

   
   if(classy.rightShoulderPressed)
   {
     sWristRot = 208;

   }








    int interpolate = 10;


    
      g_sBase = constrain(g_sBase, BASE_MIN, BASE_MAX);
      g_sIKY = constrain(g_sIKY, IK_MIN_Y, IK_MAX_Y);
      g_sIKZ = constrain(g_sIKZ, IK_MIN_Z, IK_MAX_Z);
      g_sIKGA = constrain(g_sIKGA, IK_MIN_GA, IK_MAX_GA);
      sBase = constrain(sBase, BASE_MIN, BASE_MAX);
      sShoulder = constrain(sShoulder, SHOULDER_MIN, BASE_MAX);
      sElbow = constrain(sElbow, ELBOW_MIN, BASE_MAX);
      sWrist = constrain(sWrist, WRIST_MIN, WRIST_MAX);
      sWristRot = constrain(sWristRot, WROT_MIN, WROT_MAX);
      sGrip = constrain(sGrip, GRIP_MIN, GRIP_MAX);

      
  if (g_fArmActive == true) 
  {

    

     
      doArmIK(false , g_sBase,  g_sIKY,  g_sIKZ,  g_sIKGA);  //update serovs based on y/z/angle ik values
  
      MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, interpolate, true);  //move arm to nw positions
  }
  
}



//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() 
{
  
  heartBeat();


    

    updateControls();

  


    
//  
//  if (classy.aPressed == true) 
//  {
//    Serial.println("ClassyA");
//    playSequence();
//  }
//  
// 
//  if (classy.leftDPressed == true) {
//    playSequenceLeftPartOne();
//  }
//  if (classy.xPressed == true) {
//    playSequenceLeftPartTwo();
//  }
//  
//  
//  if (classy.rightDPressed == true) {
//    playSequenceRightPartOne();
//  }
//  
//  if (classy.yPressed == true) {
//    playSequenceRightPartTwo();
//  }
//  
  
  
  
  if (bioloid.interpolating > 0) 
  {
    bioloid.interpolateStep();
  }
} //end Main




//===================================================================================================
// functions
//===================================================================================================


void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable)
{
  //only run the sequence code if the enable is set
  if(enable == 1)
  {
    //if the arm is in Cartesian mode, do the arm IK in cartesian mode. This will set the arm parameters
    if(g_bIKMode == IKM_IK3D_CARTESIAN || g_bIKMode == IKM_IK3D_CARTESIAN_90)
    {
      doArmIK(true, X, Y, Z, GA); 
      
    }
    
    //if the arm is in cylindrical mode, do the arm IK in cylindrical mode. This will set the arm parameters
    else if(g_bIKMode == IKM_CYLINDRICAL || g_bIKMode ==IKM_CYLINDRICAL_90)
    {  
    //  sBase = X;
      doArmIK(false, X, Y, Z, GA); 
      
    }
    
    //otherweise the arm is in backhoe mode, so set the arm parameters directly
    else if(g_bIKMode == IKM_BACKHOE)
    {
      sBase = X;
      sShoulder = Y;
      sElbow = Z;
      sWrist = GA;
      
    }
    
    
    //set the wrist rotate and gripper parameters (as these are not part of the IK)
    sWristRot = WR;
    sGrip = grip;
    
    //move arm to position
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, interpolate, true);  
    
    //wait for pause milliseconds
    delay(pause);
  }
}








// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif


