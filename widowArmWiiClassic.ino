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
  lcd.init(4,20);
  // Print a message to the LCD.
  lcd.print("Widow Arm Startup....");

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
    lcd.print(numberOfFoundServos);
    lcd.print(" servos found");

    
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
  PutArmToSleep();
  

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);


  delay(100);
  classy.init();  //start classy library
  delay(100);
  classy.update();  //read data from the classic controller
  

  Serial.println(" You can now push buttons on the wii controller");
  delay(1000);
  //classy.aPressed = false;
  


  //startup sound
  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  
  //playSequenceLeftPartOne();

  
  lcd.setCursor(0,0);
  lcd.print("Press A to Start    ");

  classy.aPressed = false; // there seems to be a bug in the library that sets some values to true on first statup, manually set it to false 

  while(classy.aPressed == false)
  {
    classy.update();  //read data from the classic controller

  }


  
  //startup sound
  MSound(6, 100, 2000, 120, 2250, 140, 2500, 160, 2000, 180, 2250, 200, 2500);


  
  
  lcd.setCursor(0,0);
  lcd.print("Arm Started         ");

}//end setup


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() 
{

    classy.update();  //read data from the classic controller


    
  lcd.setCursor(0,0);
  lcd.print("B:");
  lcd.print(g_sBase);

  lcd.print("Y:");
  lcd.print(g_sIKY);

  lcd.print("Z:");
  lcd.print(g_sIKZ);
  lcd.print("WA:");
  lcd.print(g_sIKGA);

  lcd.print("WR:");
  lcd.print(g_sWristRot);


  lcd.print("GR:");
  lcd.print(g_sGrip);
  


    
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


