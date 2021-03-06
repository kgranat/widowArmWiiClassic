//Arm 3
//Sequence 7
//Mode 2
//Orientation 1
//DIO 0
#include "Kinematics.h"
#include "GlobalArm.h"
extern void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable);
// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
volatile int playState = 0; // 0 = stopped 1 = playing


void playSequenceMiddlePartOne()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

}

void playSequenceMiddlePartTwo()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

}

void playSequenceCornerPartOne()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

}

void playSequenceCornerPartTwo()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

}


void playSequenceRightPartOne()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

    
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(2048 , 250 , 225 , 0 , 512 , 256 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    IKSequencingControl(1682, 250 , 225 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    IKSequencingControl(1682 , 395 , 140 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 
}


void playSequenceRightPartTwo()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

    
    // SEQUENCE 3
    //###########################################################// 
    IKSequencingControl(1682 , 395 , 76 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 
    //###########################################################//
    // SEQUENCE 4
    //###########################################################// 
    IKSequencingControl(1682 , 395 , 73 , 0 , 820 , 213 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 5
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 155 , 0 , 204 , 176 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 6
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 300 , 0 , 204 , 176 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 7
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 155 , 0 , 204 , 495 , 1000,250, playState);
    //###########################################################// 


  
}


void playSequenceLeftPartOne()
{
  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;

  //###########################################################//
    // Left Stack Home
    //###########################################################// 
    IKSequencingControl(2048 , 250 , 225 , 0 , 512 , 256 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // Rotate to left stack 
    //###########################################################// 
    IKSequencingControl(2357 , 250 , 225 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 




    //###########################################################//
    // Y Z to Left stack above stack
    //###########################################################// 
    IKSequencingControl(2357 , 395 , 120 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 

}

void playSequenceLeftPartTwo()
{

  playState = 1;  //set playState to 1 as the sequence is now playing
    g_bIKMode = IKM_CYLINDRICAL;






    //###########################################################//
    // Y Z to Left stack  at astack
    //###########################################################// 
    IKSequencingControl(2357 , 395 , 76 , 0 , 829 , 512 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // Grip the Left stack
    //###########################################################// 
    IKSequencingControl(2357 , 395 , 73 , 0 , 820 , 213 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // Move to middle
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 155 , 0 , 204 , 213 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 6
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 300 , 0 , 204 , 213 , 1000,250, playState);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 7
    //###########################################################// 
    IKSequencingControl(2020 , 328 , 155 , 0 , 204 , 495 , 1000,250, playState);
    //###########################################################// 



  




  
}





void playSequence()
{
  delay(500);
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Press Pushbutton  to stop");




 delay(100);
 Serial.println("Pausing Sequencing Mode."); 
 delay(500);
 //uncomment this to  put the arm in sleep position after a sequence
 //PutArmToSleep();
}
