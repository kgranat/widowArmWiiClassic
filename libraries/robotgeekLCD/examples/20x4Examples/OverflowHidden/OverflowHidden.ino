/***********************************************************************************
 *       ___________________________________
 *      |  _______________________________  |
 *      | |RobotGeek I2C LCD              |\|
 *      | |  OverflowHidden               | |
 *      | |                               | |
 *      | |_______________________________|/|
 *      |___________________________________|
 *
 *  This example will show the behavior where text that cannot fit on a line is not shown.  
 *.  
 *  This init() is not compatible with scroll() or autoScroll()
 *    
 *  Wiring
 *    RobotGeek Sensor Shield With Geekduino/Arduino Uno/Seeeduino
 *      The RobotGeek I2C LCD will plug directly into the 4-pin I2C port on
 *      the sensor shield using a 4-pin sensor cable. Normally the sensor cable
 *      will be plugged in with the following orientation:
 *        SCL    - Yellow
 *        SDA    - White
 *        5v/VCC - Red
 *        G/GND  - Black
 *  
 *    Geekduino/Arduino/Seeeduino 
 *       The RobotGeek I2C LCD can be plugged direcly into the pins of a 
 *        ~duino board using Female-Female jumper cables and pin couplers
 *        LCD - ~duino Pin 
 *        SCL - A5
 *        SDA - A4
 *        VCC - 5v
 *        GND - Gnd
 *        
 *    ArbotiX
 *        LCD - ~duino Pin 
 *        SCL - D16 9Pin closest to processor0
 *        SDA - D17 9Pin closest to processor0
 *        VCC - 5v Pin on D17 (Middle Pin)
 *        GND - Gnd Pin on D17(Pin closest to the edge of the board)
 *
 *
 *  References
 *    This example and the RobotGeekLCD library is based off the Arduino
 *    LiquidCrystal Library
 *    http://www.arduino.cc/en/Tutorial/LiquidCrystal
 *    This example code is in the public domain.
 *
 *
 ***********************************************************************************/


//include the I2C Wire library - needed for communication with the I2C chip attached to the LCD manual 
#include <Wire.h> 
// include the RobotGeekLCD library
#include <RobotGeekLCD.h>

// create a robotgeekLCD object named 'lcd'
RobotGeekLCD lcd;

void setup() 
{
  // initlaize the lcd object - this sets up all the variables and IIC setup for the LCD object to work. By specifcying the 16x2 display, we'll initialize overflow mode
  lcd.init(4,20,false); 
  // Print a message to the LCD.
  lcd.print("Overflow is hidden - this means that text that goes past the 20th character on a line is not shown");
}

void loop() 
{
  
}
