 /***********************************************************************************
 *       ___________________________________
 *      |  _______________________________  |
 *      | |RobotGeek I2C LCD              |\|
 *      | |  RobotGeekLCD.cpp             | |
 *      | |                               | |
 *      | |_______________________________|/|
 *      |___________________________________|
 *
 *  This library will allow use of the RobotGeek IIC LCD
 *   http://learn.trossenrobotics.com/30-robotgeek-getting-started-guides/dev-kits/57-robotgeek-i2c-lcd-getting-started-guide
 *   http://www.trossenrobotics.com/robotgeek-lcd
 *     
 *	The RobotGeek IIC LCD has 3 components
 *	1)16x2 LCD
 *	2)Hitachi HD44780 LCD Driver
 *	3)pcf8574 I2C - I2C IO Expander
 *	
 *	The HD44780 is controlled by 3 control pins and 8 data pins. These pins
 *	are controlled by the pcf8574 I2C chip. By sending IIC data to this chip
 *	the controller can control the HD44780 and the LCD. The pcf8574 also gives
 *	access to the backlight.
 *
 *
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
 *    LiquidCrystal Library and examples
 *    http://www.arduino.cc/en/Tutorial/LiquidCrystal
 *     This example code is in the public domain.
 *
 *
 ***********************************************************************************/
 

#include "RobotGeekLCD.h"
#include <inttypes.h>
#include "Arduino.h"
#include <Wire.h>

//the LCD will always be IIC, so we'll strip out alot of the LiquidCrystal Library






RobotGeekLCD::RobotGeekLCD()
{
  //init(LCD_ADDRESS);//don't call init from here! all sorts of weirdness from the wire library
    _backlight = 1; //backlight will be set on by default
    _overFlowMode = 0;
}


//old style init - for legacy code with 16x2 display
//because overflowMode is set to 0, it doesn't really matter what the row/columns are set to, we've just put 16,2 here to comply with the function

void RobotGeekLCD::init()
{
  init(2,16,0);
}

void RobotGeekLCD::init(uint8_t numRows , uint8_t numColumns)
{
  init(numRows,numColumns,1); //initialze, assumimg overflow is on
}

//always four bit mode for the rg lcd
void RobotGeekLCD::init(uint8_t numRows , uint8_t numColumns, bool overFlowMode )
{
  //I2C address for the RobotGeek I2C LCD - this is hard wired to LCD_ADDRESS
  int address =LCD_ADDRESS;	
    _direction = 1;
    _numColumns = numColumns;
    _numRows = numRows;
    _overFlowMode = overFlowMode;
    _customCharFlag = 0;

  Wire.begin(); // join i2c bus
	
  _rs_pin =0;
  _rw_pin = 1;
  _enable_pin = 2;
  
  _data_pins[0] = 0;
  _data_pins[1] = 0;
  _data_pins[2] = 0;
  _data_pins[3] = 0; 
  _data_pins[4] = 4;
  _data_pins[5] = 5;
  _data_pins[6] = 6;
  _data_pins[7] = 7; 

  //always 4 bit mode
  _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

  //start lcd
  begin(address);  
}

void RobotGeekLCD::begin(uint8_t address) 
{
	_numlines = 4;
	_currline = 0;

	int tadd = address;

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  delayMicroseconds(50000); 
  // Now we pull both RS and R/W low to begin commands
  _iicLastSent = B00000000;
  writeIIC(0);
  /*
	Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
	Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
	delay(1000);
	Wire.endTransmission();   
  */


    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms

    // second try
    write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms
    
    // third go!
    write4bits(0x03); 
    delayMicroseconds(150);

    // finally, set to 4-bit interface
    write4bits(0x02); 

	// finally, set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction);  

	// turn the display on with no cursor or blinking default
  	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  	noDisplay();

  	// clear it off

  	clear();
   

  	// Initialize to default text direction (for romance languages)
  	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  	// set the entry mode
  	command(LCD_ENTRYMODESET | _displaymode);
    
    display();
}

/********** high level commands, for the user! */
void RobotGeekLCD::clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
  columnN = 0;
  rowN = 0;
}

void RobotGeekLCD::home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
  columnN = 0;
  rowN = 0;
}

void RobotGeekLCD::setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if ( row >= _numlines ) {
    row = _numlines-1;    // we count rows starting w/0
  }
  
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));

  
  columnN = col;
  rowN = row;
}

// Turn the display on/off (quickly)
void RobotGeekLCD::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void RobotGeekLCD::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}


// Turn the display on/off (quickly)
void RobotGeekLCD::noBacklight() {

		_iicLastSent = _iicLastSent & (~B00001000);
		Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
		Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
		Wire.endTransmission();    // stop transmitting
		
}

void RobotGeekLCD::backlight() {
		_iicLastSent = _iicLastSent | B00001000;
		
		Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
		Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
		Wire.endTransmission();    // stop transmitting
		
		
}



// Turns the underline cursor on/off
void RobotGeekLCD::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void RobotGeekLCD::cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void RobotGeekLCD::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void RobotGeekLCD::blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void RobotGeekLCD::scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void RobotGeekLCD::scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void RobotGeekLCD::leftToRight(void) {
  _direction = 1;
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void RobotGeekLCD::rightToLeft(void) {
  _direction = -1;
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void RobotGeekLCD::autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void RobotGeekLCD::noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void RobotGeekLCD::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    _customCharFlag = 1;
    write(charmap[i]);
    _customCharFlag = 0;
  
  }
}

/*********** mid level commands, for sending data/cmds */

inline void RobotGeekLCD::command(uint8_t value) {
  send(value, LOW);
}



inline size_t RobotGeekLCD::write(uint8_t value) {

  if(_customCharFlag == 0)//don't run character management when creating custom character
  {
    //overflow mode puts the text on the next line, up to the end of the display
    if(_overFlowMode == true)
    { 
      if(columnN >= _numColumns)
      {

        rowN++;
        if(rowN >= _numRows)
        {
          setCursor(_numColumns-1,_numRows-1); //set cursor to last possible position
          rowN = _numRows ; //save rowN to larger than last possible position to stop anything from printing anywhere else
          columnN = _numColumns;//save columnN to larger than last possible position to stop anything from printing anywhere else
          return 0;
        }
        columnN = 0;
        setCursor(columnN,rowN);
      }
      if(columnN < 0)
      {

        rowN--;
        if(rowN < 0)
        {
          setCursor(0,0); //set cursor to first possible position
          rowN = -1 ; //save rowN to smallest than last possible position to stop anything from printing anywhere else
          columnN = -1;//save columnN to smallest than last possible position to stop anything from printing anywhere else
          return 0;
        }
        columnN = 19;
        setCursor(columnN,rowN);
      }
    }

    //if overflow is off for a 20x4, then stop writing at the end of the line
    if(_overFlowMode == false && _numRows == 4 && _numColumns == 20)
    { 
      if(columnN >= _numColumns)
      {
          setCursor(_numColumns - 1,rowN);//set cursor to last possible position in current row
          columnN =  _numColumns;//save columnN to larger than last possible position to stop anything from printing anywhere else
         
          return 0;
      }
    }

    //otherwise asume 16x2 display, and don't do antyhing special in order to preserve the scroll/autoscroll features
    if(_direction == 1)
    {
      columnN = columnN + 1;

    }
    else if (_direction == -1)
    {
      columnN = columnN - 1;

    }
    //columnN++;
  }
  
  send(value, HIGH);
  return 1; // assume sucess
}



/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void RobotGeekLCD::send(uint8_t value, uint8_t mode) {

	//int tempValue = mode;//rspin high
	//tempValue = tempValue &(~(1<<2))//rw_pin low
	


	if(mode == 1)
	{
		/*_iicLastSent = _iicLastSent | B00000001;
		_iicLastSent = _iicLastSent & (~B00000010);
		Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
		Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
		//Wire.endTransmission();    // stop transmitting
		*/
    write4bitsData(byte(value>>4));
    write4bitsData(value & B00001111);
    
	}
	else
	{
	
		/*_iicLastSent = _iicLastSent & (~B00000011);
		Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
		Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
		//Wire.endTransmission();    // stop transmitting
	*/
	
    write4bitsCommand(value>>4);
    write4bitsCommand(value & B00001111);
	
	}


  //}
}

void RobotGeekLCD::pulseEnable(uint8_t current) {


   
  current = current & (~B00000100);//mask last send with inverse enable bit to turn it off 
  Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
  Wire.write(current);              // sends one byte, sets RS,R/W, and enable low  
  Wire.endTransmission();    // stop transmitting

  delayMicroseconds(1);   // enable pulse must be >450ns 
  
  current = current | B00000100;//mask last send with enable bit to turn it on 
  Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
  Wire.write(current);              // sends one byte, sets RS,R/W, and enable low  
  Wire.endTransmission();    // stop transmitting

  delayMicroseconds(1);    // enable pulse must be >450ns
  
  current = current & (~B00000100);//mask last send with inverse enable bit to turn it off 
  Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
  Wire.write(current);              // sends one byte, sets RS,R/W, and enable low  
  Wire.endTransmission();    // stop transmitting

  delayMicroseconds(100);    // commands need > 37us to settle



}

void RobotGeekLCD::write4bitsCommand(uint8_t value){

	int tempValue = (value<<4) | (_backlight<<3);//shift the value up 4 bits, add backlight bit. En,RW,RS are all 0
	Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
	Wire.write(tempValue);              // sends one byte, sets RS,R/W, and enable low  
	Wire.endTransmission();    // stop transmitting
	pulseEnable(tempValue);
}

void RobotGeekLCD::write4bitsData(uint8_t value){

	int tempValue = (value << 4) | (_backlight <<3)| 0x1;///shift the value up 4 bits, add backlight bit. En,RW, are 0, rs = 1
	Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
	Wire.write(tempValue);              // sends one byte, sets RS,R/W, and enable low  
	Wire.endTransmission();    // stop transmitting
	pulseEnable(tempValue);
}


void RobotGeekLCD::write4bits(uint8_t value)
{

	int tempValue = value << 4;//shift the value up 8 bits

	_iicLastSent = _iicLastSent & (~B11110000);//mask last sent value with value bits 
	_iicLastSent = _iicLastSent | (tempValue);//mask last sent value with value bits 

	Wire.beginTransmission(LCD_ADDRESS); // transmit to device LCD_ADDRESS, lcd
	Wire.write(_iicLastSent);              // sends one byte, sets RS,R/W, and enable low  
	Wire.endTransmission();    // stop transmitting
	pulseEnable(tempValue);
}


void RobotGeekLCD::writeIIC(uint8_t _data){                                        
	Wire.beginTransmission(LCD_ADDRESS);
	Wire.write(0);
	Wire.endTransmission();   
}


