/*

The servo controller hardware consists of 2 AVRs out of which one is Master and has Arduino bootloader. The second one is slave and runs all servo routines.
The master controls slave through I2C and slave operates all servos.
Many pins of the main controller are free and can be used for connecting other peripherals and interfaces. 

This code is for Arduino UNO board profile and the same should be chosed while programming with Arduino IDE.

Hard-coded to use only the ports from servo 1 to servo 7.

*/

#include <Wire.h>

// #defines
  #define servo1	(16>>1)
  #define servo2	(18>>1)
  #define UART_BAUD_RATE	115200
  #define LED 13
  #define SERIAL_BUFFER_SIZE  256
  #define State_Start  0
  #define State_Command  1
  #define State_Servoposition  2
  #define State_Speed  3
  #define State_Servomin  4
  #define State_Servomax  5
  #define State_Servooffset  6
  #define State_Servoreverse  7
  #define State_Servonutral  8
  #define State_ReadOffsets  9

  #define DEFAULT_DELAY 300
//---------------------------------------------------------------------------

// Function inits
  void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos);
  void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir);
  void I2C_SERVOOFFSET(unsigned char servo_num,int value);
  void I2C_SERVOSPEED(unsigned char value);
  void I2C_SERVONUTRALSET(unsigned char servo_num,unsigned int servo_pos);
  void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos);
  void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos);
  char I2C_SERVOEND(void);
  int I2C_SERVOGET(int servo_num);
  int I2C_SERVOGETOFFSET(int servo_num);
  void LEDToggle(void);
  // Self functions
  void EncoderPos(unsigned char encoder_num);
  void TCA9548A(uint8_t bus);
  // Encoder functions
  void ReadRawAngle(void);
  void correctAngle(void);
  void checkQuadrant(void);
  void checkMagnetPresence(void);
//----------------------------------------------------------------------------

// Global variables
  volatile int cnt,c,servoval;
  volatile char state,servobuf[36],bytecnt;

  int interval=100;
  unsigned long previousMillis=0;
  unsigned long currentMillis = millis();
  char LEDState=0;

  unsigned int servopos[7] = {500, 500, 500, 500, 500, 500, 500} ; // uint to hold servo position in microseconds (0-180 deg is mapped to to 500-2500 us), initialised to 0 deg
  String inString = "";  // string to hold input
  String tempString = ""; //string to hold the speed and delay values
  int counter = 0; // counter for the servopos array
  int setDelay = DEFAULT_DELAY;
  
  //Magnetic sensor things
  int magnetStatus = 0; //value of the status register (MD, ML, MH)
  int lowbyte; //raw angle 7:0
  word highbyte; //raw angle 7:0 and 11:8
  int rawAngle; //final raw angle 
  float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
  int quadrantNumber, previousquadrantNumber; //quadrant IDs
  float numberofTurns = 0; //number of turns
  float correctedAngle = 0; //tared angle - based on the startup value
  float startAngle = 0; //starting angle
  float totalAngle = 0; //total absolute angular displacement
//-----------------------------------------------------------------------------------

void setup()
{ 
	cnt=0;
	int i;
	unsigned int x;
	char buffer[10],tmp,tmp1;
	float range;
	Serial.begin(UART_BAUD_RATE);
	pinMode(13, OUTPUT);
	pinMode(2, OUTPUT);
	pinMode(8, INPUT);
	digitalWrite(8,1);
	delay(500);
	sei();
	Wire.begin();
	TWSR = 3;	// no prescaler
	TWBR = 18;	//Set I2C speed lower to suite I2C Servo controller
	pinMode(2,OUTPUT);
	digitalWrite(2,HIGH);
	delay(500);
	state=State_Start;
  
  // Servo configuration
    I2C_SERVOMAX(1,2500); 
    I2C_SERVOMAX(2,2500); 
    I2C_SERVOMAX(3,2500); 
    I2C_SERVOMAX(4,2500); 
    I2C_SERVOMAX(5,2500); 
    I2C_SERVOMAX(6,2500); 
    I2C_SERVOMAX(7,2500); 
    I2C_SERVOMAX(8,2500); 
    I2C_SERVOMAX(9,2500); 
    I2C_SERVOMAX(10,2500); 
    I2C_SERVOMAX(11,2500); 
    I2C_SERVOMAX(12,2500); 
    I2C_SERVOMAX(13,2500); 
    I2C_SERVOMAX(14,2500); 
    I2C_SERVOMAX(15,2500); 
    I2C_SERVOMAX(16,2500); 
    I2C_SERVOMAX(17,2500); 
    I2C_SERVOMAX(18,2500);		//Maximum Values

    I2C_SERVOMIN(1,500); 
    I2C_SERVOMIN(2,500); 
    I2C_SERVOMIN(3,500); 
    I2C_SERVOMIN(4,500); 
    I2C_SERVOMIN(5,500); 
    I2C_SERVOMIN(6,500); 
    I2C_SERVOMIN(7,500); 
    I2C_SERVOMIN(8,500); 
    I2C_SERVOMIN(9,500); 
    I2C_SERVOMIN(10,500); 
    I2C_SERVOMIN(11,500); 
    I2C_SERVOMIN(12,500); 
    I2C_SERVOMIN(13,500); 
    I2C_SERVOMIN(14,500); 
    I2C_SERVOMIN(15,500); 
    I2C_SERVOMIN(16,500); 
    I2C_SERVOMIN(17,500); 
    I2C_SERVOMIN(18,500);		//Minimum Values

    I2C_SERVOOFFSET(1,1500); 
    I2C_SERVOOFFSET(2,1500); 
    I2C_SERVOOFFSET(3,1500); 
    I2C_SERVOOFFSET(4,1500); 
    I2C_SERVOOFFSET(5,1500); 
    I2C_SERVOOFFSET(6,1500); 
    I2C_SERVOOFFSET(7,1500); 
    I2C_SERVOOFFSET(8,1500); 
    I2C_SERVOOFFSET(9,1500); 
    I2C_SERVOOFFSET(10,1500); 
    I2C_SERVOOFFSET(11,1500); 
    I2C_SERVOOFFSET(12,1500); 
    I2C_SERVOOFFSET(13,1500); 
    I2C_SERVOOFFSET(14,1500); 
    I2C_SERVOOFFSET(15,1500); 
    I2C_SERVOOFFSET(16,1500); 
    I2C_SERVOOFFSET(17,1500); 
    I2C_SERVOOFFSET(18,1500);		//Offset Values

    I2C_SERVOREVERSE(1,0); 
    I2C_SERVOREVERSE(2,0); 
    I2C_SERVOREVERSE(3,0); 
    I2C_SERVOREVERSE(4,0); 
    I2C_SERVOREVERSE(5,0); 
    I2C_SERVOREVERSE(6,0); 
    I2C_SERVOREVERSE(7,0); 
    I2C_SERVOREVERSE(8,0); 
    I2C_SERVOREVERSE(9,0); 
    I2C_SERVOREVERSE(10,0); 
    I2C_SERVOREVERSE(11,0); 
    I2C_SERVOREVERSE(12,0); 
    I2C_SERVOREVERSE(13,0); 
    I2C_SERVOREVERSE(14,0); 
    I2C_SERVOREVERSE(15,0); 
    I2C_SERVOREVERSE(16,0); 
    I2C_SERVOREVERSE(17,0); 
    I2C_SERVOREVERSE(18,0);		//Directions (Servo Reverse)

  
  // ------------------------------------------------------------------------
  // for encoder
  // checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring

}

void loop()
{
  // read serial input:
  while (Serial.available() > 0) {    
    int cmdMode = Serial.read();
    
    switch(cmdMode) {      
      
      case 'S':
      // servo command mode        
        while (Serial.available() > 0) {
          int inChar = Serial.read();
          
          if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
          }
          else if (inChar == 'p') {
            // keep the present servopos unchanged and skip to the next one:
            counter++;
          }   
          // if we get an underscore, store the int value in the respective servopos element:
          else if (inChar == '_') {
            servopos[counter] = inString.toInt();
            counter++;
            // then clear the string for new input:
            inString = "";
          }
          // set servo speed value
          else if (inChar == 's') {
            while (Serial.available() > 0) {
              int tempChar = Serial.read();
              if (isDigit(tempChar)) {
                // convert the incoming byte to a char and add it to the string:
                tempString += (char)tempChar;
              }
              else if (tempChar == '_') {
                // convert the tempString to int and set the servo speed:
                I2C_SERVOSPEED(tempString.toInt());
                // then reset the tempString:
                tempString = "";
                break;
              }
            }
          }
          // set delay value
          else if (inChar == 'd') {
            while (Serial.available() > 0) {
              int tempChar = Serial.read();
              if (isDigit(tempChar)) {
                // convert the incoming byte to a char and add it to the string:
                tempString += (char)tempChar;
              }
              else if (tempChar == '_') {
                // convert the tempString to int and set the servo delay:
                setDelay = tempString.toInt();
                // then reset the tempString:
                tempString = "";
                break;
              }
            }
          }
          else if (inChar == 'e') {
              // write the new servo positions
              ServoSetAll(servopos[0],servopos[1],servopos[2],servopos[3],servopos[4],servopos[5],servopos[6],0,0,0,0,0,0,0,0,0,0,0);
              // then wait for the servos to reach the desired position:
              // delay(setDelay); // commented out so that instantaneous position feedback can be obtained while the servos are moving
              // reset the counter
              counter = 0;
              break;
          }
        }
        break;
      
      case 'E':
      // encoder command mode
        while (Serial.available() > 0) {
          int inChar = Serial.read();
          if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
          }
          else if (inChar == 'e') {
            EncoderPos(inString.toInt());
            // then clear the string for new input:
            inString = "";
            break;
          }
        }       
        break;

    }
  }
}

// Encoder functions
  void TCA9548A(uint8_t bus) // Select I2C BUS
  {
    Wire.beginTransmission(0x70);  // TCA9548A address
    Wire.write(1 << bus);          // send byte to select bus
    Wire.endTransmission();
  }

  void EncoderPos(unsigned char encoder_num)
  {
    TCA9548A(encoder_num);
    ReadRawAngle(); //ask the value from the sensor
    correctAngle(); //tare the value
    checkQuadrant(); //check quadrant, check rotations, check absolute angular position
    Serial.print(totalAngle); //absolute position of the motor expressed in degree angles, 2 digits
    Serial.print('e');
  }

  void ReadRawAngle(void)
  { 
    //7:0 - bits
    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor
    
    while(Wire.available() == 0); //wait until it becomes available 
    lowbyte = Wire.read(); //Reading the data after the request
  
    //11:8 - 4 bits
    Wire.beginTransmission(0x36);
    Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    
    while(Wire.available() == 0);  
    highbyte = Wire.read();
    
    //4 bits have to be shifted to its proper place as we want to build a 12-bit number
    highbyte = highbyte << 8; //shifting to left
    //What is happening here is the following: The variable is being shifted by 8 bits to the left:
    //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
    //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
    
    //Finally, we combine (bitwise OR) the two numbers:
    //High: 00001111|00000000
    //Low:  00000000|00001111
    //      -----------------
    //H|L:  00001111|00001111
    rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

    //We need to calculate the angle:
    //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
    //360/4096 = 0.087890625
    //Multiply the output of the encoder with 0.087890625
    degAngle = rawAngle * 0.087890625; 
    
    //Serial.print("Deg angle: ");
    //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
    
  }

  void correctAngle(void)
  {
    //recalculate angle
    correctedAngle = degAngle - startAngle; //this tares the position

    if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
    {
    correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
    }
    else
    {
      //do nothing
    }
    //Serial.print("Corrected angle: ");
    //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
  }

  void checkQuadrant(void)
  {
    /*
    //Quadrants:
    4  |  1
    ---|---
    3  |  2
    */

    //Quadrant 1
    if(correctedAngle >= 0 && correctedAngle <=90)
    {
      quadrantNumber = 1;
    }

    //Quadrant 2
    if(correctedAngle > 90 && correctedAngle <=180)
    {
      quadrantNumber = 2;
    }

    //Quadrant 3
    if(correctedAngle > 180 && correctedAngle <=270)
    {
      quadrantNumber = 3;
    }

    //Quadrant 4
    if(correctedAngle > 270 && correctedAngle <360)
    {
      quadrantNumber = 4;
    }
    //Serial.print("Quadrant: ");
    //Serial.println(quadrantNumber); //print our position "quadrant-wise"

    if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
    {
      if(quadrantNumber == 1 && previousquadrantNumber == 4)
      {
        numberofTurns++; // 4 --> 1 transition: CW rotation
      }

      if(quadrantNumber == 4 && previousquadrantNumber == 1)
      {
        numberofTurns--; // 1 --> 4 transition: CCW rotation
      }
      //this could be done between every quadrants so one can count every 1/4th of transition

      previousquadrantNumber = quadrantNumber;  //update to the current quadrant
    
    }  
    //Serial.print("Turns: ");
    //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

    //after we have the corrected angle and the turns, we can calculate the total absolute position
    totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
    //Serial.print("Total angle: ");
    //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
  }

  void checkMagnetPresence(void)
  {  
    //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

    while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
    {
      magnetStatus = 0; //reset reading

      Wire.beginTransmission(0x36); //connect to the sensor
      Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
      Wire.endTransmission(); //end transmission
      Wire.requestFrom(0x36, 1); //request from the sensor

      while(Wire.available() == 0); //wait until it becomes available 
      magnetStatus = Wire.read(); //Reading the data after the request

      //Serial.print("Magnet status: ");
      //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
    }      
    
    //Status register output: 0 0 MD ML MH 0 0 0  
    //MH: Too strong magnet - 100111 - DEC: 39 
    //ML: Too weak magnet - 10111 - DEC: 23     
    //MD: OK magnet - 110111 - DEC: 55

    //Serial.println("Magnet found!");
    //delay(1000);  
  }

//------------------------------------------------------------------------
// Controller pre-baked functions
  void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos)
  {
    if(servo_pos<500)
      servo_pos = 500;
    else if(servo_pos>2500)
      servo_pos=2500;

    if(servo_pos>501)
      servo_pos=(((servo_pos-2)*2)-1000);
    else
      servo_pos=0;

    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write(servo_num-1);
    Wire.write(servo_pos>>8);
    Wire.write(servo_pos & 0XFF);
    Wire.endTransmission();
  }

  void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos)
  {
    if(servo_pos<500)
      servo_pos = 500;
    else if(servo_pos>2500)
      servo_pos=2500;
    servo_pos=((servo_pos*2)-1000);

    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write((servo_num-1)+(18*4));
    Wire.write(servo_pos>>8);
    Wire.write(servo_pos & 0XFF);
    Wire.endTransmission();
    delay(20);
  }

  void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos)
  {
    if(servo_pos<500)
      servo_pos = 500;
    else if(servo_pos>2500)
      servo_pos=2500;
    servo_pos=((servo_pos*2)-1000);

    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write((servo_num-1)+(18*3));
    Wire.write(servo_pos>>8);
    Wire.write(servo_pos & 0XFF);
    Wire.endTransmission();
    delay(20);
  }

  void I2C_SERVONUTRALSET(unsigned char servo_num,unsigned int servo_pos)
  {
    if(servo_pos<500)
      servo_pos = 500;
    else if(servo_pos>2500)
      servo_pos=2500;
    servo_pos=((servo_pos*2)-1000);

    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write((servo_num-1)+(18*5));
    Wire.write(servo_pos>>8);
    Wire.write(servo_pos & 0XFF);
    Wire.endTransmission();
  }

  void I2C_SERVOSPEED(unsigned char value)
  {
    Wire.beginTransmission(servo1);
    Wire.write(18*2);
    Wire.write(value);
    Wire.write(0);
    Wire.endTransmission();
    Wire.beginTransmission(servo2);
    Wire.write(18*2);
    Wire.write(value);
    Wire.write(0);
    Wire.endTransmission();
    delay(20);
  }

  void I2C_SERVOOFFSET(unsigned char servo_num,int value)
  {
    value=3000-value;
    value=value-1500;

    if (value<-500)
      value=-500;
    else if (value>500)
      value=500;

    if(value>0)
      value=2000+(value*2);
    else if(value<=0)
      value=-value*2;

    
    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write((servo_num-1)+(18*6));
    Wire.write(value>>8);
    Wire.write(value & 0XFF);
    Wire.endTransmission();
    delay(20);
  }

  void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir)
  {
    if(servo_dir>0)
      servo_dir=1;
    if(servo_num<19)
      Wire.beginTransmission(servo1);
    else
      Wire.beginTransmission(servo2);
    Wire.write((servo_num-1)+(18*7));
    Wire.write(servo_dir);
    Wire.write(0);
    Wire.endTransmission();
    delay(20);
  }

  char I2C_SERVOEND(void)
  {
    int i, n;
    char buffer;
    Wire.beginTransmission(servo1);
    n = Wire.write(181);
    if (n != 1)
      return (-10);

    n = Wire.endTransmission(false);
    if (n != 0)
      return (n);

    delayMicroseconds(350);
    Wire.requestFrom(servo1, 1, true);
    while(Wire.available())
      buffer=Wire.read();

    return(buffer);
  }

  int I2C_SERVOGET(int servo_num)
  {
    int i, n, error;
    uint8_t buffer[2];
    Wire.beginTransmission(servo1);

    n = Wire.write((servo_num-1)+(18*8));
    if (n != 1)
      return (-10);

    n = Wire.endTransmission(false);
    if (n != 0)
      return (n);

    delayMicroseconds(240);
    Wire.requestFrom(servo1, 2, true);
    i = 0;
    while(Wire.available() && i<2)
    {
      buffer[i++]=Wire.read();
    }
    if ( i != 2)
      return (-11);
    return (((buffer[0]*256 + buffer[1])+4)/2 +500);
  }

  int I2C_SERVOGETOFFSET(int servo_num)
  {
    int i, n, error;
    uint8_t buffer[2];
    Wire.beginTransmission(servo1);

    n = Wire.write((servo_num-1)+(182));
    if (n != 1)
      return (-10);

    n = Wire.endTransmission(false);
    if (n != 0)
      return (n);

    delayMicroseconds(240);
    Wire.requestFrom(servo1, 2, true);
    i = 0;
    while(Wire.available() && i<2)
    {
      buffer[i++]=Wire.read();
    }
    if ( i != 2)
      return (-11);
    i=((buffer[0]*256 + buffer[1]));
    if(i>2000)
      return(3000-(((i-2000)/2)+1500));
    else
      return(3000-((-i/2)+1500));
  }

  void ServoSetAll(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
  {
    if (Servo1 >= 500) {I2C_SERVOSET(1,Servo1);}
    if (Servo2 >= 500) {I2C_SERVOSET(2,Servo2);}
    if (Servo3 >= 500) {I2C_SERVOSET(3,Servo3);}
    if (Servo4 >= 500) {I2C_SERVOSET(4,Servo4);}
    if (Servo5 >= 500) {I2C_SERVOSET(5,Servo5);}
    if (Servo6 > 500) {I2C_SERVOSET(6,Servo6);}
    if (Servo7 >= 500) {I2C_SERVOSET(7,Servo7);}
    if (Servo8 >= 500) {I2C_SERVOSET(8,Servo8);}
    if (Servo9 >= 500) {I2C_SERVOSET(9,Servo9);}
    if (Servo10 >= 500) {I2C_SERVOSET(10,Servo10);}
    if (Servo11 >= 500) {I2C_SERVOSET(11,Servo11);}
    if (Servo12 >= 500) {I2C_SERVOSET(12,Servo12);}
    if (Servo13 >= 500) {I2C_SERVOSET(13,Servo13);}
    if (Servo14 >= 500) {I2C_SERVOSET(14,Servo14);}
    if (Servo15 >= 500) {I2C_SERVOSET(15,Servo15);}
    if (Servo16 >= 500) {I2C_SERVOSET(16,Servo16);}
    if (Servo17 >= 500) {I2C_SERVOSET(17,Servo17);}
    if (Servo18 >= 500) {I2C_SERVOSET(18,Servo18);}
    while (!I2C_SERVOEND())
    {
      delay(1);
    }
    LEDToggle();
  }

  void LEDToggle(void)
  {
    if (LEDState == 0)
      LEDState = 1;
    else
      LEDState = 0;
    digitalWrite(LED, LEDState);
  }
//------------------------------------------------------------------------