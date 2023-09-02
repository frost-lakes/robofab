#include <Servo.h>

Servo servo1, servo2, servo3, servo4, servo5, servo6;  // create servo object to control a servo 
unsigned int servopos[6] = {500, 500, 500, 500, 500, 500} ; // uint to hold servo position in microseconds (0-180 deg is mapped to to 500-2500 us), initialised to 0 deg
String inString = "";  // string to hold input
int counter = 0; //Counter for the servopos array

void setup() {
  servo1.attach(2,500,2500);  // (pin, min angle us, max angle us)
  servo2.attach(3,500,2500);
  servo3.attach(4,500,2500);
  servo4.attach(5,500,2500);
  servo5.attach(6,500,2500);
  servo6.attach(7,500,2500);
  
  // Initialise all servos to 0 deg
  servo1.writeMicroseconds(servopos[0]); 
  servo2.writeMicroseconds(servopos[1]);
  servo3.writeMicroseconds(servopos[2]);
  servo4.writeMicroseconds(servopos[3]);
  servo5.writeMicroseconds(servopos[4]);
  servo6.writeMicroseconds(servopos[5]); 
  
  Serial.begin(9600); // Start the serial communication
 
}

void loop() {
 
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    if (inChar == 'p') {
      counter++;
    }   
    // if we get an underscore, store the int value in the respective servopos element:
    if (inChar == '_') {
      servopos[counter] = inString.toInt();
      counter++;
      // clear the string for new input:
      inString = "";
    }
    if (inChar == 'e') {
        // Write the new servo positions
        servo1.writeMicroseconds(servopos[0]); 
        servo2.writeMicroseconds(servopos[1]);
        servo3.writeMicroseconds(servopos[2]);
        servo4.writeMicroseconds(servopos[3]);
        servo5.writeMicroseconds(servopos[4]);
        servo6.writeMicroseconds(servopos[5]); 
        // Reset the counter
        counter = 0;
    }
  }
}
