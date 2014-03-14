#include <Servo.h>
#include <Wire.h>

byte firstByte;
byte secondByte;
Servo servos[2];

void setup()
{
  // Set I2C Bus address of arduino slave device
  Wire.begin(0x69);
  
  // Initialize serial for console debugging/display
  Serial.begin(115200);
 
  // Attach servos to pin 6 and 7
  servos[0].attach(6);
  servos[1].attach(7);
  
  // Set servos to "middle" position at 90 degrees
  servos[0].write(90);
  servos[1].write(90);

  // Powering IMU with Arduino 3.3V pin 12
  pinMode(12, HIGH);
  digitalWrite(12, HIGH);
  Wire.onReceive(receiveEvent);
}

void loop()
{ 
  delayMicroseconds(10);
}


// Handler for receiving data from Raspberry Pi
void receiveEvent(int howMany)
{  
    firstByte = Wire.read();            // receive first byte // servo1Value
    secondByte = Wire.read();           // receive second byte // servo2Value
    
    // Print Debugging Data
    Serial.print(firstByte);          
    Serial.print(" ");
    Serial.println(secondByte);
  
    servos[0].write(firstByte);
    servos[1].write(secondByte);
}
