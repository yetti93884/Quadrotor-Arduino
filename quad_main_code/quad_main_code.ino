

//#include <PinChangeInt.h>
//#include <Timer.h>

#include <Wire.h>
#include <Servo.h>

#include "quad.h"

#include "quad_config.h"

void setup() {
  Serial.begin(57600);  // lappy
  Serial2.begin(57600);  // imu
  Serial3.begin(57600);  // xbee
  Wire.begin();
  
  delay(5);
  delay(5);

  initializeIMU();
  initializeMotors();
  
}

void loop() {
  
  if (Serial3.available() > 0) {                              // Joystick reading available
    parseSerialInput();
  }
  
  
  if (Serial2.available() > 0) {                              // Xbee reading available
    parseIMUInput();
  }

  updateMotors();
  
  printJoyStickInput();
  printIMUReadings();
  printMotorPWM()
  ;
  Serial.println();
}


