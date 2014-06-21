

//#include <PinChangeInt.h>
//#include <Timer.h>

#include <Wire.h>
#include <Servo.h>
#include "Timer.h"

#include "quad.h"

#include "quad_config.h"

void setup() {
  Serial.begin(57600);  // lappy
  Serial2.begin(115200);  // imu
  Serial3.begin(57600);  // xbee
  Wire.begin();
  
  //delay(5);
  delay(5);

  initializeIMU();
  initializeMotors();
  
}

void loop() {
  
  if (Serial3.available() > 0) {                              // Joystick reading available
    parseSerialInput();
  }
  
  
  if (Serial2.available() > 0) {                              // IMU reading available
    parseIMUInput();
  }

  if (Serial.available() > 0) {                               // Instructions from lappy
    setIMUoffset();
  }
  
  updateControlParams();                                      // Updates parameters used by control algo
  executeController();                                        // Evaluates the control inputs U1, U2, U3, U4
  getPWM();                                                   // Evaluates PWM signal based on U1, U2, U3, U4
  updateMotors();                                             // Provides PWM signal
  
  printJoyStickInput();
  printIMUReadings();
  printMotorPWM();
  Serial.println();
}


