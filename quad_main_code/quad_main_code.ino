

//#include <PinChangeInt.h>
//#include <Timer.h>

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include "Timer.h"

#include "quad.h"
#include "quad_config.h"

////////TIMER VARIABLES////////////////////////////
long timeNow=0; // Hold the milliseond value for now
long timeOld;
int delT;
///////////////////////////////////////////////////


void setup() {
  
  
  Serial.begin(57600);  // lappy
  Serial2.begin(115200);  // imu
  Serial3.begin(57600);  // xbee
  Wire.begin();
  
  Serial.println("\n\nSTARTING up ");
  
  delay(5);
  //delay(5000);
  
  initializeIMU();
  initializeMotors();
  
  readParamFromEEPROM();
  delay(10000); 
  timeOld = millis();
  
}

void loop() {
  timeNow = millis();
  delT = timeNow-timeOld;
  
  if (Serial3.available() > 0) {                              // Joystick reading available
      parseSerialInput();
    }
  if (Serial2.available() > 0) {                              // IMU reading available
      parseIMUInput();
  }
  if (Serial.available() > 0) {                               // Instructions from lappy
    setIMUoffset();
  }
  
  if(delT >= 20)                                //main control loop runs at 50Hz
  {  
    timeOld = timeNow;
    if(USER_OVERRIDE == false)
    {
      updateControlParams();                                      // Updates parameters used by control algo
      executePDController();                                        // Evaluates the control inputs U1, U2, U3, U4
      getPWM();                                                   // Evaluates PWM signal based on U1, U2, U3, U4
      getInBounds();
      updateMotors();                                             // Provides PWM signal
    }
    if(FLAG_SEND_DATA == true)
    {
        sendDataMRF(delT);                                              //sends the IMU data and motor PWMs via 
        //Serial3(xbee) in MRF(MATLAB Readable Format)
    }
    
    /*uncommenting this block leads to time step between two iterations incease to more than 20ms
    because of which control code no longer runs at 50Hz*/
    
    //  printJoyStickInput();                
    //  printIMUReadings();
    //  printMotorPWM();
    
//    Serial.print(pose_setpoints[0]);
//    Serial.print(", ");
//    Serial.print(pose_setpoints[1]);
//    Serial.print(", ");
//    Serial.print(pose_setpoints[2]);
//    Serial.print(", ");
//    Serial.print(m);
//    Serial.print(" ");
  Serial.print(motor_front_pwm);    //sending front PWM
  Serial.print(',');
  Serial.print(motor_left_pwm);    //sending left PWM  
  Serial.print(',');
  Serial.print(motor_back_pwm);    //sending back PWM
  Serial.print(',');
  Serial.print(motor_right_pwm);    //sending right PWM
  Serial.print(',');
//    
    Serial.println(delT);
  }
}


