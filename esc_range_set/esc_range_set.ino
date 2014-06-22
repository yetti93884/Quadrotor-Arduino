//This code sets the ESCs to be reprogrammed to latch on to the maximum and the minimum PWM value

#include "Servo.h"

#define MOTOR_PIN 11
#define MOTOR_PWM_MIN 800
#define MOTOR_PWM_MAX 2000

Servo motor;
int motor_pwm;

void setup()
{
  Serial.begin(57600);
  motor.attach(MOTOR_PIN);
  
  Serial.println("ESC throttle range setting mode");
  Serial.print("BLDC on PIN ");
  Serial.println(MOTOR_PIN);
  Serial.println("KEEP POWER to motors SWITCHED OFF NOW");
  delay(1000);
  Serial.println("---------PRESS 'S' to INITIATE ...");
  
  ////////////WAITING TO INITIATE///////////////
  boolean FLAG_INITIATE = false;
  
  while(FLAG_INITIATE == false)
  {
    if(Serial.available())
    {
      char in_char = Serial.read();
      
      if(in_char == 'S')
      {
        FLAG_INITIATE = true;
        Serial.println("Entering THROTTLE LATCH mode");
        delay(1000);
      }
    }
  }
  ////////////////////////////////////////////////
  
  /////////SETTING THROTTLE TO MAX////////////////
  Serial.println("Setting throttle to max now");
  motor.writeMicroseconds(MOTOR_PWM_MAX);
  Serial.print("THROTTLE : ");
  Serial.println(MOTOR_PWM_MAX);
  delay(2000);
  ////////////////////////////////////////////////
  
  Serial.println("--------POWER up the motors NOW-----\n\n");
  
  Serial.println("ESC will do the self test. Emit special tone like 123.");
  Serial.println("After this it will emit 3 Beeps corresponding to number of batteries");
  Serial.println("Wait for 2 seconds. ESC will emit Beep twice which means ESC has latched maximum throttle position");
  
  Serial.println("\n\n--------PRESS 'Z' within 5 seconds of this beep---------");
  
  //////WAITING FOR USER TO SET THROTTLE TO ZERO//////
  
  boolean FLAG_SET_ZERO = false;
  
  while(FLAG_SET_ZERO == false)
  {
    if(Serial.available())
    {
      char in_char = Serial.read();
      if(in_char == 'Z')
      {
        FLAG_SET_ZERO = true;
        Serial.println("SETTING THROTTLE TO MINIMUM ");
        motor.writeMicroseconds(MOTOR_PWM_MIN);
        Serial.print("THROTTLE : ");
        Serial.println(MOTOR_PWM_MIN);
        delay(2000);
      }
    }
  }
  /////////////////////////////////////////////////////
  
  Serial.println("IF you heard a long beep ESC has latched to the zero position of throttle");
  Serial.println("ELSE REPEAT");
  delay(1000);
  
  Serial.println("\n\nENTERING KEY BOARD CONTROL MODE");
  Serial.println("---------PRESS 'q' to increase and 'a' to decrease motor PWM value");  
  motor_pwm = MOTOR_PWM_MIN;
}

void loop()
{
  if(Serial.available())
  {
    char in_char = Serial.read();
    
    if(in_char == 'q') {
      motor_pwm += 5;}
    if(in_char == 'a') {
      motor_pwm -= 5;}
      
    if(motor_pwm > MOTOR_PWM_MAX)
      motor_pwm = MOTOR_PWM_MAX;
    if(motor_pwm < MOTOR_PWM_MIN)
      motor_pwm = MOTOR_PWM_MIN;
  }
  
  Serial.print("THROTTLE : ");
  Serial.println(motor_pwm);
  motor.writeMicroseconds(motor_pwm);
  delay(20);
}

