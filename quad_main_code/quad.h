#ifndef QUAD_H
#define QUAD_H  

#include <Arduino.h>
#include "quad_config.h"
/////////// Joystick variables /////////////////////////////

char joystick[3][10];
char joystick_bla;

int index1;
int j_index = 0;

#define FLOAT_SIZE 4 

union float_num
{
  char inp[FLOAT_SIZE];
  float val;
}u;
 
float_num command_roll,command_pitch,command_thrust;
 
////////////////////////////////////////////////////////////

/////////// IMU variables /////////////////////////////////
char IMU[4][100];
char IMU_bla;
char POS[3][100];  
char pos_bla;
int index2, index3;
int q_index = 0;
int p_index = 0;

float quaternion[4] = {0.0,0.0,0.0,0.0};
float q_Euler[3] = {0.0,0.0,0.0};

float_num q0,q1,q2,q3;

///////////////////////////////////////////////////////////

////////////////Motor Running Variables ///////////////////
Servo motor_left;
Servo motor_right;
Servo motor_front;
Servo motor_back;

int motor_left_pwm;
int motor_right_pwm;
int motor_front_pwm;
int motor_back_pwm;

/////////////////////////////////////////////////////////// 


///////////////Other operation variables///////////////////
int loop_start;

float angles[3]; // yaw pitch roll
float rates[6];

float roll,pitch,rollzero,pitchzero;
float zhuman;
float speeds[4];
float k, d , i, kr, dr , ir;
float pitch_set, roll_set, pitch_set_zero,roll_set_zero , roll_get, pitch_get;
float ipitch , iroll,  gyroX , gyroY , pitch_in, roll_in, gyroZ ;
int fly,c, count;
///////////////////////////////////////////////////////////

String in_string = "";    // string to hold input
int in_num;                // argument to be passed along with the string

void updateMotors() {
    motor_left.writeMicroseconds(motor_left_pwm);
    motor_right.writeMicroseconds(motor_right_pwm);
    motor_front.writeMicroseconds(motor_front_pwm);
    motor_back.writeMicroseconds(motor_back_pwm);
}  

void stopMotors() {

  int pwm_val = MOTOR_PWM_MIN;

  motor_left_pwm = pwm_val;
  motor_right_pwm = pwm_val;
  motor_front_pwm = pwm_val;
  motor_back_pwm = pwm_val;

  updateMotors();
}

void throttleToMotor() {
  int pwm_val = 0;
  if(command_pitch.val>0 && command_pitch.val<1)
    pwm_val = MOTOR_PWM_MIN + (command_pitch.val)*(MOTOR_PWM_MAX-MOTOR_PWM_MIN);  
  else
    pwm_val = MOTOR_PWM_MIN;
    
  motor_left_pwm = pwm_val;
  motor_right_pwm = pwm_val;
  motor_front_pwm = pwm_val;
  motor_back_pwm = pwm_val;
}

void parseSerialInput()
{
  char in_char = Serial3.read();
  if(in_char == JOYSTICK_START)
  {
    parseJoyStickInput();
  }
  else
   {
     if(in_char == MESSAGE_START)
        parseMessage();
     else if(in_char == EMERGENCY_STOP)
        stopMotors();
   }
}

void parseMessage()
{
  boolean FLAG_STR_END = false;
  boolean FLAG_INP_ERROR = false;
  boolean FLAG_VALID_INP = false;
  int inp;
  
  //////////////READING THE INPUT//////////////
  while(Serial3.available()>0)
  {  
    char in_char = Serial3.read();

    if((in_char != ' ')&& FLAG_STR_END==false)
    {
      in_string += in_char;
    }
    else if(in_char == ' ')
    {
      FLAG_STR_END = true;
    }
    else if(FLAG_STR_END == true)
    {
      if(isDigit(in_char))
        in_num = in_num*10 + (in_char-'0');
      else
        FLAG_INP_ERROR = true;
    }
    delay(1);      //this delay is necessary for the proper functioning of Serial.
  }
  
  //////////////////////////////////////////////////
  
  ////////COMPARING WITH EXISTING COMMANDS//////////

  if(in_string.equals("LEFT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX)
      motor_left_pwm = in_num;
  }
  

  if(in_string.equals("RIGHT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX)
      motor_right_pwm = in_num;
  }

  if(in_string.equals("FRONT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX)
      motor_front_pwm = in_num;
  }

  if(in_string.equals("BACK"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX)
      motor_back_pwm = in_num;
  }

  if(in_string.equals("ALL"))
  {
    FLAG_VALID_INP = true;
    int pwm_val = MOTOR_PWM_MIN;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX)
      pwm_val = in_num;
    motor_left_pwm = pwm_val;
    motor_right_pwm = pwm_val;
    motor_front_pwm = pwm_val;
    motor_back_pwm = pwm_val;
  }
  /////////////////////////////////////////////////////
  
  ////////////CHECKING IF THE INPUT IS VALID///////////
  if(FLAG_VALID_INP == false)
  {
    Serial3.println("INVALID INPUT");
  }
  /////////////////////////////////////////////////////
  
  
  ////////////PRINTING MOTOR STATES////////////////////
  Serial3.print(" Motor PWM : left- ");
  Serial3.print(motor_left_pwm);
  Serial3.print(" right- ");
  Serial3.print(motor_right_pwm);
  Serial3.print(" front- ");
  Serial3.print(motor_front_pwm);
  Serial3.print(" back- ");
  Serial3.print(motor_back_pwm);  
  Serial3.println("  |  ");
  /////////////////////////////////////////////////////
  in_string = "";
  in_num = 0;
}

void parseJoyStickInput()
{  
  
  in_string = "";
  boolean FLAG_INPUT_PACKET_END = false;
  
  
  while(FLAG_INPUT_PACKET_END == false) {
      
      joystick_bla = Serial3.read();
      
      
      if (joystick_bla == ',') {
        j_index = j_index + 1;
        index1 = 0;
      }
      else if (joystick_bla == ':') {
        FLAG_INPUT_PACKET_END = true;
//        printJoyStickInput();
        //FLAG_INPUT_PACKET_END = false;
        j_index = 0;
        index1 = 0;
      }
      else {
        if(joystick_bla>='A' && joystick_bla<='F')
        {  
          joystick[j_index][index1] = joystick_bla-'A'+10;
          index1++;
        }
        else
       {
         if(joystick_bla>='0'&&joystick_bla<='9')
         {
           joystick[j_index][index1] = joystick_bla-'0';
           index1++;
         }
       }
            
    }
  }
  if(FLAG_INPUT_PACKET_END == true)
  {
    //FLAG_INPUT_PACKET_END = false;
    
    for(int i=0;i<FLOAT_SIZE;i++)
        command_roll.inp[i] = joystick[0][2*i]*16 + joystick[0][2*i+1];
  
    for(int i=0;i<FLOAT_SIZE;i++)
        command_pitch.inp[i] = joystick[1][2*i]*16 + joystick[1][2*i+1];
    
    for(int i=0;i<FLOAT_SIZE;i++)
        command_thrust.inp[i] = joystick[2][2*i]*16 + joystick[2][2*i+1];
  }  
  throttleToMotor();
}

void parseIMUInput()
{
  boolean FLAG_IMU_PACKET_END = false;
  while (Serial2.available() > 0) {
      IMU_bla = Serial2.read();
      if (IMU_bla == ',') {
        q_index = q_index + 1;
        index2 = 0;
      }
      else if (IMU_bla == ':') {
        FLAG_IMU_PACKET_END = true;
//        printIMUReadings();
        //FLAG_IMU_PACKET_END = false;
        q_index = 0;
        index2 = 0;
      }
      else {
        if(IMU_bla>='A')
          IMU[q_index][index2] = IMU_bla-'A'+10;
        else if(IMU_bla>='0'&&IMU_bla<='9')
          IMU[q_index][index2] = IMU_bla-'0';
        else
          IMU[q_index][index2] = 0;
        index2++;
    }
  }
  if(FLAG_IMU_PACKET_END == true)
  {
    //FLAG_IMU_PACKET_END = false;
    for(int i=0;i<FLOAT_SIZE;i++)
        q0.inp[i] = IMU[0][2*i]*16 + IMU[0][2*i+1];
  
    for(int i=0;i<FLOAT_SIZE;i++)
        q1.inp[i] = IMU[1][2*i]*16 + IMU[1][2*i+1];
    
    for(int i=0;i<FLOAT_SIZE;i++)
        q2.inp[i] = IMU[2][2*i]*16 + IMU[2][2*i+1];
        
    for(int i=0;i<FLOAT_SIZE;i++)
        q3.inp[i] = IMU[3][2*i]*16 + IMU[3][2*i+1];
  }
}

void printJoyStickInput()
{
  Serial.print("Joy Inputs: ");
  Serial.print(command_roll.val,4);
  Serial.print(' ');
  //Serial.print("Pitch Input: ");
  Serial.print(command_pitch.val,4);
  Serial.print(' ');
  //Serial.print("Thrust Input: ");
  Serial.print(command_thrust.val,4);
  Serial.print("  |  ");
  
}

void quaternionToEuler(float *q, float *euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void printMotorPWM() {
  Serial.print(" Motor PWM : left- ");
  Serial.print(motor_left_pwm);
  Serial.print(" right- ");
  Serial.print(motor_right_pwm);
  Serial.print(" front- ");
  Serial.print(motor_front_pwm);
  Serial.print(" back- ");
  Serial.print(motor_back_pwm);  
  Serial.print("  |  ");
}

void printIMUReadings()
{
  quaternion[0] = q0.val;
  quaternion[1] = q1.val;
  quaternion[2] = q2.val;
  quaternion[3] = q3.val;

  quaternionToEuler(quaternion, q_Euler);
  Serial.print("Yaw: ");
  Serial.print(degrees(q_Euler[0]),4);
  Serial.print(' ');
  Serial.print("Pitch: ");
  Serial.print(degrees(q_Euler[1]),4);
  Serial.print(' ');
  Serial.print("Roll: ");
  Serial.print(degrees(q_Euler[2]),4);
  Serial.print("  |  ");
}

void initializeIMU() {
  loop_start = 0;
  count = 0;
   k = 0.00;
   d = 0.00;
   i = 0.00;
   kr = 6.00;
   dr = 1.10;
   ir = 0.00;
     
    roll = 0;
    pitch = 0;
    
    gyroX = 0;
    gyroY = 0;
    roll_set = 0;
    pitch_set = 0;
    roll_set_zero = 0;
    pitch_set_zero = 0;
    zhuman = 800.0;
    iroll = 0; 
    ipitch = 0;
}

void initializeMotors() {
      /// motor input settings////
    
    in_string.reserve(30);

    motor_left.attach(MOTOR_LEFT_PIN);
    motor_right.attach(MOTOR_RIGHT_PIN);
    motor_front.attach(MOTOR_FRONT_PIN);
    motor_back.attach(MOTOR_BACK_PIN);
    
    motor_left_pwm = MOTOR_PWM_MIN;
    motor_right_pwm = MOTOR_PWM_MIN;
    motor_front_pwm = MOTOR_PWM_MIN;
    motor_back_pwm = MOTOR_PWM_MIN;
    
    motor_left.writeMicroseconds(motor_left_pwm);
    motor_right.writeMicroseconds(motor_right_pwm);
    motor_front.writeMicroseconds(motor_front_pwm);
    motor_back.writeMicroseconds(motor_back_pwm);
}

#endif
