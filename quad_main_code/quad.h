#ifndef QUAD_H
#define QUAD_H  

#include <Arduino.h>
#include "quad_config.h"

/////////// Control Execution Constants  ///////////////////
boolean USER_OVERRIDE = false;    // turns true on emergency stop which disable controller to set pwms anymore.
boolean FLAG_SEND_DATA = false;   // turns on sending data in MRF format

////////////////////////////////////////////////////////////

/////////// Joystick variables /////////////////////////////

char joystick[3][10];
char joystick_bla;

int index1;
int j_index = 0;

#define FLOAT_SIZE 4
#define INT_SIZE 4

union float_num
{
  char inp[FLOAT_SIZE];
  float val;
}u;
 
float_num command_roll,command_pitch,command_thrust;
 
////////////////////////////////////////////////////////////
union int_num
{
  char inp[INT_SIZE];
  int val;
}junk_num;

/////////// IMU variables /////////////////////////////////
char IMU[4][100];
char IMU_bla;
char POS[3][100];  
char pos_bla;
int index2, index3;
int q_index = 0;
int p_index = 0;

float quaternion[4] = {0.0,0.0,0.0,0.0};
float bla_quaternion[4] = {0.0,0.0,0.0,0.0};
float home_quaternion[4] = {0.0,0.0,0.0,0.0};
float q_Euler[3] = {0.0,0.0,0.0};
boolean FLAG_IMU_PACKET_END = false;
boolean IMU_offset_set = false;  // true if offset is set

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

float thrust_pwm_constant = 1.846;    // Thrust = 1.846*(pwm - 819.6)
float torque_pwm_constant = 1.858;  // scaled by 10**4
int thrust_pwm_min = (int)900;  //900 is good
int torque_pwm_min = (int)1144.24;

/////////////////////////////////////////////////////////// 


///////////////Other operation variables///////////////////
int last_pose_update = millis();    // time when last pose update is carried out - in milli seconds
int last_pos_update = millis();    // time when last position update is carried out - in milli seconds
Timer t;
float pose_dt = 0.0;
float position_dt = 0.0;


///////////////Control Parameters//////////////////////////
float pose[3] = {0.0, 0.0, 0.0}; // yaw pitch roll
float angularrates[3] = {0.0, 0.0, 0.0};  //yaw_dot, pitch_dot, roll_dot
float pose_setpoints[3] = {0.0, 0.0, 0.0};  // yaw_set, pitch_set, roll_set
float position_setpoints[3] = {0.0, 0.0, 0.0};  //x,y,z
float quad_position[3];        //x,y,z
float velocities[3];

float speeds[4];
float alpha1 = 0.1;
float alpha2 = 0.1;
float alpha3 = 0.1;
float alpha4 = 0.1;
float alpha5 = 2.5;
float alpha6 = 1.5;
float alpha7 = 0.1;
float alpha8 = 0.1;
float U1, U2, U3, U4;

/////////////Physical Parameters//////////////////////////
float m = 1.1;
float g = 9.8;
float Ixx = 0.043675;//0.0085;
float Iyy = 0.043675;//0.0085;
float Izz = 0.082800;//0.0165;
float a1 = (Iyy-Izz)/Ixx;
float a3 = (Izz-Ixx)/Iyy;
float a5 = (Ixx-Iyy)/Izz;
float b1 = 1/Ixx;
float b2 = 1/Iyy;
float b3 = 1/Izz;
///////////////////////////////////////////////////////////

String in_string = "";    // string to hold input
int in_num;                // argument to be passed along with the string
float in_float = 0.0;            // argument to be passed along with the string

void getInBounds() {
    if (motor_left_pwm <= MOTOR_PWM_MIN) {
      motor_left_pwm = MOTOR_PWM_MIN;
    }
    else if (motor_left_pwm >= MOTOR_PWM_MAX){
      motor_left_pwm = MOTOR_PWM_MAX;
    }
    
    if (motor_right_pwm <= MOTOR_PWM_MIN) {
      motor_right_pwm = MOTOR_PWM_MIN;
    }
    else if (motor_right_pwm >= MOTOR_PWM_MAX){
      motor_right_pwm = MOTOR_PWM_MAX;
    }
    
    if (motor_front_pwm <= MOTOR_PWM_MIN) {
      motor_front_pwm = MOTOR_PWM_MIN;
    }
    else if (motor_front_pwm >= MOTOR_PWM_MAX){
      motor_front_pwm = MOTOR_PWM_MAX;
    }
    
    if (motor_back_pwm <= MOTOR_PWM_MIN) {
      motor_back_pwm = MOTOR_PWM_MIN;
    }
    else if (motor_back_pwm >= MOTOR_PWM_MAX){
      motor_back_pwm = MOTOR_PWM_MAX;
    }
}
void updateMotors() {
    //delay(20);
    
    motor_front.writeMicroseconds(motor_front_pwm);
    motor_right.writeMicroseconds(motor_right_pwm);
    motor_left.writeMicroseconds(motor_left_pwm);
    motor_back.writeMicroseconds(motor_back_pwm);
    
//    Serial.println();
//    printMotorPWM();
//    Serial.println();
}  

void stopMotors() {

  int pwm_val = MOTOR_ARM_PWM;

  motor_left_pwm = pwm_val;
  motor_right_pwm = pwm_val;
  motor_front_pwm = pwm_val;
  motor_back_pwm = pwm_val;
  
  motor_right.writeMicroseconds(motor_right_pwm);
  motor_left.writeMicroseconds(motor_left_pwm);
  motor_front.writeMicroseconds(motor_front_pwm);
  motor_back.writeMicroseconds(motor_back_pwm);
  //delay(10000);

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
        {
          Serial3.println("EMERGENCY STOP FOR MOTORS");
          USER_OVERRIDE = true;
          stopMotors();
        }
     else if(in_char == CONTROL_RESTART)
       {
          Serial3.println("CONTROL RESTART FOR MOTORS");
          USER_OVERRIDE = false;
          //updateMotors();
       }
   }
}

void parseMessage()
{
  boolean FLAG_STR_END = false;
  boolean FLAG_INP_ERROR = false;
  boolean FLAG_VALID_INP = false;
  int decimal_count = 0;
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
      if(isDigit(in_char)) {
        in_num = in_num*10 + (in_char-'0');
      }
      else if (in_char == '.')
        decimal_count++;
      else
        FLAG_INP_ERROR = true;
    }
    delay(1);      //this delay is necessary for the proper functioning of Serial.
  }
  
  in_float = in_num/pow(10.0, decimal_count);
  decimal_count = 0;
  
  //////////////////////////////////////////////////
  
  ////////COMPARING WITH EXISTING COMMANDS//////////

  if(in_string.equals("LEFT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX) {
      motor_left_pwm = in_num;
    }
    else if (in_num <= MOTOR_PWM_MIN) {
      motor_left_pwm = MOTOR_PWM_MIN;
    }
    else {
      motor_left_pwm = MOTOR_PWM_MAX;
    }
    
  }
  

  if(in_string.equals("RIGHT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX) {
      motor_right_pwm = in_num;
    }
    else if (in_num <= MOTOR_PWM_MIN) {
      motor_right_pwm = MOTOR_PWM_MIN;
    }
    else {
      motor_right_pwm = MOTOR_PWM_MAX;
    }
  }

  if(in_string.equals("FRONT"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX) {
      motor_front_pwm = in_num;
    }
    else if (in_num <= MOTOR_PWM_MIN) {
      motor_front_pwm = MOTOR_PWM_MIN;
    }
    else {
      motor_front_pwm = MOTOR_PWM_MAX;
    }
  }

  if(in_string.equals("BACK"))
  {
    FLAG_VALID_INP = true;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX) {
      motor_back_pwm = in_num;
    }
    else if (in_num <= MOTOR_PWM_MIN) {
      motor_back_pwm = MOTOR_PWM_MIN;
    }
    else {
      motor_back_pwm = MOTOR_PWM_MAX;
    }
  }

  if(in_string.equals("ALL"))
  {
    FLAG_VALID_INP = true;
    int pwm_val = MOTOR_PWM_MIN;
    if(in_num>=MOTOR_PWM_MIN && in_num<=MOTOR_PWM_MAX) {
      pwm_val = in_num;
    }
    else if (in_num <= MOTOR_PWM_MIN) {
      pwm_val = MOTOR_PWM_MIN;
    }
    else {
      pwm_val = MOTOR_PWM_MAX;
    }
    motor_left_pwm = pwm_val;
    motor_right_pwm = pwm_val;
    motor_front_pwm = pwm_val;
    motor_back_pwm = pwm_val;
  }
  
  if (in_string.equals("ALPHA1")) {
    FLAG_VALID_INP = true;
    alpha1 = in_float;
  }
  
  if (in_string.equals("ALPHA2")) {
    FLAG_VALID_INP = true;
    alpha2 = in_float;
  }
  
  if (in_string.equals("ALPHA3")) {
    FLAG_VALID_INP = true;
    alpha3 = in_float;
  }
  
  if (in_string.equals("ALPHA4")) {
    FLAG_VALID_INP = true;
    alpha4 = in_float;
  }
  
  if (in_string.equals("ALPHA5")) {
    FLAG_VALID_INP = true;
    alpha5 = in_float;
  }
  
  if (in_string.equals("ALPHA6")) {
    FLAG_VALID_INP = true;
    alpha6 = in_float;
  }
  
  if (in_string.equals("ALPHA7")) {
    FLAG_VALID_INP = true;
    alpha7 = in_float;
  }
  
  if (in_string.equals("ALPHA8")) {
    FLAG_VALID_INP = true;
    alpha8 = in_float;
  }
  
  Serial.println();
  Serial.print("Alphas: ");
  Serial.print(alpha1);
  Serial.print(" ");
  Serial.print(alpha2);
  Serial.print(" ");
  Serial.print(alpha3);
  Serial.print(" ");
  Serial.print(alpha4);
  Serial.print(" ");
  Serial.print(alpha5);
  Serial.print(" ");
  Serial.print(alpha6);
  Serial.print(" ");
  Serial.print(alpha7);
  Serial.print(" ");
  Serial.print(alpha8);
  Serial.println();
    
  if(in_string.equals("SHOW"))
  {
    FLAG_VALID_INP = true;
    Serial3.print("Alphas: ");
    Serial3.print(alpha1);
    Serial3.print(" ");
    Serial3.print(alpha2);
    Serial3.print(" ");
    Serial3.print(alpha3);
    Serial3.print(" ");
    Serial3.print(alpha4);
    Serial3.print(" ");
    Serial3.print(alpha5);
    Serial3.print(" ");
    Serial3.print(alpha6);
    Serial3.print(" ");
    Serial3.print(alpha7);
    Serial3.print(" ");
    Serial3.print(alpha8);
    Serial3.println();
  }
  
  if (in_string.equals("SEND_BINARY")) {
    FLAG_VALID_INP = true;
    FLAG_SEND_DATA = true;
  }
  if (in_string.equals("STOP_BINARY")) {
    FLAG_VALID_INP = true;
    FLAG_SEND_DATA = false;
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
  //boolean FLAG_IMU_PACKET_END = false;
  while (Serial2.available() > 0) {
      IMU_bla = Serial2.read();
      //Serial3.write(IMU_bla);
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
  
  bla_quaternion[0] = q0.val;
  bla_quaternion[1] = q1.val;
  bla_quaternion[2] = q2.val;
  bla_quaternion[3] = q3.val;
  
  if (IMU_offset_set == true) {
    quatProd(home_quaternion, bla_quaternion, quaternion);
  }
  else {
    quaternion[0] = bla_quaternion[0];
    quaternion[1] = bla_quaternion[1];
    quaternion[2] = bla_quaternion[2];
    quaternion[3] = bla_quaternion[3];
  }
  
  quaternionToEuler(quaternion, q_Euler);
  pose_dt = (millis() - last_pose_update)/1000.0;
  last_pose_update = millis();
    
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
//  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
//  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
//  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
  euler[1] = -atan2(2 * q[0] * q[1] + 2 * q[2] * q[3], 1 - 2 * q[1]*q[1] - 2 * q[2] * q[2]); // phi
  euler[2] = -asin(2 * q[0] * q[2] - 2 * q[3] * q[1]); // theta
  euler[0] = atan2(2 * q[0] * q[3] + 2 * q[1] * q[2], 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]); // psi
}

void quatConjugate(float *quat, float *result) {
  result[0] = quat[0];
  result[1] = -quat[1];
  result[2] = -quat[2];
  result[3] = -quat[3];
}

void quatProd(float * a, float * b, float * result) {
  result[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  result[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  result[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  result[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
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
  /*quaternion[0] = q0.val;
  quaternion[1] = q1.val;
  quaternion[2] = q2.val;
  quaternion[3] = q3.val;*/

  //quaternionToEuler(quaternion, q_Euler);
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
}

void setIMUoffset() {
  char input = Serial.read();
  if (input == 'h') {
    Serial.println("Setting home quaternion!!");
    quatConjugate(quaternion, home_quaternion);
    IMU_offset_set = true;
  }
}

void initializeMotors() {
      /// motor input settings////
    
    in_string.reserve(30);

    motor_left.attach(MOTOR_LEFT_PIN);
    motor_right.attach(MOTOR_RIGHT_PIN);
    motor_front.attach(MOTOR_FRONT_PIN);
    motor_back.attach(MOTOR_BACK_PIN);
    
    motor_left_pwm = MOTOR_ARM_PWM;
    motor_right_pwm = MOTOR_ARM_PWM;
    motor_front_pwm = MOTOR_ARM_PWM;
    motor_back_pwm = MOTOR_ARM_PWM;
    
    motor_left.writeMicroseconds(motor_left_pwm);
    motor_right.writeMicroseconds(motor_right_pwm);
    motor_front.writeMicroseconds(motor_front_pwm);
    motor_back.writeMicroseconds(motor_back_pwm);
}

void updateControlParams() {
  
  if (FLAG_IMU_PACKET_END == true) {
    angularrates[0] = (q_Euler[0] - pose[0])/pose_dt;
    pose[0] = q_Euler[0];
    
    angularrates[1] = (q_Euler[1] - pose[1])/pose_dt;
    pose[1] = q_Euler[1];
    
    angularrates[2] = (q_Euler[2] - pose[2])/pose_dt;
    pose[2] = q_Euler[2];
    
    FLAG_IMU_PACKET_END = false;
  }
  
  quad_position[0] = 0.0;
  quad_position[1] = 0.0;
  quad_position[2] = 0.0;
  velocities[0] = 0.0;
  velocities[1] = 0.0;
  velocities[2] = 0.0;
  //Serial3.println(pose[0]);
}
  
void executeController() {  //psi,theta,phi  x,y,z
  float z1 = pose_setpoints[2] - pose[2];
  float z3 = pose_setpoints[1] - pose[1];
  float z5 = pose_setpoints[0] - pose[0];
  float z7 = position_setpoints[2] - quad_position[2];
  float z2 = angularrates[2] - alpha1*z1;
  float z4 = angularrates[1] - alpha3*z3;
  float z6 = angularrates[0] - alpha5*z5;
  float z8 = velocities[2] - alpha7*z7;
  Serial.print("Check: ");
  Serial.print(alpha5*z5);
  Serial.print(" ");
  Serial.print(z6);
  
  U1 = m*(z7 + g - alpha7*(z8+alpha7*z7) - alpha8*z8)/(cos(pose[2])*cos(pose[1]));
  U2 = (z1 - a1*angularrates[1]*angularrates[0] - alpha1*(z2+alpha1*z1) - alpha2*z2)/b1;
  U3 = (z3 - a3*angularrates[2]*angularrates[0] - alpha3*(z4+alpha3*z3) - alpha4*z4)/b2;
  U4 = (z5 - a5*angularrates[1]*angularrates[2] - alpha5*(z6+alpha5*z5) - alpha6*z6)/b3;
  
  Serial.print("Control Inputs:  ");
  Serial.print(U1);
  Serial.print(" ");
  Serial.print(U2);
  Serial.print(" ");
  Serial.print(U3);
  Serial.print(" ");
  Serial.println(U4);
  
  U1 = m*9.8;	// only yaw control
  U2 = U3 = 0.0;	// only yaw control
}

void getPWM() {
//  thrust_pwm_constant, torque_pwm_constant
//  motor_left_pwm 
//  = (U1 + 0.2126)/0.0001858;
  Serial.print("Motor pwm: ");
  Serial.print(thrust_pwm_constant);
  Serial.print(" ");
  Serial.print(torque_pwm_constant);
  Serial.print(" ");
  
  if (USER_OVERRIDE == false) {
    motor_front_pwm = (int)(thrust_pwm_min + 0.5*(0.5*(U1/thrust_pwm_constant + U4*10000/torque_pwm_constant)+U3/thrust_pwm_constant));
    motor_back_pwm = (int)(thrust_pwm_min + 0.5*(0.5*(U1/thrust_pwm_constant + U4*10000/torque_pwm_constant)-U3/thrust_pwm_constant));
    motor_right_pwm = (int)(thrust_pwm_min + 0.5*(0.5*(U1/thrust_pwm_constant - U4*10000/torque_pwm_constant)+U2/thrust_pwm_constant));
    motor_left_pwm = (int)(thrust_pwm_min + 0.5*(0.5*(U1/thrust_pwm_constant - U4*10000/torque_pwm_constant)-U2/thrust_pwm_constant));
    
    getInBounds();
  }
  
  Serial.print(motor_front_pwm);  //11 -> front
  Serial.print(" ");
  Serial.print(motor_back_pwm);  //10 -> back
  Serial.print(" ");
  Serial.print(motor_right_pwm);  //9  -> right
  Serial.print(" ");
  Serial.println(motor_left_pwm);  //8  -> left
}

void sendDataMRF(int del_t)                                              //sends the IMU data and motor PWMs via 
{                                                               //Serial3(xbee) in MRF(MATLAB Readable Format)
  float_num roll,pitch,yaw;
  int_num m1,m2,m3,m4,delT;
  
  roll.val = q_Euler[2];
  pitch.val = q_Euler[1];
  yaw.val = q_Euler[0];
  
  m1.val = motor_front_pwm;
  m2.val = motor_back_pwm;
  m3.val = motor_left_pwm;
  m4.val = motor_right_pwm;
  delT.val = del_t;

  Serial.print('$');              //message start character
  
  for(int i=0;i<INT_SIZE;i++)    //sending delta time between two executions
    Serial.write(delT.inp[i]);
  
  for(int i=0;i<FLOAT_SIZE;i++)  //sending yaw
    Serial.write(yaw.inp[i]);
  
  for(int i=0;i<FLOAT_SIZE;i++)  //sending pitch
    Serial.write(pitch.inp[i]);
  
  for(int i=0;i<FLOAT_SIZE;i++)  //sending roll
    Serial.write(roll.inp[i]);
    
  for(int i=0;i<INT_SIZE;i++)    //sending front PWM
    Serial.write(m1.inp[i]);
    
  for(int i=0;i<INT_SIZE;i++)    //sending back PWM
    Serial.write(m2.inp[i]);
  
  for(int i=0;i<INT_SIZE;i++)    //sending left PWM
    Serial.write(m3.inp[i]);
    
  for(int i=0;i<INT_SIZE;i++)    //sending right PWM
    Serial.write(m4.inp[i]);
    
  Serial.println('|');
}

#endif
