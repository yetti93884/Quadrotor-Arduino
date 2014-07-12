#ifndef QUAD_H
#define QUAD_H  

#include <Arduino.h>
#include "quad_config.h"
#include <EEPROM.h>

/////////// Control Execution Constants  ///////////////////
boolean USER_OVERRIDE = false;    // turns true on emergency stop which disable controller to set pwms anymore.
boolean FLAG_SEND_DATA = false;   // turns on sending data in MRF format

////////////////////////////////////////////////////////////

/////////// Joystick variables /////////////////////////////


#define FLOAT_SIZE 4
#define INT_SIZE 4

union float_num
{
  char inp[FLOAT_SIZE];
  float val;
}u;
 
float command_yaw,command_pitch,command_roll,command_thrust;
 
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

float thrust_pwm_constant = 1.846/4;    // Thrust = 1.846*(pwm - 819.6)
float torque_pwm_constant = 1.858;  // scaled by 10**4
int thrust_pwm_min = (int)841.2;  //900 is good
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
float angularrates_filt[3] = {0.0, 0.0, 0.0};  //yaw_dot, pitch_dot, roll_dot
float angularrates_filt_prev[3] = {0.0, 0.0, 0.0};  //yaw_dot, pitch_dot, roll_dot

float pose_setpoints[3] = {0.0, 0.0, 0.0};  // yaw_set, pitch_set, roll_set
float position_setpoints[3] = {0.0, 0.0, 0.0};  //x,y,z
float quad_position[3];        //x,y,z
float velocities[3];

float speeds[4];

////////////////PD Controller implementation////////////////
float Kp_zhi;
float Kd_zhi;
float Ki_zhi;

float Kp_theta;
float Kd_theta;
float Ki_theta;

float Kp_phi;
float Kd_phi;
float Ki_phi;
float C_phi;

float e_zhi;
float e_zhi_prev;

float e_theta;
float e_theta_prev;

float e_phi;
float e_phi_prev;

float U1, U2, U3, U4;
float U2_i = 0;
float U3_i = 0;
float U4_i = 0;
//////////////////////////////////////////////////////////
/////////////Physical Parameters//////////////////////////
float m = 0.6;
float g = 9.8;

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
long in_num;                // argument to be passed along with the string
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
    motor_front.writeMicroseconds(motor_front_pwm);
    motor_right.writeMicroseconds(motor_right_pwm);
    motor_left.writeMicroseconds(motor_left_pwm);
    motor_back.writeMicroseconds(motor_back_pwm);
    
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

}

void throttleToMotor() {
  int pwm_val = 0;
  if(command_pitch>0 && command_pitch<1)
    pwm_val = MOTOR_PWM_MIN + (command_pitch)*(MOTOR_PWM_MAX-MOTOR_PWM_MIN);  
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
          Serial.println("EMERGENCY STOP FOR MOTORS");
          Serial3.println("EMERGENCY STOP FOR MOTORS");
          USER_OVERRIDE = true;
          FLAG_SEND_DATA = false;
          stopMotors();
        }
     else if(in_char == CONTROL_RESTART)
       {
         Serial.println("CONTROL RESTART FOR MOTORS");
          Serial3.println("CONTROL RESTART FOR MOTORS");
          USER_OVERRIDE = false;
          U2_i = 0;
          U3_i = 0;
          U4_i = 0;
          //updateMotors();
       }
   }
}

void parseMessage()
{
  boolean FLAG_STR_END = false;
  boolean FLAG_INP_ERROR = false;
  boolean FLAG_VALID_INP = false;
  boolean FLAG_SET_CONTROL_PARAM = false;

  int decimal_count = -1;
  int inp;
  
  delay(2);
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
        if(decimal_count>=0)
          decimal_count++;
      }
      else if (in_char == '.')
        decimal_count++;
      else
        FLAG_INP_ERROR = true;
    }
    delay(1);      //this delay is necessary for the proper functioning of Serial.
  }
  
  in_float = in_num/pow(10.0, decimal_count);
  decimal_count = -1;
  
  //////////////////////////////////////////////////
  
  ////////COMPARING WITH EXISTING COMMANDS//////////

  if(in_string.equals("left"))
  {
    FLAG_VALID_INP = true;
    USER_OVERRIDE = true;
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
  

  if(in_string.equals("right"))
  {
    FLAG_VALID_INP = true;
    USER_OVERRIDE = true;
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

  if(in_string.equals("front"))
  {
    FLAG_VALID_INP = true;
    USER_OVERRIDE = true;
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

  if(in_string.equals("back"))
  {
    FLAG_VALID_INP = true;
    USER_OVERRIDE = true;
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

  if(in_string.equals("all"))
  {
    FLAG_VALID_INP = true;
    USER_OVERRIDE = true;
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
  
  if (in_string.equals("kp_zhi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kp_zhi = in_float;
  }
  
  if (in_string.equals("kd_zhi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kd_zhi = in_float;
  }
  
  if (in_string.equals("ki_zhi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Ki_zhi = in_float;
  }
  
  if (in_string.equals("kp_theta")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kp_theta = in_float;
  }
  
  if (in_string.equals("kd_theta")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kd_theta = in_float;
  }
  
  if (in_string.equals("ki_theta")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Ki_theta = in_float;
  }
  
  if (in_string.equals("kp_phi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kp_phi = in_float;
  }
  
  if (in_string.equals("kd_phi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Kd_phi = in_float;
  }
  
  if (in_string.equals("ki_phi")) {
    FLAG_VALID_INP = true;
    FLAG_SET_CONTROL_PARAM = true;
    Ki_phi = in_float;
  }
  
  if (in_string.equals("reset")) {
    FLAG_VALID_INP = true;
    pose_setpoints[1] = 0;
    pose_setpoints[2] = 0;
    m = 0.6;
  }
  
  if(FLAG_SET_CONTROL_PARAM == true)
  {
    writeParamToEEPROM();
    showControlParams();
  }  
  if(in_string.equals("show"))
  {
    showControlParams();
    FLAG_VALID_INP = true;
  }
  
  if (in_string.equals("send_binary")) {
    Serial3.println("Sending data in Binary format");
    FLAG_VALID_INP = true;
    FLAG_SEND_DATA = true;
  }
  if (in_string.equals("stop_binary")) {
    FLAG_VALID_INP = true;
    FLAG_SEND_DATA = false;
  }
  if (in_string.equals("version")) {
    FLAG_VALID_INP = true;
    Serial3.print("Quadcopter IITK ");
    Serial3.println(VERSION);
  }
  /////////////////////////////////////////////////////
  
  ////////////CHECKING IF THE INPUT IS VALID///////////
  if(FLAG_VALID_INP == false)
  {
    Serial3.print(in_string);
    Serial3.print(" ");
    Serial3.println(in_num);
    Serial3.println("INVALID INPUT");
  }
  /////////////////////////////////////////////////////
  
  if(USER_OVERRIDE == true && FLAG_VALID_INP == true)
  {
    getInBounds();
    updateMotors();
  }
  
  ////////////PRINTING MOTOR STATES////////////////////
  Serial3.print(" Motor PWM : front- ");
  Serial3.print(motor_front_pwm);
  Serial3.print(" left- ");
  Serial3.print(motor_left_pwm);
  Serial3.print(" back- ");
  Serial3.print(motor_back_pwm);  
  Serial3.print(" right- ");
  Serial3.print(motor_right_pwm);  
  Serial3.println("  |  ");
  /////////////////////////////////////////////////////
  in_string = "";
  in_num = 0;
}

void parseJoyStickInput()
{ 
  float joystick[4] = { 0.0,0.0,0.0,0.0};
  char joystick_bla;

  int j_index = 0;
  int num_sign = 1;

  in_string = "";
  boolean FLAG_INPUT_PACKET_END = false;
  delay(4);
  int count = 0;
  while(count<20) {
      joystick_bla = Serial3.read();

      if (joystick_bla == ',') {
        joystick[j_index] *= num_sign;
        j_index = j_index + 1;
        num_sign = 1;
      }
      else if (joystick_bla == ':') {
        FLAG_INPUT_PACKET_END = true;
        j_index = 0;
      }
      else {
        if(joystick_bla>='0' && joystick_bla<='9')
          joystick[j_index] = joystick[j_index]*10 + joystick_bla - '0';
        else if(joystick_bla == '-')
          num_sign = -1;
       }
    
      count++;
  }
  if(FLAG_INPUT_PACKET_END == true)
  {
    command_yaw = joystick[0]/100.0;
        
    command_pitch = joystick[1]/100.0;
        
    command_roll = joystick[2]/100.0;
    
    command_thrust = joystick[3]/100.0;
    
    if(command_yaw <= 3.1 && command_yaw >= -3.1)      
      pose_setpoints[0] = (1-REF_FILTER_COEFF)*pose_setpoints[0] + REF_FILTER_COEFF*command_yaw;
    
    if(command_pitch <= 1 && command_pitch >= -1)      
      pose_setpoints[1] = (1-REF_FILTER_COEFF)*pose_setpoints[1] + REF_FILTER_COEFF*command_pitch;
      
    if(command_roll <= 1 && command_roll >= -1)
      pose_setpoints[2] = (1-REF_FILTER_COEFF)*pose_setpoints[2] + REF_FILTER_COEFF*command_roll;
    
    if(command_thrust <= 1.4 && command_thrust >= 0)
      m = (1-REF_FILTER_COEFF)*m + REF_FILTER_COEFF*command_thrust;
    
  }
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
  pose_dt = 0.02;
//  pose_dt = (millis() - last_pose_update)/1000.0;
//  last_pose_update = millis();
    
}

void printJoyStickInput()
{
  Serial.print("Joy Inputs: ");
  Serial.print(command_yaw,4);
  Serial.print(' ');
  Serial.print(command_pitch,4);
  Serial.print(' ');
  Serial.print(command_roll,4);
  Serial.print(' ');
  Serial.print(command_thrust,4);
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
  Serial.print(" Motor PWM : front- ");
  Serial.print(motor_front_pwm);
  Serial.print(" left- ");
  Serial.print(motor_left_pwm);
  Serial.print(" back- ");
  Serial.print(motor_back_pwm);
  Serial.print(" right- ");
  Serial.print(motor_right_pwm);  
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

void executePDController()
{
  float zhi_ref = pose_setpoints[0];
  float zhi = pose[0];
  
  float theta_ref = pose_setpoints[1];
  float theta = pose[1];
  
  float phi_ref = pose_setpoints[2];
  float phi = pose[2];
  
  /////////Computing U1 ////////////////////
  U1 = m*g/(cos(theta)*cos(phi));
  //////////////////////////////////////////
  
  ////////PID on zhi////////////////////////
  e_zhi = zhi_ref - zhi;
  float U4_p = Kp_zhi*e_zhi;
  float U4_d = Kd_zhi/Ts*(e_zhi - e_zhi_prev);
  e_zhi_prev = e_zhi;
  U4_i = U4_i + Ki_zhi*Ts*e_zhi;
  U4 = U4_p + U4_d + U4_i;
  //////////////////////////////////////////
  
  ////////PID on theta////////////////////////
  e_theta = theta_ref - theta;
  float U3_p = Kp_theta*e_theta;
  float U3_d = Kd_theta/Ts*(e_theta - e_theta_prev);
  e_theta_prev = e_theta;
  U3_i = U3_i + Ki_theta*Ts*e_theta;
  U3 = U3_p + U3_d + U3_i;
  //////////////////////////////////////////
  
  ////////PID on phi////////////////////////
  e_phi = phi_ref - phi;
  float U2_p = Kp_phi*e_phi;
  float U2_d = Kd_phi/Ts*(e_phi - e_phi_prev);
  e_phi_prev = e_phi;
  U2_i = U2_i + Ki_phi*Ts*e_phi;
  U2 = U2_p + U2_d + U2_i;
  //////////////////////////////////////////
  
  U1 = m*g;
//  U2 = 0;
//  U3 = 0;
//  U4 = 0;
}

void getPWM() {
  
  if (USER_OVERRIDE == false) {
    float omega2_front, omega2_left, omega2_back, omega2_right;
    omega2_front = U1/(4*b_thrust) + U4/(4*d) + U3/(2*b_thrust*l)*Ixx ;
    omega2_left  = U1/(4*b_thrust) - U4/(4*d) - U2/(2*b_thrust*l)*Ixx ;
    omega2_back  = U1/(4*b_thrust) + U4/(4*d) - U3/(2*b_thrust*l)*Ixx ;
    omega2_right = U1/(4*b_thrust) - U4/(4*d) + U2/(2*b_thrust*l)*Ixx ;
    
    motor_front_pwm = Kpwm*omega2_front + thrust_pwm_min;
    motor_left_pwm  = Kpwm*omega2_left  + thrust_pwm_min;
    motor_back_pwm  = Kpwm*omega2_back  + thrust_pwm_min;
    motor_right_pwm = Kpwm*omega2_right + thrust_pwm_min;
  }
}

void sendDataMRF(int del_t)                                              //sends the IMU data and motor PWMs via 
{                                                               //Serial(xbee) in MRF(MATLAB Readable Format)
  Serial3.print('$');              //message start character
  
  Serial3.print(del_t);    //sending delta time between two executions
  Serial3.print(',');
  
  Serial3.print(q_Euler[0],3);  //sending yaw
  Serial3.print(',');
  Serial3.print(q_Euler[1],3);  //sending pitch  
  Serial3.print(',');
  Serial3.print(q_Euler[2],3);    //sending roll
  Serial3.print(',');
  
//  Serial3.print(U1);
//  Serial3.print(',');
//  Serial3.print(U2);  
//  Serial3.print(',');
//  Serial3.print(U3);
//  Serial3.print(',');
//  Serial3.print(U4);
  Serial3.print(motor_front_pwm);    //sending front PWM
  Serial3.print(',');
  Serial3.print(motor_left_pwm);    //sending left PWM  
  Serial3.print(',');
  Serial3.print(motor_back_pwm);    //sending back PWM
  Serial3.print(',');
  Serial3.print(motor_right_pwm);    //sending right PWM
//    
  Serial3.println('|');
}
void showControlParams()
{
    Serial3.print("Gains(PD,N): ");
    Serial3.print(Kp_zhi,3);
    Serial3.print(" ");
    Serial3.print(Kd_zhi,3);
    Serial3.print(" ");
    Serial3.print(Ki_zhi,3);
    Serial3.print(" ");
    Serial3.print(Kp_theta,3);
    Serial3.print(" ");
    Serial3.print(Kd_theta,3);
    Serial3.print(" ");
    Serial3.print(Ki_theta,3);
    Serial3.print(" ");
    Serial3.print(Kp_phi,3);
    Serial3.print(" ");
    Serial3.print(Kd_phi,3);
    Serial3.print(" ");
    Serial3.print(Ki_phi,3);
    Serial3.println();
}

void readParamFromEEPROM()
{
  int address;
  float_num params[CONTROLLER_PARAM_COUNT];
  byte val;
  
  for(int i=0;i<CONTROLLER_PARAM_COUNT;i++)
  {
    for(int j=0;j<FLOAT_SIZE;j++)
     {
       address = CONTROLLER_PARAM_ADDRESS_START + i*FLOAT_SIZE + j;
       val = EEPROM.read(address);
       params[i].inp[j] = val;
     } 
  }
  
  Kp_zhi = params[0].val;
  Kd_zhi = params[1].val;
  Ki_zhi = params[2].val;
  Kp_theta = params[3].val;
  Kd_theta = params[4].val;
  Ki_theta = params[5].val;
  Kp_phi = params[6].val;
  Kd_phi = params[7].val;
  Ki_phi = params[8].val;
  
}

void writeParamToEEPROM()
{
  int address;
  float_num params[CONTROLLER_PARAM_COUNT];
  byte val;

  params[0].val = Kp_zhi;
  params[1].val = Kd_zhi;
  params[2].val = Ki_zhi;
  params[3].val = Kp_theta;
  params[4].val = Kd_theta;
  params[5].val = Ki_theta;
  params[6].val = Kp_phi;
  params[7].val = Kd_phi;
  params[8].val = Ki_phi;
  
  for(int i=0;i<CONTROLLER_PARAM_COUNT;i++)
  {
    for(int j=0;j<FLOAT_SIZE;j++)
     {
       address = CONTROLLER_PARAM_ADDRESS_START + i*FLOAT_SIZE + j;
       val = params[i].inp[j];
       EEPROM.write(address, val);
     } 
  }  
}

#endif
