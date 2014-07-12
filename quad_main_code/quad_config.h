#ifndef QUAD_CONFIG_H
#define QUAD_CONFIG_H

#define MOTOR_LEFT_PIN 8  //clockwise
#define MOTOR_RIGHT_PIN 9  //clockwise
#define MOTOR_FRONT_PIN 11  //anti-clockwise
#define MOTOR_BACK_PIN 10  //anti-clockwise

#define MOTOR_PWM_MIN 900
#define MOTOR_ARM_PWM 800
#define MOTOR_PWM_MAX 1100

#define CONTROLLER_PARAM_ADDRESS_START 150
#define CONTROLLER_PARAM_COUNT 9

#define JOYSTICK_START '$'
#define JOYSTICK_END ':'

#define MESSAGE_START '%'
#define MESSAGE_END ':'

#define EMERGENCY_STOP '!'
#define CONTROL_RESTART '?'

#define VERSION "v0.3"

#define REF_FILTER_COEFF 0.08

#define pi 3.142
/////////////Experimental Parameters //////////////////////
#define Kpwm_rpm 3.896e-5
#define Kpwm Kpwm_rpm/((2*pi/60)*(2*pi/60))
#define Cpwm = 842.4
#define d 6.715e-7
#define b_thrust 1.61e-5
#define l 0.455/2
#define Ixx 0.043675
#define bzz 0.0628
///////////////////////////////////////////////////////////

#define Ts 0.02

/* set the settings for the IMU here*/
void initializeIMU();

/* set the BLDC motor initial configuration here */
void initizeMotors();

/* Bounds PWM in the [PWM_MIN, PWM_MAX] range */
void getInBounds();

/* updates the PWM to each of the BLDC motors*/
void updateMotors();

/* STOPs the action of all the motors by setting them to minimum PWM*/
void stopMotor();

/* transforms the throttle value to motor PWM for all motors*/
void throttleToMotor();

/* takes input from Serial3 port and classifies it as either Joystick
or Message input*/
void parseSerialInput();

/* A message can be taken from the user in the form of "FL 1500" for 
sending a particular message and an argument from Serial3. This 
function reads the messaae and takes an action*/
void parseMessage();

/* takes the joystick input on Serial3 and writes it to command_roll,
command_pitch, command_thrust*/
void parseJoyStickInput();

/*takes thr IMU input and saves it in the IMU variable*/
void parseIMUInput();

/*prints the joystick input through thr serial port 0*/
void printJoyStickInput();

/* converts from the quarternion to euler angle notation*/
void quaternionToEuler(float *q, float *euler);

/* returns conjugate of a quaternion */
void quatConjugate(float *quat, float *result);

/* returns product of quaternions */
void quatProd(float * a, float * b, float *result);

/* prints to Serial port 0 the PWM value passed to each of the BLDC motors*/
void printMotorPWM();

/* prints the IMU  roll pitch yaw angles as computed by the IMU*/
void printIMUReadings();

/* Updates parameters used by control algo */
void updateControlParams();

///* Evaluates the control inputs U1, U2, U3, U4 */
//void executeController();
//
///* Evaluates PWM signal based on U1, U2, U3, U4 */
//void getPWM();

/*sends the IMU data and motor PWMs via Serial3(xbee) in MRF(MATLAB Readable Format)*/
void sendDataMRF();  

/*shows the control parameters on Serial Port3*/
void showControlParams();

/*writes the current Control Parameters to EEPROM*/
void writeParamToEEPROM();

/*reads the parameters from EEPROM to global variables*/
void readParamFromEEPROM();

/*computes the inputs u1,u2,u3,u4 according to a PID controller*/
void executePDcontroller();

/*translates the control inputs to PWM values*/
void getPWM();

#endif

