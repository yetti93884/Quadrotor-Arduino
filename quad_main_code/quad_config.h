#ifndef QUAD_CONFIG_H
#define QUAD_CONFIG_H

#define MOTOR_LEFT_PIN 8
#define MOTOR_RIGHT_PIN 9
#define MOTOR_FRONT_PIN 11
#define MOTOR_BACK_PIN 10

#define MOTOR_PWM_MIN 800
#define MOTOR_PWM_MAX 2500


#define JOYSTICK_START '$'
#define JOYSTICK_END ':'

#define MESSAGE_START '%'
#define MESSAGE_END ':'

#define EMERGENCY_STOP '!'

/* set the settings for the IMU here*/
void initializeIMU();

/* set the BLDC motor initial configuration here */
void initizeMotors();

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

/* prints to Serial port 0 the PWM value passed to each of the BLDC motors*/
void printMotorPWM();

/* prints the IMU  roll pitch yaw angles as computed by the IMU*/
void printIMUReadings();

#endif

