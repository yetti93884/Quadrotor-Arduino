Quadrotor-Arduino
=================

1] quad_main_code v0.2 is compatible with v0.2 of Quadrotor-PC-cpp/joystick :
It can do two tasks:
	A] Xbee Serial can send joystick input to the arduino which then prints the joystick input and IMU data to the Serial0 port. The pitch of the joystick also controls 			the thrust for all of the motors in this mode
	B] Communicate with the Quad wirelessly using xbee commands. You can send commands through the xbee serial port(the one connected to the PC) such as : "%LEFT 1200". 			This will make the left motor move at 1200 PWM. Similarly if you give "%ALL 1500" all the motors will get 1500PWM. In this way all the four motors can be 			controlled individually and was particularly useful while testing on the thrust_testbench. NOTE: Inverted commas are not included and the motor designations 			are LEFT, RIGHT, FRONT and BACK. The % symbol is a precursor to all messages.

