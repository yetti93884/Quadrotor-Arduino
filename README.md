Quadrotor-Arduino
=================

1]  v0.3 has two major additions which are STABLE:
	A] ESC_RANGE_SET -> this code is used for setting the throttle range 
	for the ESCs. In other words it is used to define the minimum and 
	the maximum PWM that the ESC will receive in order to set the 
	corresponding minimum and maximum of the BLDC motors. Read the code 
	before using it otherwise some wrong range can be set.
	B] Quad_ArduIMU -> this is the code running on the arduIMU to process 
	data from the accelero, gyro and magneto. The device sends data via a 
	serial port to the main controller Arduino MEGA.

2] quad_main_code has some major changes in form of additional control code 
and xbee online parameter tuning. But it is recommended to read through the 
entire code as it is not STABLE.
