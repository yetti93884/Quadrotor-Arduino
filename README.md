Quadrotor-Arduino
=================

1]  v0.4 has PID control implemented on the PITCH axis. The PID values have been tuned
	according and the logs are available in Quadrotor-MATLAB->DATA->Ziglar Nichols tuning 
	day 2. PID tuning and data logging are working perfectly with Quadrotor-MATLAB v0.2
	A] ARDUIMU -> the arduimu code has been modified. a major bug was removed(regarding data
	 storage in EEPROM via ground_start). Code now has a DLPF of 5Hz and is in AIRSTART mode.
	
2] quad_main_code has some major changes in form of additional control code 
and xbee online parameter tuning. But it is recommended to read through the 
entire code as it is not STABLE.

3] ArduIMU startup time ~= 21s
4] Motors ARM time ~= 7s