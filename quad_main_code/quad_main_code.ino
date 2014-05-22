#include <PinChangeInt.h>
//#include <Timer.h>

#include <Wire.h>

//Timer t;

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

//float [] quaternion = new float [4];
//float [] q_Euler = new float [3];
float quaternion[4] = {0.0,0.0,0.0,0.0};
float q_Euler[3] = {0.0,0.0,0.0};
//quaternion[0] = 0.0;
//quaternion[1] = 0.0;
//quaternion[2] = 0.0;
//quaternion[3] = 0.0;
//q_Euler[0] = 0.0;
//q_Euler[1] = 0.0;
//q_Euler[2] = 0.0;

float_num q0,q1,q2,q3;

///////////////////////////////////////////////////////////
 
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

void pulsout (int pin, int duration) {
digitalWrite(pin, HIGH);
delayMicroseconds(duration);
digitalWrite(pin, LOW);

}  

void parseJoyStickInput()
{
  boolean FLAG_INPUT_PACKET_END = false;
  while (Serial3.available() > 0) {
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
        if(joystick_bla>='A')
          joystick[j_index][index1] = joystick_bla-'A'+10;
        else if(joystick_bla>='0'&&joystick_bla<='9')
          joystick[j_index][index1] = joystick_bla-'0';
        else
          joystick[j_index][index1] = 0;
        index1++;
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
  Serial.print("  ");
  
}

void quaternionToEuler(float *q, float *euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void printIMUReadings()
{
  quaternion[0] = q0.val;
  quaternion[1] = q1.val;
  quaternion[2] = q2.val;
  quaternion[3] = q3.val;
//  q_Euler[0] = 0.0;
//  q_Euler[1] = 0.0;
//  q_Euler[2] = 0.0;

  quaternionToEuler(quaternion, q_Euler);
  Serial.print("Yaw: ");
  Serial.print(degrees(q_Euler[0]),4);
  Serial.print(' ');
  Serial.print("Pitch: ");
  Serial.print(degrees(q_Euler[1]),4);
  Serial.print(' ');
  Serial.print("Roll: ");
  Serial.println(degrees(q_Euler[2]),4);

//  Serial.print(q0.val,4);
//  Serial.print(' ');
//  Serial.print(q1.val,4);
//  Serial.print(' ');
//  Serial.print(q2.val,4);
//  Serial.print(' ');
//  Serial.println(q3.val,4);
}

void setup() {
  Serial.begin(57600);  // lappy
  Serial2.begin(57600);  // imu
  Serial3.begin(57600);  // xbee
  loop_start = 0;
  Wire.begin();
  delay(5);
  delay(5);
  
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
    
//  t.every(20, motor);
//    pinMode(2,OUTPUT);
//    for(int x=4; x<8; x++) {pinMode(x, OUTPUT); }    
//    for(int x=4; x<8; x++) {pulsout(x, 2500); }
  
  #ifdef DEBUG_RECEIVER
//  Serial.begin(38400);
  #endif
  
  delay(200);

//  rc_setup_interrupts();
}

void motor() {
  
  for(int x=4; x<8; x++) {pulsout(x, speeds[x-4]);}
  
}  


void loop() {
  
  if (Serial3.available() > 0) {                              // Joystick reading available
    parseJoyStickInput();
  }
  
  if (Serial2.available() > 0) {                              // Xbee reading available
    parseIMUInput();

  }
  printJoyStickInput();
  printIMUReadings();
//  analogWrite(2,168);
//  //loop_start, = millis();
//  rc_process_channels();
//  
//  sixDOF.getYawPitchRoll(angles);
//  sixDOF.getValues(rates);
//  
//  if(rc_values[3] < 1500){fly = 0;}
//  else{fly = 1;}
//  /*
//  if (Serial.available() > 0 ){             
//    char m = Serial.read();
//        if(m == ' '){zhuman = 800;  Serial.println("HALT"); rollzero = roll; pitchzero = pitch; iroll = 0; ipitch = 0;}
//        if(m == 'w'){zhuman = zhuman + 25;  Serial.println(zhuman); }
//        if(m == 's'){zhuman = zhuman - 25;  Serial.println(zhuman); }
//        if(m == 'n'){pitch_set = pitch_set + 10; }//Serial.print("Roll Set = "); Serial.println(pitch_set); }
//        if(m == 'm'){pitch_set = pitch_set - 10; }//Serial.print("Roll Set = "); Serial.println(pitch_set); }
//
//    
//  }
//  */
// 
//  zhuman = map(rc_values[1],1000,1850,750,1600);
//  
//  pitch_in = mapfloat((float)rc_values[2],1050.00,1950.00,-25.00,25.00)  ;
//  roll_in =  mapfloat((float)rc_values[0],1050.00,1950.00,-25.00,25.00)  ;
//  
//  pitch_set = pitch_in - pitch_set_zero;
//  roll_set = roll_in - roll_set_zero;
//  
//  if(pitch_set < 1 && pitch_set > -1){pitch_set = 0;}
//  if(roll_set < 1 && roll_set > -1){roll_set = 0;}
//  
//  roll_get = angles[2] ; 
//  pitch_get = angles[1] ;
//  
//  roll = roll_get - rollzero;
//  pitch = pitch_get - pitchzero;
//   
//  gyroX = rates[3]; 
//  gyroY = (-rates[4]);
//  gyroZ = rates[5];
// 
// // iroll = iroll + (roll/1000);
// // ipitch = ipitch + (pitch/1000);
// 
// //if(roll_set < 1.00 && roll_set > -1.00){ roll_set  = 0;}
//  
//  if(fly == 1){
//    if(zhuman > 1000){
//      speeds[0] = zhuman - (roll-roll_set )*kr - gyroX*dr + 1.5*gyroZ; //- iroll*ir ;
//      speeds[1] = zhuman + (roll-roll_set )*kr + gyroX*dr + 1.5*gyroZ ; //+ iroll*ir ;
//      speeds[2] = zhuman - (pitch-pitch_set )*kr - gyroY*dr - 1.5*gyroZ; // - ipitch*ir ;
//      speeds[3] = zhuman + (pitch-pitch_set )*kr + gyroY*dr - 1.5*gyroZ ; //+ ipitch*ir ;
//      //speeds[0] = 800;
//      //speeds[1] = 800;
//    }
//    else{
//      speeds[0] = zhuman; 
//      speeds[1] = zhuman;
//      speeds[2] = zhuman;
//      speeds[3] = zhuman;
//      //speeds[0] = 800;
//      //speeds[1] = 800;
//    }
//      
//  }
//  else{
//  speeds[0] = 800; 
//  speeds[1] = 800;
//  speeds[2] = 800;
//  speeds[3] = 800;
//  rollzero = roll_get; pitchzero = pitch_get; pitch_set_zero = pitch_in; roll_set_zero = roll_in;
//  }
// /* 
//     #ifdef DEBUG_RECEIVER
//      if (millis() % 100 == 0)
//     {
//      rc_print_channels();
//      }
//     #endif
//  */   
//  
// t.update();


/* Serial.print(roll);
 Serial.print("  ");
 Serial.print(roll_set);
 Serial.print("  ");
 Serial.println(millis());
*/

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min)*(out_max - out_min)/(in_max - in_min)  + out_min; 
}
