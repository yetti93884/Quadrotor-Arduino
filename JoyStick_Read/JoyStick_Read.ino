const int ledPin = 13; // the pin that the LED is attached to
char joystick[3][10];
char joystick_bla;

int num_commas_pos = 0;
int num_commas_joystick = 0;
int index1, index2;
int start = 0;
int q_index = 0;
int p_index = 0;

#define FLOAT_SIZE 4 

 union float_num
 {
   char inp[FLOAT_SIZE];
   float val;
 }u;
 
 float_num command_roll,command_pitch,command_thrust;

void setup() {
  // initialize serial communication:
//  Serial2.begin(57600);  // joystick
  Serial.begin(57600);  // lappy
  Serial3.begin(57600);  // xbee
}

void loop() {
  
  if (Serial3.available() > 0) {                              // IMU reading available
    parseJoyStickInput();
//    printJoyStickInput();
  }
}

void parseJoyStickInput()
{
  boolean FLAG_INPUT_PACKET_END = false;
  while (Serial3.available() > 0) {
      joystick_bla = Serial3.read();
      if (joystick_bla == ',') {
        q_index = q_index + 1;
        index1 = 0;
      }
      else if (joystick_bla == ':') {
        FLAG_INPUT_PACKET_END = true;
        printJoyStickInput();
        q_index = 0;
        index1 = 0;
      }
      else {
        if(joystick_bla>='A')
          joystick[q_index][index1] = joystick_bla-'A'+10;
        else if(joystick_bla>='0'&&joystick_bla<='9')
          joystick[q_index][index1] = joystick_bla-'0';
        else
          joystick[q_index][index1] = 0;
        index1++;
    }
  }
  if(FLAG_INPUT_PACKET_END == true)
  {
    for(int i=0;i<FLOAT_SIZE;i++)
        command_roll.inp[i] = joystick[0][2*i]*16 + joystick[0][2*i+1];
  
    for(int i=0;i<FLOAT_SIZE;i++)
        command_pitch.inp[i] = joystick[1][2*i]*16 + joystick[1][2*i+1];
    
    for(int i=0;i<FLOAT_SIZE;i++)
        command_thrust.inp[i] = joystick[2][2*i]*16 + joystick[2][2*i+1];
  }
}

void printJoyStickInput()
{
  Serial.print(command_roll.val,4);
  Serial.print(' ');
  Serial.print(command_pitch.val,4);
  Serial.print(' ');
  Serial.println(command_thrust.val,4);
}
