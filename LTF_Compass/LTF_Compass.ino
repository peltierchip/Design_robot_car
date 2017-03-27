
#include <SoftwareSerial.h>
/// XBEE///
SoftwareSerial XBee(A8, A9); // RX, TX

///MOTOR///

///INPUT PROCESSING///

const unsigned int MAX_INPUT = 50;
char out_buff[64];
String inp;
String command;

///COMPASS///
#include <Wire.h>
#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from
unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

/// VARIABLES ///
int mode;
float currentbearing;
float targetbearing;

float GetBearings(){
  float angle;
  Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS11_ADDRESS, 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  float reading =float(angle16/10.0);
  float balanceno;

  if (abs(reading)>180){
    while(reading>180){
      balanceno = reading-360.0;
      reading=balanceno;
    while(reading<-180){
      balanceno = reading + 360.0;
      reading=balanceno;
    }
    }
  }else{
    balanceno = reading;
  }
  Serial.println(-1.0*balanceno);
  return -1.0*balanceno;
}

void Motor(char motor, char direct, int spd=0){
  if (motor == 'l'){
    if (direct == 'f'){
      digitalWrite(13, HIGH);
      digitalWrite(8, LOW);
      analogWrite(11, spd);
    }else if(direct == 'b'){
      digitalWrite(13, LOW);
      digitalWrite(8, LOW);
      analogWrite(11, spd);      
    }else if(direct == 's'){
      digitalWrite(8, HIGH);
    }
  }else if (motor == 'r'){
    if(direct=='f'){
      digitalWrite(12, HIGH);
      digitalWrite(9, LOW);
      analogWrite(3, spd);
    }else if(direct == 'b'){
      digitalWrite(12, LOW);
      digitalWrite(9, LOW);
      analogWrite(3, spd);      
    }else if(direct == 's'){
      digitalWrite(9, HIGH);
    }}
  }

void process_data (String data)
  {
    Serial.print(data);
if (data.startsWith("Bearing:"))
  {
    // interpret rest of the command
    int val;
    if (data.length() < 9)
    {
      // Assuming no other argumet in the cmd.. let's say we turn off the motors
     ; 
    }
    val = data.substring(8).toInt();
    while( val > 180){
      val -= 360;
    }
    // call function to deal with the motor output
    targetbearing=val;
    mode=1;
  } else if ( data.startsWith("Stop")){
     Motor('r','s');
     Motor('l','s');  
     mode=0;
  }}
  
void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte
      
      // terminator reached! process input_line here ...
      process_data (input_line);
      
      // reset buffer for next time
      input_pos = 0;  
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
  } // end of processIncomingByte  


String ret_str;
String readInput()
{
  ret_str = "";
  
  // check incoming serial data from XBee (and from Arduino IDE serial interface.. if connected)
  if (XBee.available() > 0) 
  {
    while (XBee.available() > 0) 
    {
      // append data to imput
      char byteIn = (char)XBee.read();
      // uncomment following line if want to see every byte received
      // //COMM_PORT.print(byteIn);

      // if received new_line or carriage_return chars, end input read and return the whole string
      if (byteIn == '\n' || byteIn == '\r')
      {
        // got my message
        ret_str = inp;
        
        // reinitialise string
        inp = "";
        return ret_str;
      } else {
        // append byte to inp String
        inp += byteIn;
      }
    }
  }

  // if here means no new_line char found yet.. return empty String
  return ret_str;
}

void setup() {
  // put your setup code here, to run once:
  XBee.begin(9600);
  Serial.begin(9600);
  Wire.begin();

  //Channel A
  pinMode(12, OUTPUT);
  pinMode(9, OUTPUT);
  
  //Channel B
  pinMode(13,OUTPUT);
  pinMode(8,OUTPUT);

  Motor('r','s');
  Motor('l','s');  

  mode=0;
}

void loop() {

  command = readInput();
  if (command.length() > 0)
  {
    // echo back received command
    Serial.print("echo: ");
    Serial.println(command);
    process_data(command);
  }

  if(mode==1){ // rotate to specified bearing
    currentbearing = GetBearings();
    if(currentbearing<targetbearing+7.5 && currentbearing>targetbearing-7.5){
     Motor('r','s');
     Motor('l','s');      
    }else{
    if (targetbearing - currentbearing <0){
     Motor('r','f',165);
     Motor('l','b',165);
    }else if (targetbearing - currentbearing >0){
     Motor('l','f',165);
     Motor('r','b',165); 
      }
    }
    Serial.println(targetbearing);
  }
  delay(50);
}



