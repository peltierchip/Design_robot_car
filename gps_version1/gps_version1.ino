/////STEPPER MOTOR///
//
//#include <Stepper.h>
//
//
//#define STEPS_PER_MOTOR_REVOLUTION 32   
//#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048  
//int step_mode = 4;                               // 4 = full, 8 = half
//Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 41, 43, 45, 47,step_mode);
//int  Steps2Take;
//int motorstate=1; // 1->Up 0->Down
//
/////TEMPERATURE SENSOR///
//
//#include<dht.h>
//dht DHT;
//#define DHT11_PIN 31
//
/// XBEE///
#include <SoftwareSerial.h>
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

///GPS///
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
float homelat=120.44909667968750000000;
float homelong=10357.89355468750000000000;
float homelat2=120.43170166015625000000;
float homelong2=10357.91210937500000000000;
float homelat3=120.43969726562500000000;
float homelong3=10357.89257812500000000000;
float tarlat=120.50099945068359375000;
float tarlong=10357.88964843750000000000;
float mylat;
float mylong;
float deltaHeading=0.0;
float WPHeading;
float deltaDist;
float followbearing;
float followerror;
float followlasterror;

/// VARIABLES ///
int mode;
float currentbearing;
float targetbearing;
int confirm =0;
int travelling =0;
float lasterror = 0;
float kp = 2.50; // remain to be tested
float kd = 8.0  ; // remain to be tested
float ki = 0.0;
int i = 0;
int basespeed=150;
int offset =0;

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
//  XBee.println(-1.0*balanceno);
  return -1.0*balanceno;
}

void Motor(char motor, char direct, int spd=0){
  if (motor == 'l'){
    if (direct == 'b'){
      digitalWrite(13, HIGH);
      digitalWrite(8, LOW);
      analogWrite(11, spd);
    }else if(direct == 'f'){
      digitalWrite(13, LOW);
      digitalWrite(8, LOW);
      analogWrite(11, spd);      
    }else if(direct == 's'){
      digitalWrite(8, HIGH);
    }
  }else if (motor == 'r'){
    if(direct=='b'){
      digitalWrite(12, HIGH);
      digitalWrite(9, LOW);
      analogWrite(3, spd);
    }else if(direct == 'f'){
      digitalWrite(12, LOW);
      digitalWrite(9, LOW);
      analogWrite(3, spd);      
    }else if(direct == 's'){
      digitalWrite(9, HIGH);
    }}
  }

void process_data (String data)
  {
//    Serial.print(data);
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
  } else if(data.startsWith("T:")){
    int val;
    val = data.substring(2).toInt();
    while( val > 180){
      val -= 360;
      }
    followbearing=val;
    mode=3;
    }else if (data.startsWith("1h")){
    homelat = mylat;
    homelong = mylong;
    XBee.println("Ok!"); 
  } else if (data.startsWith("2h")){
    homelat2 = mylat;
    homelong2 = mylong;
    XBee.println("Ok!");
  }else if (data.startsWith("3h")){
    homelat3 = mylat;
    homelong3 = mylong;
    XBee.println("Ok!");
  } else if (data.startsWith("i")){
    XBee.print("Home1 Latitude: ");
    XBee.println(homelat,20);
    XBee.print("Home1 Longitude: ");
    XBee.println(homelong,20);
    XBee.print("Home2 Latitude: ");
    XBee.println(homelat2,20);
    XBee.print("Home2 Longitude: ");
    XBee.println(homelong2,20);
    XBee.print("Home3 Latitude: ");
    XBee.println(homelat3,20);
    XBee.print("Home3 Longitude: ");
    XBee.println(homelong3,20);
    XBee.print("Target Latitude: ");
    XBee.println(tarlat,20);
    XBee.print("Target Longitude: ");
    XBee.println(tarlong,20);
    XBee.print("Current Latitude: ");
    XBee.println(mylat,20);
    XBee.print("Current Longitude: ");
    XBee.println(mylong,20);

  } else if(data.startsWith("sla:")){
    tarlat= data.substring(20).toFloat();
    XBee.print("OK! ");
    XBee.print("Target Latitude : ");
    XBee.println(tarlat,20);
  }else if(data.startsWith("slo:")){
    tarlong= data.substring(20).toFloat();
    XBee.print("OK! ");
    XBee.print("Target Latitude : ");
    XBee.println(tarlong,20);
  }else if(data.startsWith("gt")){
   XBee.println("Confirm? (Y/N)");
   confirm=1; 
  }else if(data.startsWith("g1")){
   XBee.println("Home1. Confirm? (Y/N)");
   tarlat=homelat;
   tarlong=homelong;
   confirm=1; 
  }else if(data.startsWith("g2")){
   XBee.println("Home2. Confirm? (y/n)");
   tarlat=homelat2;
   tarlong=homelong2;
   confirm=1; 
  }
  else if(data.startsWith("g3")){
   XBee.println("Home3. Confirm? (y/n)");
   tarlat=homelat3;
   tarlong=homelong3;
   confirm=1; 
  }else if(data.startsWith("y")){
    if (confirm==1){
    XBee.println("Heading to Target Now!");
    PathFinder(mylat,mylong);
    targetbearing=WPHeading;
    XBee.print(" Initial WPHeading:"); XBee.print(WPHeading);
    travelling =1;
    mode=1;
    
    } else{
    XBee.println("Invalid input!");
    }
    confirm =0;
  }else if (data.startsWith("n")){
   XBee.println("Go process stopped!");
   confirm=0;  
//   }else if (data.startsWith("Su")){
//   steppermotor(1);
//   }else if (data.startsWith("Sd")){
//   steppermotor(0);
//   }
//   else if (data.startsWith("Stepmodeup")){
//   XBee.println("Stepper mode: Up");
//   motorstate=1;
//  }   else if (data.startsWith("Stepmodedown")){
//   XBee.println("Stepper mode: Down");
//   motorstate=0;
//  }else if (data.startsWith("t")){
//    int chk = DHT.read11(DHT11_PIN);
//    XBee.print("Humidity: " );
//    XBee.print(DHT.humidity, 1);
//    XBee.println("%");
//    XBee.print("Temparature: ");
//    XBee.print(DHT.temperature, 1);
//    XBee.println(" Deg. Cel.");
  }else if(data.startsWith("C")){
       XBee.println("Calibration Start!");
//  Motor('r','f',basespeed-20);
//  Motor('l','b',basespeed-20);  
    calibrateCMPS11();
     Motor('r','s');
     Motor('l','s');  
       XBee.println("Calibration Complete!");
  }else if(data.startsWith("pp")){
       XBee.println("Calibration Start!");
  Motor('r','f',basespeed-10);
//  Motor('l','b',basespeed-20);  
    calibrateCMPS11();
     Motor('r','s');
     Motor('l','s');  
       XBee.println("Calibration Complete!");
  
  }else if(data.startsWith("p")){
       XBee.println(currentbearing);
       }
    else if (data.startsWith("s")){
     Motor('r','s');
     Motor('l','s');  
     mode=0;
  }    else if (data.startsWith("f")){
     Motor('r','f',basespeed);
     Motor('l','f',basespeed);  
     mode=0;
  }    else if (data.startsWith("b")){
     Motor('r','b',basespeed);
     Motor('l','b',basespeed);  
     mode=0;
  }    else if (data.startsWith("l")){
     Motor('r','s',basespeed);
     Motor('l','f',basespeed); 
     mode=0;
  }    else if (data.startsWith("r")){
     Motor('r','f',basespeed);
     Motor('l','s',basespeed); 
     mode=0;
  }
      else if (data.startsWith("cw")){
     Motor('r','b',basespeed);
     Motor('l','f',basespeed); 
     mode=0;
  }
     else if (data.startsWith("ac")){
     Motor('r','f',basespeed);
     Motor('l','b',basespeed); 
     mode=0;
  }else if(data.startsWith("Bs:")){
    basespeed= data.substring(3).toFloat();
    XBee.print("Basespeed: ");
    XBee.println(basespeed);
  }}

//void steppermotor(int dir){
//  if(dir != motorstate){
//    if (dir==1){
//        Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION*0.25;  // Rotate CCW 1 turn  
//  small_stepper.setSpeed(500);  // 700 a good max speed??
//  XBee.println("Stepper is moving up!");
//  small_stepper.step(-Steps2Take);
//  motorstate =1;
//  XBee.println("Stepper has moved up!");
//    }else if (dir==0){
//  Steps2Take  =  STEPS_PER_OUTPUT_REVOLUTION*0.25 ;  // Rotate CW 1 turn
//  small_stepper.setSpeed(500);   
//    XBee.println("Stepper is moving down!");
//  small_stepper.step(Steps2Take);
//    motorstate =0;
//      XBee.println("Stepper has moved down!");
//    }
//  }else{
//    XBee.println("Can't Turn!");
//  }
//}
  
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
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);

  delay(1000);
  
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

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void PathFinder(float MyLat, float MyLon) {
  //Calculates the distance left to next waypoint
  //Updates the WPCounter if it is within 10m range
//  Serial.print("Current Latitude: "); Serial.println(MyLat,7);
//  Serial.print("Current longitude: "); Serial.println(MyLon,7);
  float deltaLat = tarlat - MyLat;
  float deltaLon = tarlong - MyLon;
  //Earth's circumference (40130170m) divided by 360 degrees gives 111195
 float deltaLatDist = deltaLat*111195;
 float deltaLonDist = deltaLon*111195;
 deltaDist = sqrt((deltaLatDist*deltaLatDist) + (deltaLonDist*deltaLonDist));
//  Serial.print("Latitudinal Distance to Waypoint is: "); Serial.println(deltaLat,7);
//  Serial.print("Longitudinal Distance to Waypoint is: "); Serial.println(deltaLon,7);
//  Serial.println(deltaLatDist);
//  Serial.println(deltaLonDist);
  XBee.print("Deg: "); Serial.println(deltaDist);
  // Calculate the required heading now that distance has been calculated
  WPHeading = (atan(deltaLonDist/deltaLatDist))*57.3;
  // The following set of If-Else Conditionals normalises the angles about y-axis
  if ((deltaLat < 0) && (deltaLon >= 0)) {
  WPHeading = WPHeading + 180;
  } else if ((deltaLat < 0) && (deltaLon < 0)) {
    WPHeading = WPHeading - 180;
  }
//  Serial.print("The heading to the waypoint is "); Serial.println(WPHeading);
  // Addition to find deltaHeading because MyHeading has opposite sign convention to WPHeading
  // If deltaHeading is POSITIVE, then Robot has to turn RIGHT
  // If deltaHeading is NEGATIVE, then Robot has to turn LEFT
  deltaHeading = WPHeading - currentbearing;
  if (deltaHeading>180){
    deltaHeading-=360;
  } else if (deltaHeading <-180){
    deltaHeading+=360;
  }
  XBee.print(" deltaHeading:"); XBee.print(deltaHeading);XBee.print(" Distance to Goal:"); XBee.println(deltaDist);
  // Positive deltaHeading implies clockwise rotation about y-x plane
  // Negative deltaHeading implies counter-clockwise rotation about y-x plane
  // deltaHeading is the error that you want to minimise to zero
 
}

void loop() {
  currentbearing = GetBearings();
  Serial.println(currentbearing);
  command = readInput();
  if (command.length() > 0)
  {
    // echo back received command
//    Serial.print("echo: ");
//    Serial.println(command);
    process_data(command);
  }

    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
//    if (False){
//       if (c) Serial.print(c);
//    }
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }  

if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer
    
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat,7);
//      Serial.print(", "); 
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon,7);
//      Serial.print("Location (in degrees, works with Google Maps): ");
//      Serial.print(GPS.latitudeDegrees, 4);
//      Serial.print(", "); 
//      Serial.println(GPS.longitudeDegrees, 4);
      
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      mylat = GPS.latitude;
      mylong = GPS.longitude;
//      XBee.println("Coordinates Updated!");
    }
  }

  if(mode==1){ // rotate to specified bearing
    
    if(currentbearing<targetbearing+10 && currentbearing>targetbearing-10){
     Motor('r','s');
     Motor('l','s');
     if(travelling==1){
      mode=2;
//      steppermotor(0);
      travelling=0;}else{
        mode=0;
      }
    }else{
    if (targetbearing - currentbearing <0){
     Motor('r','f',basespeed+20);
     Motor('l','s');
    }else if (targetbearing - currentbearing >0){
     Motor('l','f',basespeed+20);
     Motor('r','s'); 
      }
    }
    Serial.println(targetbearing);
  }
  else if(mode==2){
   PathFinder(mylat,mylong);
    i = i + deltaHeading;
//    int motorspeed = kp*deltaHeading + kd *( deltaHeading - lasterror) + ki * i;
   int motorspeed = kp*deltaHeading + kd *( deltaHeading - lasterror) ;
   int leftspeed;
   int rightspeed;
   leftspeed = min( basespeed + motorspeed, 255);
   rightspeed = min(basespeed - motorspeed,255);
   lasterror = deltaHeading;
   Motor('r','f',rightspeed);
   Motor('l','f',leftspeed);
  if (deltaDist<200){
     Motor('r','s');
     Motor('l','s');
     XBee.println("Destination Reached!");
//     steppermotor(1);
     i=0;
     lasterror=0;
     mode=0;
  }
  }
  else if(mode==3){
    XBee.println(currentbearing);
   followerror=currentbearing-followbearing;
   if (followerror>0){
    int testval = abs(followerror -360);
    if (abs(testval)<abs(followerror)){
      followerror=followerror-360;
    }} else if (followerror<0){
    int testval = abs(followerror +360);
        if (abs(testval)<abs(followerror)){
      followerror=followerror+360;      
    }
   }
   int motorspeed = kp*followerror + kd *( followerror - followlasterror) ;
   int leftspeed;
   int rightspeed;
   leftspeed = min( basespeed - motorspeed, 255);
   rightspeed = min(basespeed + motorspeed,255);
   followlasterror = followerror;
   Motor('r','f',rightspeed);
   Motor('l','f',leftspeed);
  }
  delay(25);
}

void calibrateCMPS11()
{

   Wire.beginTransmission(CMPS11_ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF0);                          //Calibration sequence byte 1
   Wire.endTransmission();
   delay(20);

   Wire.beginTransmission(CMPS11_ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF5);                          //Calibration sequence byte 2
   Wire.endTransmission();
   delay(20);

   Wire.beginTransmission(CMPS11_ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF7);                          //Calibration sequence byte 2
   Wire.endTransmission();
   delay(20);
   
   delay(30000);
   
   Wire.beginTransmission(CMPS11_ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF8);                          //Exit calibration mode
   Wire.endTransmission();
   delay(20);

}

