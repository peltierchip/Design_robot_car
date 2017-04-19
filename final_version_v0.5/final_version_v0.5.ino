#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
// One thing needs to be noticed: connect the left motor to myservo and right servo to myservo2
// for myservo, set the same direction as the real moving direction. For myservo2, gives the opposite direction

//GPS is included in this version.

//For solar panel turning, this version is using pins: 24,25,26,27,28,29,30,31.

/// solar panel ///
String data;
const int sensorMin = 0;     // sensor minimum for rain sensor
const int sensorMax = 1024;  // sensor maximum for rain sensor
int range;
int motorPin1 = 24;
int motorPin2 = 25;
int motorPin3 = 26;
int motorPin4 = 27;
int motorPin5 = 32;
int motorPin6 = 33;
int motorPin7 = 34;
int motorPin8 = 35;
int delayTime = 10;
int s_mode = 0;         // drawer mode
int r_mode = 0;         // rain sensor mode
int current_position =0; //stepper position
int final_position=0;
int current_leg = 0; //servo position
int final_leg = 0;
String up_data;
    int t;
int ir1;
int ir2;
int ir3;


/// HC05///
SoftwareSerial Genotronex(A8, A9); // RX, TX    (A8-A15 can be used as RX and TX);
char BluetoothData; // the data given from Computer

/// Ultrasonic Sensor///
const int trigPin1 = 2;
const int echoPin1 = 4;
const int trigPin2 = 5;
const int echoPin2 = 6;
long duration, cm1, cm2;
char mode = 'b';          //mode a: straight line following     b: self control 
int motor_speed;
char left_direct;
char right_direct;
char direct;
int servo_status = 0;
int switchbuttonf = 40;
int switchbuttonb = 42;
Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position


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
float homelat1= 120.4492950439;


float homelong1=10357.8925781250;
float homelat2=120.4349975585;


float homelong2=10357.8935546875;
float homelat3=120.4328002929;


float homelong3=10357.9111328125;
float homelat4=120.4683990478;

float homelong4=10357.9423828125;
float homelat5=120.4345016479;
float homelong5=10357.9169921875;
float tarlat=homelat1;
float tarlong=homelong1;
float mylat;
float mylong;
float deltaHeading;
float WPHeading;
float deltaDist;
float followbearing;
float followerror;
float followlasterror;

/// VARIABLES ///
int g_mode;                  /// be careful here 
float currentbearing;
float targetbearing;
int confirm =0;
int travelling =0;
float lasterror = 0;
float kp = 2.50; // remain to be tested
float kd = 8.0  ; // remain to be tested
float ki = 0.0;
int i = 0;
int basespeed=220;
int offset =0;


//////////////////////////////////////////////////////////////

//// wifi upload ///
void upload(){
   String mylong1 = String(mylong,3);
   String mylat1 = String(mylat,3);
   String mybearing1= String(abs(currentbearing),2);
   if (mybearing1.length() == 5){
    mybearing1 = "0" + mybearing1;
   }
   if (mybearing1.length() == 4){
    mybearing1 = "00" + mybearing1;
   }
   if (range==2){
    data = "01"+mylong1+mylat1+mybearing1;
  }
   else{
     data = "11"+mylong1+mylat1+mybearing1;
   }
   data.remove(21,1);
   data.remove(14,1);
   data.remove(5,1);
   Serial3.println(data);
}
  



void ultra_reading(){
  
  pinMode(trigPin1, OUTPUT);
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  pinMode(echoPin1, INPUT);
  duration = pulseIn(echoPin1, HIGH);
  cm1 = microsecondsToCentimeters(duration);                       //define cm1 as left sensor reading, cm2 as right sensor reading

  pinMode(trigPin2, OUTPUT);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  pinMode(echoPin2, INPUT);
  duration = pulseIn(echoPin2, HIGH);
  cm2 = microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

//// stepper motor ///
void anticlockwise(){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(delayTime);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
}
void clockwise(){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(delayTime);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(delayTime);
}

void anticlockwise2(){
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, HIGH);
  delay(delayTime);
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, HIGH);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, HIGH);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  digitalWrite(motorPin5, HIGH);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  
}
void clockwise2(){
  digitalWrite(motorPin5, HIGH);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, HIGH);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, HIGH);
  digitalWrite(motorPin8, LOW);
  delay(delayTime);
  digitalWrite(motorPin5, LOW);
  digitalWrite(motorPin6, LOW);
  digitalWrite(motorPin7, LOW);
  digitalWrite(motorPin8, HIGH);
  delay(delayTime);
}
////

///// motor movement //////
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


  /////////  bearing ////////////
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


//////////////// GPS ////////////////
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
     //Serial.println(deltaDist);
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
    // Positive deltaHeading implies clockwise rotation about y-x plane
    // Negative deltaHeading implies counter-clockwise rotation about y-x plane
    // deltaHeading is the error that you want to minimise to zero
 
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////





  
void setup() {
/// gps setup
  Wire.begin();
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  Serial1.println(PMTK_Q_RELEASE);
  targetbearing =90;

// servo setup  
  myservo.attach(7);  // attaches the servo on pin 7 to the servo object
  myservo2.attach(10);
  // put your setup code here, to run once:
  
  Genotronex.begin(9600);
  Genotronex.println("Start listening from bluetooth");
  Serial.begin(19200);
  Serial.println("Start checking infos");
    //Channel A
  pinMode(12, OUTPUT);
  pinMode(9, OUTPUT);
  
  //Channel B
  pinMode(13,OUTPUT);
  pinMode(8,OUTPUT);

  //Solar panel steppers
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(motorPin8, OUTPUT);
  pinMode(2, INPUT);
  Motor('r','s');
  Motor('l','s');  
   pinMode(switchbuttonf, INPUT);      // sets the digital pin 13 as output
  pinMode(switchbuttonb, INPUT); 

  Serial3.begin(115200);
  pinMode(A11,INPUT);
  pinMode(A12,INPUT);
  pinMode(A13,INPUT);
}
  
void loop() {
  //always read rain sensor reading first
  int sensorReading = analogRead(A10);
  

  ir1 = analogRead(A11);
  ir2 = analogRead(A12);
  ir3 = analogRead(A13);
//  Serial.print("ir1: ");
//  Serial.println(ir1);
//  
//  Serial.print("ir2: ");
//  Serial.println(ir2);
//  
//  Serial.print("ir3: ");
//  Serial.println(ir3);
//  if(digitalRead(switchbuttonf) == LOW){
//    Serial.println("front switch botton low");
//  }
//  if(digitalRead(switchbuttonb) == LOW){
//    Serial.println("back switch botton low");
//  }
//  delay(1000);
  range = map(sensorReading, sensorMin, sensorMax, 0, 3);   ///0-flood     1-rain warning      2-not raining
  if (range==1 || range ==0){    //// raining
    Serial.println("raining");   
    final_position == 0;
    if (current_position == final_position){
     //////stop
     ///do nothing
  }
  else if (current_position > final_position){
     ///back to 0
     current_position = current_position -1;
     clockwise();
     anticlockwise2();
  }
  else if(current_position < final_position){
    ////go to 180
    current_position = current_position +1;
    anticlockwise();
    clockwise2();
  }
  }


  if (Genotronex.available()){
    BluetoothData=Genotronex.read();
   //Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="b"){   // if button "b" pressed ....
   Genotronex.println("Changed to control mode");
   mode = 'b';
   }
  if (String(BluetoothData)=="a"){// if button "a" pressed ....
   Genotronex.println("Changed to object following mode");
   mode = 'a';
  }
  if (String(BluetoothData)=="g"){// if button "g" pressed ....
   Genotronex.println("Changed to gps mode");
   mode = 'g';
  }
  }
  /////////////////mode a         object following mode////////////////
  if (mode=='a'){                  
    
    ultra_reading();

    int distance = min(cm1,cm2);
    if (cm1>200 && cm2 >200){
      ultra_reading();      // make sure that it is a constant reading but not unstable misreading
      if (cm1>200 && cm2 >200){
      // rotate in one direction to find the object
      Motor('r','b',200);
      Motor('l','f',0);
      }
      
    }
    else if (cm1>200 || cm2 >200){
      if(cm1>cm2){
        Motor('r','f',200);
      Motor('l','f',200);
      }    
      else{
        Motor('r','b',200);
      Motor('l','b',200);
      }
      Motor('r','f',motor_speed);
      Motor('l','b',motor_speed);
    }
    else if (cm1<200 &&cm2<200){
      distance = min(cm1,cm2);
      if (distance<30){
       motor_speed = min(10*(30-distance)+50,250);
      Motor('r','f',motor_speed);
      Motor('l','b',motor_speed);
      }
      else if(distance >40){
        motor_speed = min(6*(distance-30)+50,250);
        Motor('r','b',motor_speed);
      Motor('l','f',motor_speed);
      }
      else{
        Motor('r','s',0);
      Motor('l','s',0);
      }
    }
    else{
      Motor('r','s',0);
      Motor('l','s',0);
    }
    
    
  }

/////////////////    control mode ////////////////////////////
  
  else if (mode =='b'){           // control mode here
    //Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="f"){   //forward
    //Genotronex.println("forward");
      Motor('r','b',230);
    Motor('l','f',250);

   }
  if(String(BluetoothData)=="h"){   //backward
    //Genotronex.println("backward");
      Motor('r','f',230);
    Motor('l','b',250);
    
   }
   if(String(BluetoothData)=="l"){   //left
    //Genotronex.println("left");
    Motor('r','b',255);
    Motor('l','b',255);
//    delay(100);
//     Motor('r','s',255);
//    Motor('l','b',255);
//    delay(100);
   }
   if(String(BluetoothData)=="r"){   //right
    //Genotronex.println("right");
    Motor('r','f',255);
    Motor('l','f',255);
//    delay(500);
//    Motor('r','f',255);
//    Motor('l','s',255);
//    delay(500);
    }
       if(String(BluetoothData)=="d"){   //left shaking 
    //Genotronex.println("left");
    Motor('r','b',255);
    Motor('l','s');
    delay(400);
    Motor('r','s');
    delay(200);
//     Motor('r','s',255);
//    Motor('l','b',255);
//    delay(500);
   }
   if(String(BluetoothData)=="e"){   //right shaking
    //Genotronex.println("right");
    Motor('r','f',255);
    Motor('l','s');
    delay(400);
    Motor('r','s');
    delay(200);
//    Motor('r','f',255);
//    Motor('l','s',255);
//    delay(200);
    }

    if(String(BluetoothData)=="s"){   //right
    //Genotronex.println("stop");
    Motor('r','s');
    Motor('l','s');
    }
    if(String(BluetoothData)=="m"){   //// turn legs down 20 degree
    //Serial.println(BluetoothData);
    Serial.println(current_leg);
    Serial.println(final_leg);
    Motor('r','s');
    Motor('l','s');
    current_leg = final_leg;
    final_leg = final_leg+20;
    final_leg = min(180,final_leg);
    for (pos = current_leg; pos <= final_leg; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(160-pos);          // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    
  }
  delay(500);
    for (pos =current_leg ; pos <= final_leg; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);        // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    }
    delay(500);
  }
  if(String(BluetoothData)=="n"){   //// turn legs up 20 degree
    Serial.println(BluetoothData);
    Serial.println(current_leg);
    Serial.println(final_leg);
    Motor('r','s');
    Motor('l','s');
    current_leg = final_leg;
    final_leg = final_leg-20;
    final_leg = max(0,final_leg);
    for (pos = current_leg; pos >= final_leg; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(160-pos);          // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  delay(500);
    for (pos = current_leg; pos >= final_leg; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);        // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    }
    delay(500);
  }
  if(String(BluetoothData)=="x"){   //// turn legs down 10 degree
    Motor('r','s');
    Motor('l','s');
    current_leg = final_leg;
    final_leg = final_leg+ 10;
    for (pos = current_leg; pos <= final_leg; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(160-pos);          // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  delay(500);
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);        // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    }
    delay(500);
    BluetoothData = "s";
  }
  if(String(BluetoothData)=="y"){   //// turn legs up 10 degree
    Motor('r','s');
    Motor('l','s');
    current_leg = final_leg;
    final_leg = final_leg-10;
    for (pos = current_leg; pos <= final_leg; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(160-pos);          // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  delay(500);
    for (pos = 0; pos <= 180; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);        // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    }
    delay(500);
    BluetoothData = "s";
  }
///////////////control for servo only... not using this////////////////////////
//    if(String(BluetoothData)=='v'){
//      Motor('r','s');
//    Motor('l','s');
//      if(servo_status ==0){
//        for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);        // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//    }
//    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo2.write(180-pos);          // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//      }
//      else{
//        for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees   
//    myservo2.write(180-pos);   // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//    
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);     
//      // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//    
//  }
//      }
//    }
    if(String(BluetoothData)=="o"){
      Serial.println("opening");
      Motor('r','s');
      Motor('l','s');
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(160-pos);          // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  delay(500);
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);        // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
    }
    delay(500);
  while(digitalRead(switchbuttonb) == LOW){
    delay(100);
    Serial.println("forward");
    Motor('r','b',250);
    Motor('l','f',250);
  }
  Motor('r','s');
    Motor('l','s');
    delay(5000);
   while(ir2<100){
    delay(100);
    Motor('r','b',250);
    Motor('l','f',250);
   }
  while(digitalRead(switchbuttonf) == LOW){
    delay(100);
    Serial.println("b");
    Motor('r','f',250);
    Motor('l','b',250);
  }
  delay(200);
  Motor('r','s');
    Motor('l','s');
     for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(160-pos);     
      // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  delay(500);
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees   
    myservo2.write(pos);   // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  }


  
  if(String(BluetoothData)=="t" ){   //// turn solar panels
    
    Motor('r','s');
    Motor('l','s');
    if (current_position == 0){
      final_position = 160;
      }
    if (current_position == 160){
      final_position = 0;      
    }
    
    if (range==2){
      if (current_position == final_position){
     //////stop
     ///do nothing
  }else if (current_position > final_position){
     ///back to 0
     current_position = current_position -1;
     clockwise2();
     anticlockwise();
     if (current_position <12){       // over turn the solar panel
     clockwise2();
     anticlockwise();
     }
  }
  else if(current_position < final_position){
    ////go to 180
    current_position = current_position +1;
    anticlockwise2();
    clockwise();
  }
    }
  }

  }



  //////////////     GPS mode         ///////////////////
  else if (mode == 'g'){
    //firtly get current bearing
    currentbearing = GetBearings();
   Serial.print(String(BluetoothData));
    Serial.println(currentbearing);
    
//    Genotronex.println(currentbearing);
//    Genotronex.println("");
//    

    //then read gps
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
        Serial.println(mylat,10);
      Serial.println(mylong,10);
      Serial.println("gps position get");
//      XBee.println("Coordinates Updated!");
    }
    
  }
  upload();
  if(String(BluetoothData)=="z" ){ 
    // move to the bearing 
    if(currentbearing<targetbearing+10 && currentbearing>targetbearing-10){
     Motor('r','s');
     Motor('l','s');
    }
    else{
    if (targetbearing - currentbearing <-15){   /// needs to turn left
    Motor('r','b',255);
    Motor('l','s');
    delay(200);
    Motor('r','s');
    Motor('l','b',255);
    delay(200);
    }else if (targetbearing - currentbearing >15){
    Motor('r','f',255);
    Motor('l','s');
    delay(200);
    Motor('r','s');
    Motor('l','f',255);
    delay(200);
     }
     Serial.println("bearing mode");
  }
  }
  if(String(BluetoothData)=="d" ){                   // move to the point
    
    PathFinder(mylat,mylong);
    Serial.println(mylat);
    Serial.println(mylong);
    Serial.print("delta: ");
    Serial.println(deltaDist);
    if (deltaDist<200){
     Motor('r','s');
     Motor('l','s');
     Serial.println("Destination Reached!");
     Genotronex.println("reach");
//     mode='b';
//     Motor('r','s');
//     Motor('l','s');
     delay(5000);
     if (tarlat == homelat1){          // go to second point 
      tarlat = homelat2;
      tarlong = homelong2;
      delay(2000);
      Genotronex.println("set to target 2");
     }
     if (tarlat == homelat2){          // go to third point 
      tarlat = homelat3;
      tarlong = homelong3;
      delay(2000);
     }
     if (tarlat == homelat3){          // go to fourth point
      tarlat = homelat4;
      tarlong = homelong4;
      delay(2000);
     }
     if (tarlat == homelat4){          // go to fifth point
      tarlat = homelat5;
      tarlong = homelong5;
      delay(2000);
     }
     if (tarlat == homelat5){          // back to control mode, stop (need to control it or else it would be in jerky moving status, because the bluetoothdata is at "d" now, which is move to left in control mode.
     mode='b';
     Motor('r','s');
     Motor('l','s');
     delay(10000);
     }
    }
    else if(deltaHeading<15 && deltaHeading>-15){
     Motor('r','b',220);
     Motor('l','f',220);
     Serial.println("forward!");
    }
    else{
    if (deltaHeading<-10){
//      Motor('r','b',255);
//    Motor('l','s');
//    delay(400);
//    Motor('r','s');
//    delay(200);
       Motor('r','s',255);
    Motor('l','b',255);
    delay(200);
     Motor('r','b',255);
    Motor('l','s',255);
    delay(200);
    }
    else if (deltaHeading>10){
//    Motor('r','f',255);
//    Motor('l','s');
//    delay(400);
//    Motor('r','s');
//    delay(200);
    Motor('r','s',255);
    Motor('l','f',255);
    delay(200);
     Motor('r','f',255);
    Motor('l','s',255);
    delay(200);
    }
    }
     
//    i = i + deltaHeading;
//    
////  int motorspeed = kp*deltaHeading + kd *( deltaHeading - lasterror) + ki * i;
//   int motorspeed = kp*deltaHeading + kd *( deltaHeading - lasterror) ;
//   int leftspeed;
//   int rightspeed;
//   
//   Serial.println("destination mode");
//   Serial.println(motorspeed);
//   leftspeed = min( basespeed + motorspeed, 255);
//   rightspeed = min(basespeed - motorspeed,255);
//   lasterror = deltaHeading;
//   Motor('r','f',rightspeed);
//   Motor('l','f',leftspeed);
//  if (deltaDist<200){
//     Motor('r','s');
//     Motor('l','s');
//     //Serial.println("Destination Reached!");
////     steppermotor(1);
//     i=0;
//     lasterror=0;
//     mode=0;
  
  
  }
  if(String(BluetoothData)=="e" ){                   // calibrate gps
//    Genotronex.println("calibration starts");
    Serial.println("calibration starts");
     Motor('r','s');
     Motor('l','s');
     delay(1000);
    Motor('r','b',255);
    Motor('l','f',200);
    delay(200);
     Motor('r','s',255);
    Motor('l','b',255);
    delay(200);
     calibrateCMPS11();
     delay(5000);
     Motor('r','s');
     Motor('l','s');
//     Genotronex.println("calibration ends");
     Serial.println("calibration starts");
  }
  }

}





