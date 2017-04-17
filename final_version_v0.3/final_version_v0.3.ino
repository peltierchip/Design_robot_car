#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
// One thing needs to be noticed: connect the left motor to myservo and right servo to myservo2
// for myservo, set the same direction as the real moving direction. For myservo2, gives the opposite direction

//GPS is not included in this version.

//For solar panel turning, this version is using pins: 24,25,26,27,28,29,30,31.

/// solar panel ///
const int sensorMin = 0;     // sensor minimum for rain sensor
const int sensorMax = 1024;  // sensor maximum for rain sensor

int motorPin1 = 24;
int motorPin2 = 25;
int motorPin3 = 26;
int motorPin4 = 27;
int motorPin5 = 28;
int motorPin6 = 29;
int motorPin7 = 30;
int motorPin8 = 31;

int delayTime = 10;
int s_mode = 0;         // drawer mode
int r_mode = 0;         // rain sensor mode
int current_position =0; //stepper position
int final_position =0;
int current_leg = 0; //servo position
int final_leg = 0;



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



//////////////////////////////////////////////////////////////



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
  
void setup() {
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

}
  
void loop() {
  //always read rain sensor reading first
  int sensorReading = analogRead(A10);
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);   ///0-flood     1-rain warning      2-not raining
  
  if (range==1 || range ==2){    //// raining
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
    Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="b"){   // if button "b" pressed ....
   Genotronex.println("Changed to control mode");
   mode = 'b';
   }
  if (String(BluetoothData)=="a"){// if button "a" pressed ....
   Genotronex.println("Changed to object following mode");
   mode = 'a';
  }
  if (String(BluetoothData)=="g"){// if button "g" pressed ....
   Genotronex.println("Changed to object following mode");
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
    Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="f"){   //forward
    //Genotronex.println("forward");
    Motor('r','b',250);
    Motor('l','f',250);
   }
  if(String(BluetoothData)=="h"){   //backward
    //Genotronex.println("backward");
    Motor('r','f',250);
    Motor('l','b',250);
   }
   if(String(BluetoothData)=="l"){   //left
    //Genotronex.println("left");
    Motor('r','b',255);
    Motor('l','b',255);

   }
   if(String(BluetoothData)=="r"){   //right
    //Genotronex.println("right");
    Motor('r','f',255);
    Motor('l','f',255);
    }

    if(String(BluetoothData)=="s"){   //right
    //Genotronex.println("stop");
    Motor('r','s');
    Motor('l','s');
    }
    if(String(BluetoothData)=="m"){   //// turn legs down 20 degree
    Serial.println(BluetoothData);
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
  while(digitalRead(switchbuttonf) == LOW){
    delay(100);
    Motor('r','b',250);
    Motor('l','f',250);
  }
  Motor('r','s');
    Motor('l','s');
    }
    
    if(String(BluetoothData)=="c"){   
  while(digitalRead(switchbuttonb) == LOW){
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
    if (final_position == 0){
      final_position == 180;
    }
    else if (final_position == 180){
      final_position == 0;
    }
    if (range==0){
      if (current_position == final_position){
     //////stop
     ///do nothing
  }else if (current_position > final_position){
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
  }

  }
  
  

}
 
    



