#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
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
  cm1 = microsecondsToCentimeters(duration);

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
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object
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

  Motor('r','s');
  Motor('l','s');  

}
  
void loop() {
  if (Genotronex.available()){
    BluetoothData=Genotronex.read();
    Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="b"){   // if number 1 pressed ....
   Genotronex.println("Changed to control mode");
   mode = 'b';
   }
  if (String(BluetoothData)=="a"){// if number 0 pressed ....
   Genotronex.println("Changed to object following mode");
   mode = 'a';
  }
  }
  if (mode=='a'){
    
    ultra_reading();

    int distance = (cm1+cm2)/2;
    if (distance>30){
      
      motor_speed = 6*(distance-30)+50;
      motor_speed = min(motor_speed,255);
      Serial.println(motor_speed);
      Motor('r','b',motor_speed);
      Motor('l','f',motor_speed);
      
    }
    if (distance<30){
      motor_speed = 12*(30-distance)+50;
      char left_direct = 'b';
      char right_direct = 'f';
      motor_speed = min(motor_speed,255);
      Motor('r','f',motor_speed);
      Motor('l','b',motor_speed);
      Serial.println('b');
      Serial.println(motor_speed);
    }
    else{
      char left_direct = 's';
      char right_direct = 's';
    }
    
    
  }
  else if (mode =='b'){
    // control mode here
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

    if(String(BluetoothData)=='v'){
      Motor('r','s');
    Motor('l','s');
      if(servo_status ==0){
        for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);        // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(180-pos);          // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
      }
      else{
        for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees   
    myservo2.write(180-pos);   // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);     
      // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    
  }
      }
    }
  }
  

}
 
    



