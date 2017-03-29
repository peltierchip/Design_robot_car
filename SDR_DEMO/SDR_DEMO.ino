#include <SoftwareSerial.h>
#include <Wire.h>
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
    if (distance>40){
      
      motor_speed = 8*(distance-40)+50;
      motor_speed = min(motor_speed,255);
      Serial.println(motor_speed);
      Motor('r','b',motor_speed);
      Motor('l','f',motor_speed);
      
    }
    if (distance<30){
      motor_speed = 5*(30-distance)+50;
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
    Genotronex.println("forward");
    Motor('r','b',200);
    Motor('l','f',200);
   }
  if(String(BluetoothData)=="h"){   //backward
    Genotronex.println("backward");
    Motor('r','b',200);
    Motor('l','f',200);
   }
   if(String(BluetoothData)=="l"){   //left
    Genotronex.println("left");
    Motor('r','f',150);
    Motor('l','f',150);

   }
   if(String(BluetoothData)=="r"){   //right
    Genotronex.println("right");
    Motor('r','b',150);
    Motor('l','b',150);
    }

    if(String(BluetoothData)=="s"){   //right
    Genotronex.println("stop");
    Motor('r','s');
    Motor('l','s');
    }
   
  }
 
    



