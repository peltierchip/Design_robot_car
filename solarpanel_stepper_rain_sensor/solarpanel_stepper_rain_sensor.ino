/* Stepper Copal
 * -------------
 *
 * Program to drive a stepper motor coming from a 5'25 disk drive
 * according to the documentation I found, this stepper: "[...] motor 
 * made by Copal Electronics, with 1.8 degrees per step and 96 ohms 
 * per winding, with center taps brought out to separate leads [...]"
 * [http://www.cs.uiowa.edu/~jones/step/example.html]
 *
 * It is a unipolar stepper motor with 5 wires:
 * 
 * - red: power connector, I have it at 5V and works fine
 * - orange and black: coil 1
 * - brown and yellow: coil 2
 *
 * (cleft) 2005 DojoDave for K3
 * http://www.0j0.org | http://arduino.berlios.de
 *
 * @author: David Cuartielles
 * @date: 20 Oct. 2005
 */
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum

int motorPin1 = 8;
int motorPin2 = 9;
int motorPin3 = 10;
int motorPin4 = 11;
int motorPin5 = 4;
int motorPin6 = 5;
int motorPin7 = 6;
int motorPin8 = 7;

int delayTime = 10;
int mode = 0;         // 0 --- clsoe   1---- open
int pre_mode = 0;
int current_position =0;
int final_position =0;

int switchstate=0;
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

void setup() {
   // initialize serial communication @ 9600 baud:
  Serial.begin(9600);  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
  pinMode(motorPin7, OUTPUT);
  pinMode(motorPin8, OUTPUT);
  pinMode(2, INPUT);
}

void loop() {
  pinMode(3,INPUT);
  switchstate = digitalRead(3);
  int sensorReading = analogRead(A0);
  // map the sensor range (four options):
  // ex: 'long int map(long int, long int, long int, long int, long int)'
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);   ///0-flood     1-rain warning      2-not raining
  if(range == 0){
    mode = 0;
  }
  else if (range==1){
    mode = 0;
  }
  else if(range == 2 && switchstate == HIGH){
    mode =1;
  }
  else{
    mode=0;
  }

    if (mode == 1){
      final_position = 130;///deploy the solar panel
    }
    else{
      final_position = 0;///collect the solar panel
    }
  if (current_position == final_position){
     //////stop
     ///do nothing
  }else if (current_position > final_position){
     ///back to 0
     current_position = current_position -1;
     clockwise();d
     anticlockwise2();
  }
  else if(current_position < final_position){
    ////go to 180
    current_position = current_position +1;
    anticlockwise();
    clockwise2();
  }
  
  Serial.println(mode);
  
}


