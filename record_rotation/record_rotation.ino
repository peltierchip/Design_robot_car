int len = 0;
int loops = 0;
float current_time;
float timechart[100];
int  footprint[100];
 #define outputA 6
 #define outputB 7
 int counter = 0; 
 int aState;
 int aLastState;  
 
void setup() {
  // put your setup code here, to run once:
    Serial.begin (9600);
    pinMode (outputA,INPUT);
    pinMode (outputB,INPUT); 
    aLastState = digitalRead(outputA);   
}

void loop() {
  // put your main code here, to run repeatedly:
    aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     
//     Serial.print("Position: ");
//    Serial.println(counter);
      current_time = millis();
      footprint[loops] = current_time;
       footprint[loops] = counter;
       Serial.print(current_time);
       Serial.print("\n");
       Serial.print(counter);
       Serial.print("\n");
       loops ++;
       aLastState = aState;
   } 
   

}
