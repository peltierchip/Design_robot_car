#include <SoftwareSerial.h>// import the serial library

SoftwareSerial Genotronex(A8, A9); // RX, TX    (A8-A15 can be used as RX and TX);
char BluetoothData; // the data given from Computer

void setup() {
  // put your setup code here, to run once:
  Genotronex.begin(9600);
  Genotronex.println("Bluetooth On please press 1 or 0 blink LED ..");
  Serial.begin(19200);
  Serial.println("Start checking infos");
}

void loop() {
  // put your main code here, to run repeatedly:
   if (Genotronex.available()){
    BluetoothData=Genotronex.read();
    Serial.println(String(BluetoothData));
   if(String(BluetoothData)=="1"){   // if number 1 pressed ....
   Genotronex.println("Number 1 received.");
   }
  if (String(BluetoothData)=="0"){// if number 0 pressed ....
   Genotronex.println("Number 0 received.");
  }
  if (String(BluetoothData)=="a"){// if number 0 pressed ....
   Genotronex.println("String a received");
  }
}
delay(100);// prepare for next data ...
}
