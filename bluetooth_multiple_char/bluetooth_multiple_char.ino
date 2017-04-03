//// need to add in an extra N to the end of number, to convert it into speed (int)

#include "SoftwareSerial.h";
String Data = "";

SoftwareSerial mySerial(A8, A9);  //RX,TX

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop()
{
  while (mySerial.available())
    {
        char character = mySerial.read(); // Receive a single character from the software serial port
        
        Data.concat(character); // Add the received character to the receive buffer
        if (character == 'N')
        {
            Serial.print("Received: ");
            Data.remove(Data.length()-1);
            Serial.println(Data);
            Data = Data.toInt();

//            if(Data == 200){
//              Serial.println("got it");
//            }

            // Add your code to parse the received line here....

            // Clear receive buffer so we're ready to receive the next line
            Data = "";
        }
    }
}
