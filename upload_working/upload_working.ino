
// tried out to use TX/RX communication between adafruit 8266 and arduino mega, not working. too much noise cannot get the useful signal.
// still need to use other pin connections to get the info from arduino 
// the general idea would be using analog read and analog write

String postData;
String inString = "";    // string to hold input
String data = "";   
String rain = "";
String drone = "";
String positionx = "";
String positiony = "";
String current = "";
String bearing = "";
int i;
int mode = 0;  /////means upload, 1 means download


#include <ESP8266WiFi.h>
#include <ArduinoHttpClient.h>
const char* ssid = "SUTD_GLAB";
const char* password = "Gl@b_120513";


int port = 80;
const char* host = "www.victomteng.net";

WiFiClient wifi;
HttpClient client = HttpClient(wifi, host, port);
int status = WL_IDLE_STATUS;


String response;
int statusCode = 0;

void setup() {
Serial.begin(115200);
delay(10);

// We start by connecting to a WiFi network

Serial.println();
Serial.println();
Serial.print("Connecting to ");
Serial.println(ssid);


/* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
would try to act as both a client and an access-point and could cause
network-issues with your other WiFi-devices on your WiFi-network. */
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
Serial.println("Signal Strenght : " + String(WiFi.RSSI()) + " dBm");

while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
}

Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());
}

int value = 0;
/// need to see how to write mega code. depends on wiring
void get_status_from_mega(){
  
}
void process(){
    Serial.println(data.length());////length is 21
    rain = data[0];
    drone = data[1];
    for (i = 2; i <8; i +=1){
        positionx+= data[i];
    }
    for (i = 8; i <16; i +=1){
        positiony+= data[i];
    }
    for (i = 16; i <21; i +=1){
        bearing+= data[i];
    }
    postData = "start=1&rain="+rain+"&drone="+drone+"&positionx="+positionx+"&positiony="+positiony+"&bearing="+bearing;
    Serial.print("POSTDATA: ");
    Serial.println(postData);
    mode = 0;
    positionx = "";
    positiony = "";
    bearing = "";
    rain = "";
    drone = "";

  
}
void upload(){
  Serial.println("making POST request"); 
  
  String contentType = "application/x-www-form-urlencoded";
  //String postData = "start=1&rain=0&drone=1&positionx=10.26&positiony=15033&bearing=172.36";
  //Serial.println(postData); 
  client.post("/host.php", contentType, postData);
  //client.get("/host.php");
  // read the status code and body of the response
  statusCode = client.responseStatusCode();
  response = client.responseBody(); // in response, drone first then rain (first yes means drone there.... 
  String info = String(response);
  
  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
  
  
//  Serial.println("Wait five seconds");
//  delay(10000);
}

void loop() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
//      Serial.print("Value:");
//      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      data = inString;
      // clear the string for new input:
      inString = "";
      process();
      Serial.print("POSTDATA: ");
      Serial.println(postData);
      upload();
    }
    
  }
} 



//////////////////////corresponding php file/////////////////
//<?php
//
//$start = 0;
//foreach ($_POST as $key => $value)
//{
//if ($key == "start") {
//$start = $value;
//}
//if ($key == "rain") {
//$rain = $value;
//
//}
//if ($key == "positionx") {
//$positionx= $value;
//
//}
//if ($key == "positiony") {
//$positiony = $value;
//}
//if ($key == "drone") {
//$drone = $value;
//}
//if ($key == "bearing"){
//$bearing = $value;
//}
//}
//if ($start == 1){
//$myfile = fopen("host_data.txt", "w") or die("Unable to open file!");
//fwrite($myfile, $rain);
//fwrite($myfile, "\n");
//fwrite($myfile, $drone);
//fwrite($myfile, "\n");
//fwrite($myfile, $bearing);
//fwrite($myfile, "\n");
//fwrite($myfile, $positionx);
//fwrite($myfile, "\n");
//fwrite($myfile, $positiony);
//fclose($myfile);
//}
//
//
//$myfile = fopen("host_data.txt", "r") or die("Unable to open file!");
//echo fread($myfile,filesize("webdictionary.txt"));
//fclose($myfile);




