// assume that only two status to upload
// for drone, 0 means no drone, 1 means drone is in there
// for rain, 0 means no rain, 1 means raining
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
  if (data[0]!='g'&& data[0]!='p'){
    //nothing happened
  }
  else if (data[0] == 'p' && data[1] == 'r' && data[3] == 'd'){
        rain = data[2];
    drone = data[4];
    for (i = 5; i <data.length(); i +=1){
      if (data[i] == 'x'){
        current = "x";
      }
      if (data[i] == 'y'){
        current = "y";
      }
      if (data[i] == 'b'){
        current = "b";
      }
      if (data[i] == '\n'){
        break;
      }
      
      
      if (current == "x"){
        positionx+= data[i];
      }
      else if (current == "y"){
        positiony+= data[i];
      }
      else if (current == "b"){
        bearing+= data[i];
      }
    }
    positionx.remove(0,1);
    positiony.remove(0,1);
    bearing.remove(0,1);
    postData = "start=1&rain="+rain+"&drone="+drone+"&positionx="+positionx+"&positiony="+positiony+"&bearing="+bearing;
    mode = 0;
    positionx = "";
    positiony = "";
    bearing = "";
    rain = "";
    drone = "";
  }
  else if (data[0] == 'g' && data[1] == 'r' && data[3] == 'd'){
    rain = data[2];
    drone = data[4];
    for (i = 5; i <data.length(); i +=1){
      if (data[i] == 'x'){
        current = "x";
      }
      if (data[i] == 'y'){
        current = "y";
      }
      if (data[i] == 'b'){
        current = "b";
      }
      if (data[i] == '\n'){
        break;
      }
      
      
      if (current == "x"){
        positionx+= data[i];
      }
      else if (current == "y"){
        positiony+= data[i];
      }
      else if (current == "b"){
        bearing+= data[i];
      }
    }
    positionx.remove(0,1);
    positiony.remove(0,1);
    bearing.remove(0,1);
    postData = "start=0&rain="+rain+"&drone="+drone+"&positionx="+positionx+"&positiony="+positiony+"&bearing="+bearing;
    mode = 1;
    positionx = "";
    positiony = "";
    bearing = "";
    rain = "";
    drone = "";
  }

  
}
void upload(){
  Serial.println("making POST request"); 
  
  String contentType = "application/x-www-form-urlencoded";
  String postData = "start=1&rain=0&drone=1&positionx=10.26&positiony=15033&bearing=172.36";
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

void download(){
  
}

void loop() {
  upload();
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
//      Serial.print("String: ");
//      Serial.println(inString);
        data = inString;
      // clear the string for new input:
      inString = "";
      process();
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




