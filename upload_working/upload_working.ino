// assume that only two status to upload
// for drone, 0 means no drone, 1 means drone is in there
// for rain, 0 means no rain, 1 means raining
int rain = 0;
String postData;

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
void set_up_code(){
  
}

void loop() {
set_up_code();
Serial.println("making POST request"); 

String contentType = "application/x-www-form-urlencoded";
String postData = "start=1&rain=0&positionx=237.26&positiony=158.03";
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


Serial.println("Wait five seconds");
delay(10000);

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
//}
//if ($start == 1){
//$myfile = fopen("host_data.txt", "w") or die("Unable to open file!");
//fwrite($myfile, $rain);
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



