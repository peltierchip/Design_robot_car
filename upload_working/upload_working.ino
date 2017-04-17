// assume that only two status to upload
// for drone, 0 means no drone, 1 means drone is in there
// for rain, 0 means no rain, 1 means raining
int rain = 0;
int drone = 1;
String postData = "drone=1&rain=0";
// tried out to use TX/RX communication between adafruit 8266 and arduino mega, not working. too much noise cannot get the useful signal.
// still need to use other pin connections to get the info from arduino 
// the general idea would be using analog read and analog write

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
  if (rain == 0 and drone == 1){
    postData = "drone=1&rain=0";
  }
  else if (rain == 1 and drone == 1){
    postData = "drone=1&rain=1";
  }
  else if (rain == 1 and drone == 0){
    postData = "drone=0&rain=1";
  }
  else{
    postData = "drone=0&rain=0";
  }
}

void loop() {
set_up_code();
//Serial.println("making POST request"); 

String contentType = "application/x-www-form-urlencoded";
String postData = "drone=1&rain=1";
//Serial.println(postData); 
client.post("/host.php", contentType, postData);
//client.get("/host.php");
// read the status code and body of the response
statusCode = client.responseStatusCode();
response = client.responseBody(); // in response, drone first then rain (first yes means drone there.... 
String info = String(response);
info.remove(0,2);
info.remove(4);
Serial.println(info);
drone = info[0].toInt;
rain = info[2].toInt;
//Serial.print("Status code: ");
//Serial.println(statusCode);
//Serial.print("Response: ");
//Serial.println(response);


//Serial.println("Wait ten seconds");
delay(10000);

} 



//////////////////////corresponding php file/////////////////
//<?php
//
//foreach ($_POST as $key => $value)
//{
//if ($key == "drone") {
//$drone = $value;
//}
//if ($key == "rain") {
//$rain = $value;
//}
//}
//
//echo "$drone $rain\n";
//
//if ($drone < 1) {
//echo "No\n";
//} else {
//echo "Yes\n";
//}
//if ($rain < 1) {
//echo "No\n";
//} else {
//echo "Yes\n";
//}
//?>



