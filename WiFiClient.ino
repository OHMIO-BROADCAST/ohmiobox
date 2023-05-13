/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <WiFi.h>
#include "heltec.h" 
#include "images.h"


#include <Arduino.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>


// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "ohmioboxtest/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ohmioboxtest/sub"


#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
String rssi = "RSSI --";
String packSize = "--";
String packet ;

void logo(){
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}


void LoRaData(){
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 15 , "Received "+ packSize + " bytes");
  Heltec.display->drawStringMaxWidth(0 , 26 , 128, packet);
  Heltec.display->drawString(0, 0, rssi);  
  Heltec.display->display();
}




void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  LoRaData();
}

const char* ssid     = "HENAO_GOME";
const char* password = "10358732";

const char* host = "https://ohmiobroadcast.io";
const char* streamId   = "....................";
const char* privateKey = "....................";

//variables internas
bool NetworkStatus = false;
bool CloudStatus =false;
bool SerialStatus = false;

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");

  if(WiFi.status() == WL_CONNECTED){
    NetworkStatus=true;
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  if(client.connected()){
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("AWS IoT Connected!");
    CloudStatus=true;
  }

}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["time"] = millis();
  doc["Frecuencia (MHz)"] = 100;
  doc["MER (dB)"] = 0.5;
  doc["BER"] = 1.0;


  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  Serial.println("Enviando datos de Broadcast");
}

void initializeOLED(){
  //Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
   Heltec.display->init();
   Heltec.display->flipScreenVertically();  
  // logo();
  // delay(1500);
  // Heltec.display->clear();
     Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(0, 0, "OHMIO Box");
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 25, "Web 3.0 for");
  Heltec.display->drawString(0, 35, "Broadcast industry");
    Heltec.display->drawString(0, 50, "Version 1.0");
  Heltec.display->display();
   delay(2000);
}

void setup()
{
    initializeOLED();
    Serial.begin(115200);
    connectAWS();    
}

int value = 0;

void checkStatus(){
  if(WiFi.status() == WL_CONNECTED){
    NetworkStatus=true;
      if(client.connected()){
        CloudStatus=true;
      }else{
        CloudStatus=false;
        persistentAWS();
      }
  }else{
    NetworkStatus=false;
    CloudStatus=false;    
    persistentWifi();
  }
}

void persistentWifi()
{
  Serial.println("Persistent Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.status() != WL_CONNECTED){
      NetworkStatus=false;
  }
  if(WiFi.status() == WL_CONNECTED){
    NetworkStatus=true;
    persistentAWS();
  }
}


void persistentAWS()
{
  Serial.println("Persistent AWS");
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  // Create a message handler
  client.onMessage(messageHandler);
  if (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }
  if(!client.connected()){
    Serial.println("AWS not connected yet!");
    CloudStatus=false;
  }else{
    Serial.println("AWS connected");
    CloudStatus=true;
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  }
}

void showUI(){
  checkStatus();
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(15, 0, "CURRENT STATUS");
   if(NetworkStatus==false){
  Heltec.display->drawString(0, 10, "Network: Disconnected");
  }else{
      Heltec.display->drawString(0, 10, "Network: Connected");
  }
  if(CloudStatus==false){
  Heltec.display->drawString(0, 20, "Cloud: Disconnected");
  }else{
      Heltec.display->drawString(0, 20, "Cloud: Connected");
  }
  if(SerialStatus==false){
  Heltec.display->drawString(0, 30, "Serial: Disconnected");
  }else{
      Heltec.display->drawString(0, 30, "Serial: Connected");
  }
  Heltec.display->drawString(0, 40, "Command:");
  
  Heltec.display->display();
}

void loop()
{  

  showUI();
  publishMessage();
  client.loop();
  delay(5000);
    
}

