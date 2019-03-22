Source code for receiving all the sensor data from LoRa Ra-01 Rx and publishing these data to MQTT protocol.
//connections between LoRA and ESP-12E NODE MCU 
//MISO-12 DIO0-D2 SCK-14 MOSI-13 RST-16 NSS-15 GND-GND 3V3-3.3V

// include libraries

#include <SPI.h>            // include SPI library
#include <LoRa.h>           // include LoRa library                     
#include <ESP8266WiFi.h>    // include ESP8266WiFi library
#include <PubSubClient.h>   // include PubSubClient library 
#include <bits/stdc++.h>    // include standard library

//define the pins used by the transceiver module

#define ss 15               //define the NSS pin to 15
#define rst 16              //define the Reset pin to 16                     
#define dio0 2              //define the DIO0 pin to 2 

// Update these with values suitable for your network

const char* ssid = "CDI_STAFF";             // Name of the Wi-Fi network          
const char* password =  "B5sPSFKb";         //Password of the Wi-Fi network 
const char* mqttServer = "172.16.73.4";     //Name of the mqtt server PP address
const int mqttPort = 1883;                  // default Port 1883
const char* mqttUser = "user";              // Set mqtt Usename
const char* mqttPassword = "pass";          // Set mqtt password
 
WiFiClient espClient;                      // Instantiate the client
PubSubClient client(espClient);

void setup() 
{

 //initialize Serial Monitor
 
  Serial.begin(115200);                            

  //******************** LoRa***********************
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  
  while (!LoRa.begin(433E6)) //Initialize the frequency with 433MHz
  {
    Serial.println(".");
    delay(500); 
  }
  
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  
  //************************** LoRa***********************
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Connecting to WiFi..");       // Connect to the network
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client",mqttUser,mqttPassword) ) 
    {
      Serial.println("connected");  
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }  
}


void loop() {

// try to parse packet
  
  int packetSize = LoRa.parsePacket();
  char data[10]="\0";
  if (packetSize) {
  
// received a packet
	
    Serial.print("Received packet '");
int i;

// read packet
 
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData);
	  
       client.publish("j/data",LoRaData.c_str());; // Publish the LoRa data 
    }
	
 // print RSSI of packet
	
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    client.loop();
  
  } 
  
}

void callback(char* topic, byte* payload, unsigned int length) // Messages arrive at the client this function is called
 {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}
