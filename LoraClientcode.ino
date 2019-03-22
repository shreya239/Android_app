/* LoRa Code:
Source code for transmitting all the sensor data from LoRa Ra-01 Tx.
 */
//connections between LoRA and Arduino UNO 
//MISO-12 DIO0-D2 SCK-13 MOSI-11 RST-10 NSS-5 GND-GND 3V3-3.3V



// include libraries
#include <SPI.h>    // include SPI library
#include <LoRa.h>   // include LoRa library
#include<string.h>  // include String library

//define the pins used by the transceiver module
#define ss 5      //define the NSS pin to 5
#define rst 10   //define the Reset pin to 10
#define dio0 2   //define the DIO0 pin to 2 
SPIClass spi;

#define DHT11_PIN 0  //define analog port 0

char sensor_string[] = "Humidity: 00 Temperature: 00"; //Intilize the temperature and humidity value 

byte read_dht11_dat()
{
  byte i = 0;
  byte result = 0;
  for (i = 0; i < 8; i++)
  {
    while (!(PINC & _BV(DHT11_PIN))); // Wait forever until analog input port 0 is '1' 
    delayMicroseconds(30);
	
    if (PINC & _BV(DHT11_PIN))        //if analog input port 0 is still '1' after 30us
      result |= (1 << (7 - i));       // This position is 1
    while ((PINC & _BV(DHT11_PIN)));  //wait until '1' finish
  }
  return result;
}
int counter = 0;
//Initializing sensor pins
int sensor_pin = A1;
int waterPump = 7;              //moisture and water pump
int output_value ;
int pirPin = A2;               //pir sensor
int pirVal;
int sensorPin = A3;            //LDR sensor
unsigned int sensorValue = 0;
void setup() {
  DDRC |= _BV(DHT11_PIN);  //Let analog port 0 be output port
  PORTC |= _BV(DHT11_PIN) ; // Let initial value of port be 1  
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

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

  pinMode(waterPump, OUTPUT);  // Set the pin of waterpump as output
  pinMode(pirPin, INPUT);      // Set the pin of pir as input


}

void loop() {

  moisture(); // call function for moisture
  pir();      // call function for pir
  ldr();      // call function for ldr
 byte dht11_dat[5];
  byte dht11_in;
  char humstring[3];
  char tmpstring[3];
  char moisture[10];
  char pir[10];
  char lint[10];

  byte i;
  PORTC &= ~_BV(DHT11_PIN);        //1.pull-down I/O pin for 18ms
  delay(18);
  PORTC |= _BV(DHT11_PIN);         //2.pull-up I/O pin for 40us
  delayMicroseconds(40);
  DDRC &= ~_BV(DHT11_PIN);         //Let analg port 0 be input port
  delayMicroseconds(40);
  dht11_in = PINC & _BV(DHT11_PIN); //read only the input port 0
  delayMicroseconds(80);
  dht11_in = PINC & _BV(DHT11_PIN);
  delayMicroseconds(80);
  
  int x = 0;
  for (x = 0; x < 5; x++)
    dht11_dat[x] = read_dht11_dat();
  DDRC |= _BV(DHT11_PIN);          //Let analog port 0 be output port
  PORTC |= _BV(DHT11_PIN);         //Let te initial value of this port be '1'

  byte dht11_check_sum = dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]; //Check check_sum
  
  if (dht11_dat[4] != dht11_check_sum)
  {
    Serial.println("DHT11 checksum error");
  }

  Serial.println(sensor_string);
  itoa(dht11_dat[0], humstring, 10);
 itoa(dht11_dat[2], tmpstring, 10);      // int data type to string data type conversion
  itoa(output_value, moisture, 10);
  itoa(pirVal, pir, 10);
  itoa(sensorValue, lint, 10);
  sensor_string[10] = humstring[0];
  sensor_string[11] = humstring[1];
  sensor_string[26] = tmpstring[0];
  sensor_string[27] = tmpstring[1];
  char data[50] = "\0";
  strcat(data, humstring);
  strcat(data, " ");
  strcat(data, tmpstring);
  strcat(data, " ");
  strcat(data, moisture);            //append a copy of the source string to the     end of destination string
  strcat(data, " ");
  strcat(data, pir);
 strcat(data, " ");
 strcat(data, lint);
 Serial.println(data);
 Serial.println("Sending to SX1278_server"); 

 Serial.print("Sending packet:' ");
 Serial.println(counter);
  
 //Send LoRa packet to receiver
 LoRa.beginPacket();
  
  // LoRa.print("DHT11 Humidity and Temperature Sensor\n\n");
  LoRa.print(data);
  LoRa.endPacket();

  counter++;

  delay(1000);
}
 void (moisture)
 {
  output_value = analogRead(sensor_pin);
  Serial.println("Mositure : ");
  Serial.print(output_value);
  Serial.println("");
  if (output_value > 650)

{
    digitalWrite(waterPump, HIGH);
    Serial.println("Motor is ON and  water started pumping");
  }

  else
  {
    digitalWrite(waterPump, LOW);
    Serial.println("Motor is OFF and  water stopped pumping");

  }
  delay(1000);
 }

 void (pir)
 {
  int pirVal = analogRead(pirPin);
  Serial.println(pirVal);
  strcat(output_value, "  ");
  strcat(output_value, pirVal);
  if (pirVal == 0)
  {
    Serial.println(" no Motion Detected");
  }
  else
  {
    Serial.println(" Motion Detected");
  }
  
 }
 
 void (ldr)
 {
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  strcat(output_value, "  ");
  strcat(output_value, sensorValue);
  delay(100);
 }


