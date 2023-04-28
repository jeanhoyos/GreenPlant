

// Import config file
#include "config.h"

// Import required libraries
//#include "WiFi.h"
//#include "ESPAsyncWebServer.h"
#include <ESP8266WiFi.h>
//#include "ESPAsyncWebServer.h"
//#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <SPI.h>
//#include <SD.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>


File myFile;


const int pump_pin = 4;
const int lamp_pin = 5;
const int lamp_pin_2 = 0;

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;


//----------------------MQTT--------------------------------------

// MQTT
WiFiClient espClient;
PubSubClient client(espClient);

const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASS;


//--------------------Fin-MQTT--------------------------------

#define DHTPIN 2     // Digital pin connected to the DHT sensor
float hum = 0.0 ;
float temp = 0.0 ; 
int moist = 0;

// Regulation objective
int moist_objective = 30;


// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);


// ------------ Timer

unsigned long startTime = 0; // Start time of the timer
unsigned long elapsedTime = 0; // Elapsed time since the timer started
unsigned long interval = 30000; // Interval of the timer in milliseconds  


void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  if(strcmp(topic,"nodered/water") == 0){
    Serial.print("Water using Pump");
    pump();
    }
  else if (strcmp(topic,"nodered/light") == 0){
    Serial.print("Light !!!! ");
    }
  
  Serial.println("-----------------------");
 
}



void readMoisture(){
  // Soil Moisture
  moist = map(analogRead(A0), 1023, 0, 0, 100);
  }


void readHum(){
  hum = dht.readHumidity();
  //return String(hum);
  }

void readTemp(){
  temp= dht.readTemperature();
  //return String(temp);
  }
  

void pump(){

  digitalWrite(pump_pin, HIGH);
  delay(5000);
  digitalWrite(pump_pin, LOW);  
  Serial.println("Pumping over");
  
  
  }


void light(){
  digitalWrite(lamp_pin, HIGH);  
  }

void light_2(bool output){
  digitalWrite(lamp_pin_2,output);
  }

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

  pinMode(pump_pin, OUTPUT);
  pinMode(lamp_pin, OUTPUT);
  pinMode(lamp_pin_2, OUTPUT);

  //delay(30000);
  //Serial.println("Waiting DHT.");
  dht.begin();

  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // turn on wifi ok light
  light();

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  Serial.println("Connecting ...");
  
  while (!client.connected()) {
      Serial.println("Connecting to MQTT...");
   
      if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
        Serial.println("connected"); 
        light_2(HIGH);
      } else {
   
        Serial.print("failed with state ");
        Serial.println(client.state());
        delay(2000);
   
      }
  }
  
  Serial.println("Connected ! ");

  // Subscription from the Raspberry 
  //client.subscribe("nodered/water");
  client.subscribe("nodered/light");  
   

    startTime = millis();

}


void reconnect(){
    light_2(LOW);
    while (!client.connected()) {
      Serial.println("Connecting to MQTT...");
      if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
        Serial.println("connected"); 
        light_2(HIGH);
      } else {
        Serial.print("failed with state ");
        Serial.println(client.state());
        delay(2000);
      }
    }}
    
void loop(){


  //--------Start timer------------

  /*
   * Send data only at each interval 
   */
   

   unsigned long timeLeft = interval - (millis() - startTime);
   //Serial.print("Next measure in ");
   //Serial.print(int(timeLeft/1000));
   //Serial.println(" seconds");

   if (millis() - startTime >= interval) {
      startTime = millis(); // Reset the start time
      elapsedTime += interval;
    
      //--------Get data------------
    
      readMoisture();
      Serial.print("Moisture = ");
      Serial.println(moist);
      readTemp();
      Serial.print("Temperature = ");
      Serial.println(temp);
      readHum();
      Serial.print("Humidity = ");
      Serial.println(hum);

      // ----------- Regulation for watering --------

      if (moist < moist_objective){
        Serial.println("Need to water plant");
        pump();
        }
    
    
      //--------MQTT------------
  
      
      String mqtt_message = String(moist) + "" + String(temp) + "" + String(hum);
      String mqtt_message_moist = String(moist);
      String mqtt_message_temp = String(temp);
      String mqtt_message_hum = String(hum);
    
      
      char mqtt_message_char[mqtt_message.length() +1];
      char mqtt_message_char_moist[mqtt_message_moist.length() +1];
      char mqtt_message_char_temp[mqtt_message_temp.length() +1];
      char mqtt_message_char_hum[mqtt_message_hum.length() +1];
    
      
      mqtt_message.toCharArray(mqtt_message_char, mqtt_message.length() +1);
      mqtt_message_moist.toCharArray(mqtt_message_char_moist, mqtt_message_moist.length() +1);
      mqtt_message_temp.toCharArray(mqtt_message_char_temp, mqtt_message_temp.length() +1);
      mqtt_message_hum.toCharArray(mqtt_message_char_hum, mqtt_message_hum.length() +1);
    
    
       if (!client.connected()){
          Serial.println("Not connected to MQTT broken ");
          Serial.println("Reconnecting ... ");
          reconnect();
       }
    
       Serial.println("Sending telemetries:");
       Serial.print("     - moist: ");
       if(client.publish("esp/moist", mqtt_message_char_moist )){
          Serial.println("success");
       }
       else{ 
          Serial.println(" failed");
       }
       Serial.print("     - temp: ");
       if(client.publish("esp/temp", mqtt_message_char_temp )){
          Serial.println(" success");
       }
       else{
          Serial.println("failed");
       }
       Serial.print("     - hum: ");
       if(client.publish("esp/hum", mqtt_message_char_hum )){
         Serial.println("  success");
       }
       else{
          Serial.println("  failed");
       }
   }

 /*
  * Publish heartbeat to keep connection alive
  */

  if(client.publish("esp/heartbeat", "" )){
    Serial.println(".");
    }
  else{
    Serial.println("Failed to send heartbeat");
    }
  
  client.loop();

  
//----------Fin-MQTT--------
  
  delay(3000);
  
}
