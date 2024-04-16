#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "HardwareSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define NUM_BALLAST_READINGS 100

// for interrupt from Jetson
portMUX_TYPE interruptMutex = portMUX_INITIALIZER_UNLOCKED;
const int interruptPin = 35;
volatile bool interrupt = false;

StaticJsonDocument<200> latestData;
StaticJsonDocument<200> ballastdoc;
StaticJsonDocument<1024> commandDoc;


const char *ssid = "sailbot_trimtab_ap";
const char *password = "sailbot123";

WebSocketsServer webSocket = WebSocketsServer(81);

Servo rudderServo;
Servo talonPWM;

const uint8_t ballastPotPin = 33;
const uint8_t talonPWMPin = 26;

unsigned long lastBallastCommandTime = millis();

void print_memory_usage() {
    // Get the total free memory
    size_t free_mem = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    
    // Get the minimum free memory observed since system boot
    size_t min_free_mem = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    
    // Print the memory usage
    Serial.print("Total free memory: ");
    Serial.println(free_mem);
}

void parseJson(char *jsonString)
{
  DeserializationError error = deserializeJson(latestData, jsonString);
  if (error)
  {
    // Serial.print(F("deserializeJson() failed: "));
    // Serial.println(error.f_str());
    return;
  }
  // Serial.println("Got json");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  // Handle WebSocket events (e.g., new connections, messages)
  switch (type)
  {
  case WStype_DISCONNECTED:
    // Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connection from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
  }
  break;
  case WStype_TEXT:
    // Serial.printf("[%u] Text: %s\n", num, payload);
    parseJson((char *)payload);
    break;
  case WStype_BIN:
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 18, 19);
  delay(0.5);

  pinMode(ballastPotPin, INPUT);
  talonPWM.attach(talonPWMPin);
  rudderServo.attach(27);

  // Start the WiFi access point
  WiFi.softAP(ssid, password);
  Serial.println("WiFi Access Point started");

  // Start mDNS with the hostname 'esp32-websocket'
  if (!MDNS.begin("sailbot-trimtab-local"))
  {
    //Serial.println("Error starting mDNS");
    return;
  }

  // Advertise the WebSocket server over mDNS
  MDNS.addService("_sailbot-trimtab", "_tcp", 81);
  Serial.println("mDNS service started");

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void broadcastToAllClients(String message)
{
  for (uint8_t i = 0; i < webSocket.connectedClients(); i++)
  {
    if (webSocket.remoteIP(i) != IPAddress(0, 0, 0, 0))
    {
      webSocket.sendTXT(i, message);
    }
  }
}
uint16_t ballastPositions[NUM_BALLAST_READINGS];
uint ballastPosIndex=0;
void loop()
{
  webSocket.loop();
  auto ballastPos = analogRead(ballastPotPin);
  ballastPositions[ballastPosIndex]=ballastPos;
  ballastPosIndex++;
  ballastPosIndex%=NUM_BALLAST_READINGS;

  if (Serial1.available())
  {
    //Serial.println("Got data");
    String jsonData = Serial1.readStringUntil('\n');

    if (jsonData.length() > 0)
    {
      DeserializationError error = deserializeJson(commandDoc, jsonData);
      if (error)
      {
      }
      else if (commandDoc.containsKey("requestData"))
      {
        String jsonString;
        serializeJson(latestData, jsonString);
        //Serial.println(jsonString);
      }
      else if (commandDoc.containsKey("get_ballast_pos")){
        uint ballastPos = 0;
        for(int i=0; i<NUM_BALLAST_READINGS; i++){
          ballastPos += ballastPositions[i];
        }
        ballastPos /= NUM_BALLAST_READINGS;
        //Serial.print("Sending ballast pos: ");
        //Serial.println(ballastPos);
        ballastdoc["ballast_pos"] = ballastPos;
        String jsonString;
        serializeJson(ballastdoc, jsonString);
        Serial1.println(jsonString);
      }
      else if (commandDoc.containsKey("rudder_angle"))
      {
        Serial.print("moving rudders: ");
        Serial.println(commandDoc["rudder_angle"].as<int16_t>()+90);
        rudderServo.write(commandDoc["rudder_angle"].as<int16_t>()+90);
      }
      else if (commandDoc.containsKey("ballast_pwm")){
        lastBallastCommandTime = millis();
        //Serial.print("Moving ballast: ");
        int16_t val = commandDoc["ballast_pwm"].as<int16_t>();
        //Serial.println(val);
        talonPWM.write(val);
      }
      else
      {
        broadcastToAllClients(jsonData);
        Serial.print("Sending command to remote:");
        Serial.println(jsonData);
      }
    }
    //print_memory_usage();
  }

  // Stop ballast if we haven't gotten a command in a while
  // This stops the ballast from falling off the boat if the Jetson crashes
  unsigned long currentTime = millis();
  if (currentTime-lastBallastCommandTime > 500){
    talonPWM.write(95);
  }
}