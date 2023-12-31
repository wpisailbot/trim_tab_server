#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include "HardwareSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pb_decode.h"
#include "messages.pb.c"

//for interrupt from Jetson
portMUX_TYPE interruptMutex = portMUX_INITIALIZER_UNLOCKED;
const int interruptPin = 35;
volatile bool interrupt = false;
volatile int i;

StaticJsonDocument<200> latestData;

const char* ssid = "sailbot_trimtab_ap";
const char* password = "sailbot123";

WebSocketsServer webSocket = WebSocketsServer(81);

void IRAM_ATTR requestData() {
  interrupt = true;
  i+=1;
}

void parseJson(char* jsonString) {
  DeserializationError error = deserializeJson(latestData, jsonString);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  Serial.println("Got json");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // Handle WebSocket events (e.g., new connections, messages)
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      parseJson((char*)payload);
      break;
    case WStype_BIN:
      Serial.println("Got bin. data");
      // DataMessage message = DataMessage_init_zero;
      // pb_istream_t stream = pb_istream_from_buffer(payload, length);
      // bool status = pb_decode(&stream, ControlMessage_fields, &message);
      // if (!status)
      // {
      //   Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
      //   return;
      // }
      // Serial.println("Received data");
      // latestData["battery_level"] = message.batteryLevel;
      // latestData["wind_angle"] = message.windAngle;
      break;
  }
}

void setup() {
  i=0;
  pinMode(interruptPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPin), requestData, RISING);
  Serial.begin(115200);
  Serial2.begin(115200);
  sleep(0.5);

  // Start the WiFi access point
  WiFi.softAP(ssid, password);
  Serial.println("WiFi Access Point started");

  // Start mDNS with the hostname 'esp32-websocket'
  if (!MDNS.begin("sailbot-trimtab-local")) {
    Serial.println("Error starting mDNS");
    return;
  }

  // Advertise the WebSocket server over mDNS
  MDNS.addService("sailbot-trimtab", "tcp", 81);
  Serial.println("mDNS service started");

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void loop() {
  webSocket.loop();
  String data = "testing";
  bool write = false;
  portENTER_CRITICAL(&interruptMutex);
  if(interrupt){
    write = true;
    interrupt = false;
  }
  portEXIT_CRITICAL(&interruptMutex);
  if(write){
    Serial.println("sending data");
    serializeJson(latestData, Serial2);
    Serial.println(); //necessary to mark end of message
  }
}