#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <AsyncMqttClient.h>
#include <AsyncMqttClient.hpp>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


const char* ssid     = "D 2.4GHz";
const char* password = "dtd131001";

// Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192,168,0,230)
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "node1/temp"
#define MQTT_PUB_TIME "node1/time"

//MQTT Subscribe Topics
#define MQTT_SUB_LV1 "node1/ctrl"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */
#define Threshold 40
// Variables to hold sensor readings
float temp;
float level1, level2;

RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_LV1, 2);
  Serial.print("Subscribing at QoS 1, packetId: ");
  Serial.println(packetIdSub1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void callback(){
  //placeholder callback function
}


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedDate;
String dayStamp;
String timeStamp;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    // case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    // case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
   ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  //Setup interrupt on Touch Pad 3 (GPIO15)
  touchAttachInterrupt(T8, callback, Threshold);

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup();

  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  digitalWrite(21, HIGH);
  digitalWrite(22, HIGH);
  digitalWrite(23, HIGH);
  delay(1000);
  digitalWrite(21, LOW);
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);

  Serial.println();
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();

  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(25200);
}
  
  void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
 
  double a = 1.009249522e-03, b = 2.378405444e-04, c = 2.019202697e-07;
  double adc = 0;
  double Rt,temp;
  adc = analogRead(34);
  float voltage_out_value = (adc * 3.3 ) / (4095);
  Rt = 9800*((3.3 / voltage_out_value) - 1);
  temp = (1.0 / (a + b*log(Rt) + c*log(Rt)*log(Rt)*log(Rt))) - 273.2;
  Serial.printf("Nhiet do: %.2f \n", temp);
     
  if(temp <= level1)
      {Serial.printf("Bat den xanh \n");
      digitalWrite(21, HIGH);}
  else 
      {Serial.printf("Tat den xanh \n");
      digitalWrite(21, LOW);}

  if(temp > level1 && temp <= level2)
      {Serial.printf("Bat den vang \n");
      digitalWrite(22, HIGH);}
  else 
      {Serial.printf("Tat den vang \n"); 
      digitalWrite(22, LOW);}

  if(temp > level2)
      {Serial.printf("Bat den do \n");
      digitalWrite(23, HIGH);}
  else 
      {Serial.printf("Tat den do \n"); 
      digitalWrite(23, LOW);  }  
   
    // Publish an MQTT message on topic esp32/dht/temperature
    delay(3000);
    uint16_t packetIdPub1 = mqttClient.publish( MQTT_PUB_TIME, 2, true, String(formattedDate).c_str());
    delay(3000);
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_TEMP, 2, true, String(temp).c_str());
    delay(3000);  

    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    // Serial.printf("%.2f %.2f \n",level1,level2);

    
      
    delay(5000);
     //Go to sleep now
    Serial.println("Going to sleep now");
    esp_deep_sleep_start();
  }

}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message
  
  // Save the message in a variable
  String messageTemp;
 for (int i = 0; i < len; i++) {
  // Serial.print((char)payload[i]);
  messageTemp += (char)payload[i];
 }
 StaticJsonDocument <256> doc;
 deserializeJson(doc , messageTemp);
 float l1 = doc["level1"];
 float l2 = doc["level2"];
 Serial.printf("%.2f %.2f \n",l1,l2);
 level1 = l1;
 level2 = l2;
}

