#include <WiFi.h>
#include <PubSubClient.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "constant.h"


#define BOOT_BUTTON_PIN 0  // GPIO0 is usually the boot button
const int LED_BUILTIN = 2;

bool isWifiOk = false;

char buffPubPay[80];
char bufPubTopic[40];

// MQTT broker details
const char* mqtt_server = YOUR_MQTT_SERVER;  // You can use any public MQTT broker or your own
const int mqtt_port = YOUR_MQTT_PORT;
const char* mqtt_client_id = YOUR_MQTT_CLIENT_ID;  // If your broker requires authentication, set the password
const char* mqtt_user = YOUR_MQTT_USER;     // If your broker requires authentication, set the username
const char* mqtt_password = YOUR_MQTT_PWD;  // If your broker requires authentication, set the password

WiFiClient espClient;
PubSubClient mqttClient(espClient);


// MQTT topics
const char* setDateTopic = "1\0";//setDate
const char* setTimeTopic = "2\0";//setTime
const char* setDateTimeTopic = "3\0";//setDateTime

const char* setRelayTimeOnTopic = "4\0";//setRelayTimeOn
const char* setRelayDateTimerOnTopic = "5\0";//setRelayDateTimeOn

const char* setRelayEnableTopic = "6\0";   //relay/setEnable         //Chanenl SubChannel value => 0 0 1 => ch1 subCh1 Enable
const char* setRelayManualStateTopic = "7\0"; //relay/setManualState //Chanenl SubChannel value => 0 0 1 => ch1 subCh1 ON
const char* setRelayStartTopic = "8\0";  //relay/setStart            //Chanenl SubChannel value => 0 0 07:00:00 => ch1 subCh1 Start Time = 07:00:00
const char* setRelayEndTopic = "9\0";   //relay/setEnd               //Chanenl SubChannel value => 0 0 07:05:00 => ch1 subCh1 Start Time = 07:05:00
const char* setRelayDaysRepeatTopic = "10\0";//relay/setDaysRepeat    //Chanenl SubChannel value => 0 0 0:0:1:1:0:0:0 => ch1 subCh1 Repeat = sun=on,mon=no,tue=yes,wed=yes,thurs=on,fri=no,sat=no

const char* getRelayEnableTopic = "11\0";  //relay/getEnable          //Chanenl SubChannel value => 0 0 1 => ch1 subCh1 Enable
const char* getRelayManualStateTopic = "12\0"; //relay/getManualState //Chanenl SubChannel value => 0 0 1 => ch1 subCh1 ON
const char* getRelayStartTopic = "13\0"; //relay/getStart             //Chanenl SubChannel value => 0 0 07:00:00 => ch1 subCh1 Start Time = 07:00:00
const char* getRelayEndTopic = "14\0";  //relay/getEnd                //Chanenl SubChannel value => 0 0 07:05:00 => ch1 subCh1 Start Time = 07:05:00
const char* getRelayDaysRepeatTopic = "15\0"; //relay/getDaysRepeat   //Chanenl SubChannel value => 0 0 0:0:1:1:0:0:0 => ch1 subCh1 Repeat = sun=on,mon=no,tue=yes,wed=yes,thurs=on,fri=no,sat=no

const char* getRelayChannelInfoTopic = "16\0";  //relay/getRelayChannelInfo
const char* getRelayStateAllTopic = "17\0";//relay/getRelayStateAll //0:0:0:0:0

const char* setTimeScheduleTopic = "18\0"; //channel item Enable hh:mm outputState repeat(0:0:0:0:0:0:0) //0 0 1 12:13 1 0:0:0:0:0:0:0
const char* getTimeScheduleTopic = "19\0"; //channel item Enable hh:mm outputState repeat(0:0:0:0:0:0:0) //0 0 1 12:13 1 0:0:0:0:0:0:0

const char* getUpdateFwTopic = "20\0"; //FW_name FW_Ver(x.x.x) status //4rl_th 4.0.4 1
const char* setReqUpdateFwTopic = "21\0"; //FW_name FW_ver(x.x.x) URL  //4rl_th 4.0.4 fw/model5/4rl_th4.0.4.bin

const char* getTempTopic = "22\0"; //enable bind_output_ch lower_than upper_than //1 3 33.0 35.0
const char* setTempTopic = "23\0"; //enable bind_output_ch lower_than upper_than //1 3 33.0 35.0

const char* getHumiTopic = "24\0"; //enable bind_output_ch lower_than upper_than //1 3 33.0 35.0
const char* setHumiTopic = "25\0"; //enable bind_output_ch lower_than upper_than //1 3 33.0 35.0

const char* getDhtStateAllTopic = "26\0";//relay/getRelayStateAll //0:0:0:0:0

const char* getDeviceInfoTopic = "27\0";//Device_name Device_model FW_name FW_Ver(x.x.x)

const char* getDatetimeTopic = "28\0";//yyyy:MM:dd HH:mm:ss

const char* setReqManualControlTopic = "29\0";//channel state //1 1 //1 0
//MQTT

// Wi-Fi credentials
String ssid = YOUR_SSID_NAME;      //YOUR_SSID_NAME;
String password = YOUR_SSID_PWD;  //YOUR_SSID_PWD;


uint32_t chipId = 0;

void readRtc();
void ledBlink(int delay);
String readStringFromEEPROM(int address);
void writeStringToEEPROM(int address, const String& str);
int8_t readIntToEEPROM(int address);
void writeIntToEEPROM(int address, const int8_t value);
void pullBootHold10Sec();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void setDateAndTime(int year, int month, int day, int hour, int minute, int second);
void setTime(int hour, int minute, int second);
void setDate(int year, int month, int day);
void timeShedConfPublish();
void tempConfPublish();
void humiConfPublish();
void relayStateAllPublish();
void devInfoPublish();
// void envStateAllPublish();
void relayProcess();
void dhtPublish();
void appReqManualState();
void netpiePublish();



void setupMQTT(){
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  // Set MQTT keep-alive interval (in seconds)
  mqttClient.setKeepAlive(60);
  // Set MQTT socket timeout (in seconds)
  mqttClient.setSocketTimeout(10);

  
}


void setupResetFacPin() {
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // Set GPIO0 as input with internal pull-up resistor
}

void setupLed() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void setupWiFi() {
  if (ssid == "" || password == "") return;


  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int contWifi = 10;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if(contWifi>0) break;
    contWifi--;
  }

  if(WiFi.status() == WL_CONNECTED){
    isWifiOk = true;
  }
  

  if(isWifiOk){
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}


// Function to be called before reset
void beforeResetCallback() {
  Serial.println("ESP32 is about to reset!");
  // You can add any additional tasks or cleanup here
}


// Function to get and print the reason for the last reset
void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();

  Serial.print("Reset reason: ");
  switch (reason) {
    case ESP_RST_UNKNOWN: Serial.println("Unknown"); break;
    case ESP_RST_POWERON: Serial.println("Power-on"); break;
    case ESP_RST_EXT: Serial.println("External reset"); break;
    case ESP_RST_SW: Serial.println("Software reset"); break;
    case ESP_RST_PANIC: Serial.println("Software reset due to panic"); break;
    case ESP_RST_INT_WDT: Serial.println("Interrupt watchdog"); break;
    case ESP_RST_TASK_WDT: Serial.println("Task watchdog"); break;
    case ESP_RST_WDT: Serial.println("Other watchdogs"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep"); break;
    case ESP_RST_BROWNOUT: Serial.println("Brownout"); break;
    case ESP_RST_SDIO: Serial.println("SDIO reset"); break;
    default: Serial.println("Unknown reason"); break;
  }

  /**
  A "Brownout" reset on an ESP32 indicates that the supply voltage dropped below a certain threshold, 
  causing the microcontroller to reset. This is a protective measure to ensure that the system does not operate unpredictably due to insufficient voltage.
  */
}

void setup() {
  Serial.begin(115200);

  // Register the shutdown handler
  esp_register_shutdown_handler(beforeResetCallback);

  // Print the reason for the last reset
  printResetReason();

  setupResetFacPin();
  
  setupLed();
  
  setupWiFi();
  if(isWifiOk){
    setupMQTT();
  }
  
}


void loop() {
  ledBlink(500);

  if(isWifiOk){
    if (!mqttClient.connected()) {
      reconnect();
    }
    mqttClient.loop();
  }

  netpiePublish();
  // Small delay to prevent overwhelming the loop
  delay(100);
}

void netpiePublish(){
  static unsigned long lastMsg = 0;
  unsigned long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;


    if(isWifiOk){
      buffPubPay[80];
      memset(buffPubPay,0,sizeof(buffPubPay));
      // sprintf(buffPubPay,"%s",(const char*)"{ \"data\": { \"Temp\" : 24, \"Humi\" : 58 }}");
      sprintf(buffPubPay,"%s",(const char*)"{ \"data\": { \"humidity\" : 24, \"temperature\" : 58, \"place\" : \"NECTEC Thailand\" }}");

      bufPubTopic[40];
      memset(bufPubTopic,0,sizeof(bufPubTopic));
      sprintf(bufPubTopic,"%s",(const char*)"@shadow/data/update");
      // sprintf(bufPubTopic,"%s",(const char*)"@msg/test");
      mqttClient.publish(bufPubTopic, buffPubPay);
    }
  }
}


void ledBlink(int delay) {
  static unsigned long lastMsg = 0;
  unsigned long now = millis();
  if (now - lastMsg > delay) {
    lastMsg = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void pullBootHold10Sec() {
  static unsigned long buttonPressStart = 0;  // Variable to store the time when the button press started
  static bool buttonHeld = false;             // Variable to track if the button is being held

  int buttonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the boot button

  if (buttonState == LOW) {  // If the button is pressed
    if (!buttonHeld) {
      buttonPressStart = millis();  // Record the time when the button press started
      buttonHeld = true;
    } else if (millis() - buttonPressStart >= 6000) {  // If the button is held for 5 seconds
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);



      Serial.println("Button held for 10 seconds, resetting ESP32");
      ESP.restart();  // Reset the ESP32
    }
  } else {
    buttonHeld = false;  // Reset the button held state if the button is released
  }
}

void reconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outSensor", "hello world");
      // client.publish("outSensorDht11", "hello world");
      // client.publish("outRtc", "hello world");
      // ... and subscribe to a topic
      
      char bufTopic[80];
      mqttClient.subscribe("@shadow/data/update");
      delay(5);

      mqttClient.subscribe("@msg/test");
      delay(5);
      
      mqttClient.subscribe("@msg/led");
      delay(5);
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
