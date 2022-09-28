#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
       #define DEBUG_ESP_PORT Serial
       #define NODEBUG_WEBSOCKETS
       #define NDEBUG
#endif 

#include <Arduino.h>
#ifdef ESP8266 
       #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   
       #include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "SinricProTemperaturesensor.h"


#define SEALEVELPRESSURE_HPA (1013.25)
#define HOSTNAME    "bedroom-ac"
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds
#define BAUD_RATE   9600

#define APP_KEY     ""
#define APP_SECRET  ""

#define TEMPERATURE_SENSOR_ID   ""
#define SWITCH_ID               ""

#define WIFI_SSID   ""
#define WIFI_PASS   ""

Adafruit_BME280 bme; // I2C
SoftwareSerial mySerial(D5, D7); // RX, TX

bool myPowerState = false;
unsigned long lastBtnPress = 0;
unsigned long startTime = millis();

bool deviceIsOn;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

void setupOTA() {
  // onStart contains SinricPro related stuff to disconnect automaticly from SinricPro when update starts!
  ArduinoOTA.onStart([]() { SinricPro.stop(); Serial.printf("Start updating %s\r\n", ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "spiffs"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(HOSTNAME);  
  ArduinoOTA.begin();
}

void setupWiFi() {
  WiFi.hostname(HOSTNAME);
  
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
  Serial.printf("[WiFi]: Hostname is \"%s\"\r\n", HOSTNAME);
}

bool onTemperatureSenosrPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s (via SinricPro) \r\n", deviceId.c_str(), state?"on":"off");
  return true; // request handled properly
}

bool onSwitchPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s (via SinricPro) \r\n", deviceId.c_str(), state?"on":"off");

  // ECON mode 
  // 216,36,53,141,241,0,78,141,67,163,94,30,82,219,89,69,72,164,94,57,251,249,78,159,191,163,94,134,245,120,2,7,249,39,22,141,178,128,10,15,1,47,30,149,186,136,18,23,9,55,38,157,194,144,26,31,17,63,46,165,202,152,34,39,25,71,54,173,210,120,1,6,25,35,20,135,171,168,52,8,41,86,17,183,171,120,1,5,249,38,17,138,219,120,1,5,246,35,17,135,174,171,4,8,42,70,17,138,171,120,49,56,249,86,68,135,219,120,1,5,246,35,65,135,174,120,1,5,246,35,17,135,171,120,4,8,249,39,52,135,174,120,1,53,41,38,68,186,171,168,1,5,246,35,17,135,171,120,4,5,41,83,17,138,174,168,1,5,246,35,17,135,174,171,1,5,246,35,17,135,171,120,1,56,246,35,17,135,171,171,1,5,246,35,17,135,171,120,4,5,246,38,65,138,171,120,49,5,246,35,17,183,222,120,54,250
 
  if(state) {
    uint8_t irArray[] = {227, 164,35,205,140,239,227,89,33,72,181,94,33,251,1,78,142,67,181,94,254,23,11,78,4,64,35,18,137,174,124,6,11,253,43,26,145,182,132,14,19,5,51,34,153,190,140,22,27,13,59,42,161,198,148,30,35,21,67,50,169,206,156,38,43,29,75,58,177,214,121,19,39,8,68,35,153,204,138,18,39,8,68,51,169,205,154,35,39,24,70,18,153,205,138,34,23,8,68,35,152,205,138,34,39,24,69,51,169,205,154,35,39,24,53,51,169,188,154,35,23,8,69,51,152,204,138,35,39,24,68,51,153,205,137,35,39,24,69,51,169,205,154,35,39,24,69,51,169,205,154,35,39,24,69,51,169,205,154,34,22,8,53,53,198};
    mySerial.write((uint8_t*)irArray, sizeof(irArray));
  } else {
    uint8_t irArray[] = {227, 164,35,205,140,237,226,89,33,72,181,94,32,251,1,78,142,67,180,94,253,23,10,78,4,64,35,18,137,174,124,6,11,253,43,26,145,182,132,14,19,5,51,34,153,190,140,22,27,13,59,42,161,198,148,30,35,21,67,50,169,206,156,38,43,29,75,58,177,214,121,19,39,8,68,35,153,204,138,18,39,8,68,51,169,205,154,35,39,24,70,18,153,205,138,34,23,8,68,35,152,205,138,34,39,24,69,51,169,205,154,35,39,24,69,51,169,188,154,35,23,8,69,51,152,204,138,35,39,24,68,51,153,205,137,35,39,24,69,51,169,205,154,35,39,24,69,51,169,205,154,35,39,24,69,51,169,205,154,34,22,8,69,53,223};
    mySerial.write((uint8_t*)irArray, sizeof(irArray));
  }

  Serial.println("Response: ");
  
  int len = 0;
  int c;  
  unsigned long timeout = 1000;
  unsigned long start = millis();
        
  while ((millis() - start < timeout)) {
    if (mySerial.available()) {
      c = mySerial.read();

      Serial.print(c, HEX); 
      Serial.print(",");
    }
    yield();
  }
   
  return true; // request handled properly
}
 
// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProSwitch &mySwitch = SinricPro[SWITCH_ID];
  mySwitch.onPowerState(onSwitchPowerState); 

  SinricProTemperaturesensor &mySensor = SinricPro[TEMPERATURE_SENSOR_ID];
  mySensor.onPowerState(onTemperatureSenosrPowerState);
  
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  
  // setup SinricPro
  SinricPro.begin(APP_KEY, APP_SECRET);
}

void setupTemperatureSensor() {
  if (!bme.begin(BME280_ADDRESS_ALTERNATE)) { 
    Serial.println("Could not find a valid BME280 sensor!");
    while (1)
      ;
  }
} 

void handleTemperaturesensor() {
  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  Serial.println("Temperature: " + String(bme.readTemperature(), 2) + " *C");
  Serial.println("Pressure: " + String(bme.readPressure() / 100.0F, 2) + " hPa");
  Serial.println("Altitude: " + String(bme.readAltitude(SEALEVELPRESSURE_HPA), 2) + " m");
  Serial.println("Humidity: " + String(bme.readHumidity(), 2) + " %");
  Serial.println("");
  
  temperature = bme.readTemperature();          // get actual temperature
  humidity = bme.readHumidity();                // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("BME reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature || humidity == lastHumidity) return; // if no values changed do nothing...

  SinricProTemperaturesensor &mySensor = SinricPro[TEMPERATURE_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event

  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}
 
void setup() {
  Wire.begin();   
  Serial.begin(BAUD_RATE);
  mySerial.begin(9600); // Start communicating with IR controller

  setupTemperatureSensor();
  setupWiFi();
  setupSinricPro();
  setupOTA();
}
 
void loop() {
  // put your main code here, to run repeatedly:
  handleTemperaturesensor();
  SinricPro.handle();
  ArduinoOTA.handle();
}
