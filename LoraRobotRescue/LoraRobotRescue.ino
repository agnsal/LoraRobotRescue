
/*
Copyright 2019 Agnese Salutari.
Licensed under the Apache License, Version 2.0 (the "License"); 
you may not use this file except in compliance with the License. 
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on 
an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
See the License for the specific language governing permissions and limitations under the License
*/

// LoRaWan dependencies:
#include <MKRWAN.h>
#include "arduino_secrets.h"

// GPS dependencies:
#include <TinyGPS.h>

// Pins:
#define ledPin 2  // PWM output (LED)
#define buzzerPin 3  // PWM output (Buzzer)
#define tempPin 18  // Analog input (Temperature Sensor)
#define voltPin 19  // Analog input (Voltage Sensor)

// Other needed Serials:
# define gpsSerial Serial1 // rx and tx pins (microUsb is Serial, or Serial0)

// Commands:
#define stopCmd "stopAll"
#define stopUsbSendingCmd "stopUsb"
#define stopLoraSendingCmd "stopLora"
#define startUsbSendingCmd "startUsb"
#define startLoraSendingCmd "startLora"
#define noisyCmd "noisy"
#define newDelayCmd "nD"
#define newLoraDelayCmd "nLD"
#define newMiddleTempCmd "nMT"
#define newRadiusTempCmd "nRT"
#define newMinVoltCmd "nMV"
#define newMiddleLatCmd "nMY"
#define newRadiusLatCmd "nRY"
#define newMiddleLonCmd "nMX"
#define newRadiusLonCmd "nRX"

// LoRaWan manager:
LoRaModem modem;
// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);

// GPS manager:
TinyGPS gps; // create gps object

// Configurations:
float middleTemp = 20, radiusTemp = 20;  // Celsius
float middleLat = 42.094079, radiusLat = 0.100000; 
float middleLon = 13.3714029, radiusLon = 0.100000; 
uint8_t minVolt = 0;  // Volts
uint8_t lookAtMeLevel = 5;
uint16_t cycleDelayTime = 600;  // Seconds/10
uint8_t loraDelayTime = 2;  // Minutes
int actualDelayTime = 0; // Milliseconds
uint8_t loraSendAttempts = 2;  // Times
float R1 = 7501, R2 = 3002;  // Voltage Sensor R values
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

// State Variables:
float currentTemp = middleTemp;
float currentLat = 0;
float currentLon = 0;
float currentVolt = 0;
String lastInMsg = "";
unsigned long loraNextTime = loraDelayTime * 60000 + millis();  // Milliseconds
boolean stopUsbSending = false;
boolean stopLoraSending = false;
float lat = 0;  // Latitude
float lon = 0;  // Longitude

typedef struct {
    uint8_t alarms;
    uint16_t temp;
    uint16_t volt;
    uint32_t lat;
    uint32_t lon;
  } msgLoraT;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(tempPin, INPUT);
  pinMode(voltPin, INPUT);

  
  Serial.begin(9600);
  while (!Serial);
  gpsSerial.begin(9600); // connect gps sensor
  while(!gpsSerial);
  
  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");  // Test
    while (1) {}
  };

  // TEST
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  loraConnect();

  // Set poll interval to 60 secs.
  modem.minPollInterval(60);
  // NOTE: independently by this setting the modem will
  // not allow to send more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.
}

void loop() {
  checkUsbInMsg();
  checkLoraInMsg();  // The input from the user (via Lora) has a major priority than the robot one, so it eventually overwrites it. 
  char msg[] = "0000";  // The state list (string): "noisy, tempAlarm, minVoltAlarm, posAlarm"; Ok = "0,0,0,0"
  if(!lastInMsg.startsWith(stopCmd)) {
    updateCurrentTemp();
    updateCurrentVolt();
    updateCurrentLatAndLon();
    actualDelayTime = cycleDelayTime * 100;  // Ok msg.
    manageAlarms(msg);
    manageCommands(msg);
    if(!stopUsbSending) {
      sendUsbMsg(msg);
    }
    if (!stopLoraSending && millis() >= loraNextTime) {
      uint8_t attempt = 1;
      boolean loraSent = sendLoraMsg(msg);
      while(loraSent && attempt < loraSendAttempts){
        loraSent = sendLoraMsg(msg);
        attempt ++;
      }
      if(loraSent) {
        updateLoraNextTime();
      }     
    }
  }
  smartDelay(actualDelayTime);
}

void manageCommands(char msg[]) {
  if(lastInMsg.startsWith(noisyCmd)) {
    lookAtMe();
    msg[0] = '1';
    actualDelayTime = 0;
  }
  else if(lastInMsg.startsWith(stopUsbSendingCmd)) {
    stopUsbSending = true;
    actualDelayTime = 0;
  }
  else if(lastInMsg.startsWith(startUsbSendingCmd)) {
    stopUsbSending = false;
    actualDelayTime = 0;
  }
  else if(lastInMsg.startsWith(stopLoraSendingCmd)) {
    stopLoraSending = true;
    actualDelayTime = 0;
  }
  else if(lastInMsg.startsWith(startLoraSendingCmd)) {
    stopLoraSending = false;
    actualDelayTime = 0;
  }
  else if(lastInMsg.startsWith(newDelayCmd)) {
    setCycleDelayTime(lastInMsg.substring(2).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newLoraDelayCmd)) {
    setLoraDelayTime(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newMiddleTempCmd)) {
    setMiddleTemp(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newRadiusTempCmd)) {
    setRadiusTemp(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newMiddleLatCmd)) {
    setMiddleLat(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newRadiusLatCmd)) {
    setRadiusLat(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newMiddleLonCmd)) {
    setMiddleLon(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newRadiusLatCmd)) {
    setRadiusLon(lastInMsg.substring(3).toFloat());
    actualDelayTime = 0;
    lastInMsg = "";
  }
  else if(lastInMsg.startsWith(newMinVoltCmd)) {
    setMinVolt(lastInMsg.substring(3).toInt());
    actualDelayTime = 0;
    lastInMsg = "";
  }
}

void manageAlarms(char msg[]) {
  if(currentTemp < middleTemp - radiusTemp) {
    msg[1] = '1';
    actualDelayTime = cycleDelayTime * 10;
  }
  if(currentTemp > middleTemp + radiusTemp) {
    msg[1] = '2';
    actualDelayTime = cycleDelayTime * 10;
  }
  if(currentVolt < minVolt) {
    msg[2] = '1';
    actualDelayTime = cycleDelayTime * 10;
  }
  if(currentLat < middleLat - radiusLat || middleLat > middleLat + radiusLat || currentLon < middleLon - radiusLon || middleLon > middleLon + radiusLon) {
    msg[3] = '1';
    actualDelayTime = cycleDelayTime * 10;
  }
}

void smartDelay(int timeToWait) {
  unsigned long theEnd = timeToWait + millis();
  while(millis() < theEnd && !Serial.available()){
    delay(100);
  }
}

void turnAllOff(){
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);
}

void checkUsbInMsg() {
  if(Serial.available()){
    lastInMsg = Serial.readString();
    Serial.print("usbInMsg: ");  // Test
    Serial.println(lastInMsg);  // Test
  }
}

void loraConnect() {
  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");  // Test
      while (!connected) {
        connected = modem.joinOTAA(appEui, appKey);
        }
  }
}

void checkLoraInMsg() {
  if (!modem.available()) {
    return;
  }
  char rcv[64];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  Serial.print("Received via Lora: ");  // Test
  Serial.println(rcv);  // Test
  lastInMsg = String(rcv);
}

void sendUsbMsg(char msg[]) {
  Serial.print("{'msg':'[");
  Serial.print(msg);
  Serial.print("]', 'state':[");
  Serial.print(currentTemp);
  Serial.print(", ");
  Serial.print(currentVolt);
  Serial.print(", ");
  Serial.print("[");
  Serial.print(currentLat, 6);
  Serial.print(", ");
  Serial.print(currentLon, 6);
  Serial.print("]");
  Serial.println("]}");
}

boolean sendLoraMsg(char msg[]) {
  msgLoraT loraMsg;
  loraMsg.alarms = (msg[0] << 7) | (msg[1] << 5) | (msg[2] << 4) | msg[3];
  loraMsg.temp = (int)((currentTemp + 20) * 100);
  loraMsg.volt = (int)(currentVolt * 100);
  loraMsg.lat = (int)((currentLat + 90) * 1000000);
  loraMsg.lon = (int)((currentLon + 180) * 1000000);
  // Test:
  Serial.println(loraMsg.alarms, HEX);
  Serial.println(loraMsg.temp, HEX);
  Serial.println(loraMsg.volt, HEX);
  Serial.println(loraMsg.lat, HEX);
  Serial.println(loraMsg.lon,  HEX);
  //
  int err;
  modem.beginPacket();
  modem.write(byte(loraMsg.alarms));
  modem.write((byte*)&loraMsg.temp, 2);
  modem.write((byte*)&loraMsg.volt, 2);
  modem.write((byte*)&loraMsg.lat, 4);
  modem.write((byte*)&loraMsg.lon, 4);
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly via Lora!");  // Test
    return true;
  } else {
    // Test
    Serial.println("Error sending message via Lora:(");
    Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
    Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
    return false;
  }
}

void setMiddleTemp(float newMiddleTemp) {
  middleTemp = newMiddleTemp;
}

void setRadiusTemp(float newRadiusTemp) {
  radiusTemp = abs(newRadiusTemp);
}

void setMiddleLat(float newMiddleLat) {
  middleLat = abs(newMiddleLat);
}

void setRadiusLat(float newRadiusLat) {
  radiusLat = abs(newRadiusLat);
}

void setMiddleLon(float newMiddleLon) {
  middleLon = abs(newMiddleLon);
}

void setRadiusLon(float newRadiusLon) {
  radiusLon = abs(newRadiusLon);
}

void setCycleDelayTime(float newCycleDelayTime) {
  cycleDelayTime = abs(newCycleDelayTime);
}

void setLoraDelayTime(float newLoraDelayTime) {
  loraDelayTime = abs(newLoraDelayTime);
}

void updateLoraNextTime() {
  loraNextTime = loraDelayTime * 60000 + millis(); 
}

void setMinVolt(int newMinVolt) {
  minVolt = abs(newMinVolt);
}

void updateCurrentTemp() {
  currentTemp = float(analogRead(tempPin)) * 330.0 / 1024.0; 
}

void updateCurrentVolt() {
  currentVolt = (float(analogRead(voltPin)) * 3.3 / 1024.0) * ((R1 + R2) / R2);
}

void updateCurrentLatAndLon() {
  while(gpsSerial.available()) { // Check for gps data
    if(gps.encode(gpsSerial.read())) { // Encode gps data
      gps.f_get_position(&currentLat,&currentLon); // get latitude and longitude
    }
  }
}

void lookAtMe() {
  analogWrite(ledPin, lookAtMeLevel);
  analogWrite(buzzerPin, lookAtMeLevel);
  smartDelay(1000);
  lookAtMeLevel += 5;
  analogWrite(ledPin, 0);
  analogWrite(buzzerPin, 0);
  smartDelay(1000 - lookAtMeLevel);
}
