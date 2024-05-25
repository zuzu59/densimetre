// Tests du push de l'accéléromètre et de la sonde de température one wire sur mqtt
// Ajout aussi de la nouvelle couche WIFI manager ainsi que l'OTA
// ATTENTION, ce code a été testé sur un esp32-c3. Pas testé sur les autres boards !
//
#define zVERSION "Densimètre accel_mqtt, zf240525.1111"
/*
Utilisation:

Plus valable ! Au moment du Reset, il faut mettre le capteur en 'vertical' sur l'axe des Y, afin que l'inclinaison du capteur soit correcte

Astuce:

Installation:

Pour les esp32-c3 super mini, il faut:
 * choisir comme board ESP32C3 Dev Module
 * disabled USB CDC On Boot et utiliser USBSerial. au lieu de Serial. pour la console !
 * changer le schéma de la partition à Minimal SPIFFS (1.9MB APP with OTA/190kB SPIFFS)

Pour le WiFiManager, il faut installer cette lib depuis le lib manager sur Arduino:
https://github.com/tzapu/WiFiManager

Pour l'accéléromètre, il faut installer la lib MPU6050 by Electrnoic Cats
depuis le library manager de l'Arduino IDE

Pour MQTT, il faut installer la lib (home-assistant-integration):
https://github.com/dawidchyrzynski/arduino-home-assistant

Pour JSON, il faut installer cette lib:
https://github.com/bblanchon/ArduinoJson

Sources:
https://forum.fritzing.org/t/need-esp32-c3-super-mini-board-model/20561
https://www.aliexpress.com/item/1005006005040320.html
https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino
https://dronebotworkshop.com/wifimanager/
https://github.com/universam1/iSpindel/tree/master
https://components101.com/sensors/mpu6050-module
https://github.com/electroniccats/mpu6050
https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_raw/MPU6050_raw.ino
https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/sensor-integer/sensor-integer.ino
https://chat.mistral.ai/    pour toute la partie API REST ᕗ
*/




#define DEBUG true
// #undef DEBUG



// General
const int ledPin = 8;    // the number of the LED pin
const int buttonPin = 9;  // the number of the pushbutton pin
float rrsiLevel = 0;      // variable to store the RRSI level
const int zSonarPulseOn = 50;    // délai pour sonarPulse
const int zSonarPulseOff = 100;    // délai pour sonarPulse
const int zSonarPulseWait = 500;    // délai pour sonarPulse
byte zSonarPulseState = 1;    // état pour sonarPulse
long zSonarPulseNextMillis = 0;    // état pour sonarPulse


float sensorValue1 = 0;  // variable to store the value coming from the sensor 1
float sensorValue2 = 0;  // variable to store the value coming from the sensor 2
float sensorValue3 = 0;  // variable to store the value coming from the sensor 3
float sensorValue4 = 0;  // variable to store the value coming from the sensor 4
float sensorValue5 = 0;  // variable to store the value coming from the sensor 5
#define TEMP_CELSIUS 0


// Solar Pulse
#include "zsolarpulse.h"


// WIFI
#include "zwifi.h"


// OTA WEB server
const char* host = "densimetre_1";
#include "otaWebServer.h"


// MQTT
#include "zmqtt.h"


// LittleFS
#include "zlittlefs.h"


// Accéléromètre MPU6050
#include "zaccelgyro.h"


// Temperature sensor
#include "ztemperature.h"


// Deep Sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;






void setup() {
  // Pulse deux fois pour dire que l'on démarre
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); delay(zSonarPulseOn); digitalWrite(ledPin, HIGH); delay(zSonarPulseOff);
  digitalWrite(ledPin, LOW); delay(zSonarPulseOn); digitalWrite(ledPin, HIGH); delay(zSonarPulseOff);
  delay(zSonarPulseWait);


    // Il faut lire la température tout de suite au début avant que le MCU ne puisse chauffer !
    // initTempSensor();
    initDS18B20Sensor();
    delay(200);
    readSensor();

  // start serial console
  USBSerial.begin(19200);
  USBSerial.setDebugOutput(true);       //pour voir les messages de debug des libs sur la console série !
  delay(3000);                          //le temps de passer sur la Serial Monitor ;-)
  USBSerial.println("\n\n\n\n**************************************\nCa commence !"); USBSerial.println(zVERSION);

  // si le bouton FLASH de l'esp32-c3 est appuyé dans les 3 secondes après le boot, la config WIFI sera effacée !
  pinMode(buttonPin, INPUT_PULLUP);
  if ( digitalRead(buttonPin) == LOW) {
    WiFiManager wm; wm.resetSettings();
    USBSerial.println("Config WIFI effacée !"); delay(1000);
    ESP.restart();
  }

  //Increment boot number and print it every reboot
  ++bootCount;
  sensorValue4 = bootCount;
  USBSerial.println("Boot number: " + String(bootCount));

  // start WIFI
  zStartWifi();
  sensorValue3 = WiFi.RSSI();

  // start OTA server
  otaWebServer();






    // initialize accelerator sensor
    Wire.begin(4, 5);     // J'ai branché mon sensor sur les pins 4 (DATA) et 5 (SLCK) de mon esp32c3 !
#ifdef DEBUG
    USBSerial.println("Initializing accelerator sensor...");
#endif
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
    accelgyro.setTempSensorEnabled(true);
    // verify connection
#ifdef DEBUG
    USBSerial.println("Testing accelerator sensor connections...");
#endif
    if (accelgyro.testConnection()){
#ifdef DEBUG
      USBSerial.println("Accelerator sensor connection successful !");
#endif
    } else {
      USBSerial.println("Accelerator sensor connection failed !");
    }
    delay(500);

    // Calibration de l'accéléromètre si le btn EN est appuyé juste après le reset
    pinMode(buttonPin, INPUT_PULLUP);
    if (digitalRead(buttonPin) == LOW) {
      // Calculate  offset
      calculateOffset();
      // Save offset into config
      saveConfig();
    }

#ifdef DEBUG
    mountFS();
    listDir(LittleFS, "/", 0); // List the directories up to one level beginning at the root directory
    readFile(LittleFS, "/config.json"); // Read the complete file
#endif

    readConfig();
    setOffset();






//     // USBSerial.println("Connect WIFI !");
//     // ConnectWiFi();
//     // digitalWrite(ledPin, HIGH);
//     // delay(500); 

//     // USBSerial.println("\n\nConnect MQTT !\n");
//     // ConnectMQTT();

//     USBSerial.println("\nC'est parti !\n");
}


void loop() {


  // readAcceleration();
  // readAccelerationMoy();
  // USBSerial.printf("x:%d,y:%d,z:%d\n", ax, ay, az);

  // Calculate Tilt
  // USBSerial.printf("inclinaison:%f\n", calculateTilt());





  // sensorValue1 = analogRead(sensorPin1);
  // sensorValue2 = analogRead(sensorPin2);

  // mqtt.loop();

  // Sensor1.setValue(sensorValue1);
  // Sensor2.setValue(sensorValue2);

  // USBSerial.printf("sensor1:%d,sensor2:%d\n", sensorValue1, sensorValue2);

  // delay(PUBLISH_INTERVAL);


  readSensor();
  USBSerial.print("sensor1:");
  USBSerial.print(sensorValue1);
  USBSerial.print(",sensor2:");
  USBSerial.print(sensorValue2);
  USBSerial.print(",sensor3:");
  USBSerial.print(sensorValue3);
  USBSerial.print(",sensor4:");
  USBSerial.print(sensorValue4);
  USBSerial.print(",sensor5:");
  USBSerial.println(sensorValue5);

  zDelay1(10000);
}

void zDelay1(long zDelayMili){
  long zDelay1NextMillis = zDelayMili + millis(); 
  while(zSonarPulseNextMillis > millis()){
    // OTA loop
    server.handleClient();
    // Un petit coup sonar pulse sur la LED pour dire que tout fonctionne bien
    sonarPulse();
  }
}
