// Tests du push de l'accéléromètre et de la sonde de température one wire sur mqtt
// Ajout aussi de la nouvelle couche WIFI manager ainsi que l'OTA
// ATTENTION, ce code a été testé sur un esp32-c3. Pas testé sur les autres boards !
//
#define zVERSION "zf240524.2112"
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
const int zSonarPulseOn = 100;    // délai pour sonarPulse
const int zSonarPulseOff = 200;    // délai pour sonarPulse
const int zSonarPulseWait = 1000;    // délai pour sonarPulse
byte zSonarPulseState = 1;    // état pour sonarPulse
long zSonarPulseNextMillis = 0;    // état pour sonarPulse

float sensorValue1 = 0;  // variable to store the value coming from the sensor 1
float sensorValue2 = 0;  // variable to store the value coming from the sensor 2
float sensorValue3 = 0;  // variable to store the value coming from the sensor 3
float sensorValue4 = 0;  // variable to store the value coming from the sensor 4
float sensorValue5 = 0;  // variable to store the value coming from the sensor 5
#define TEMP_CELSIUS 0


// File system FS esp32-c3
#include <FS.h>          //this needs to be first
#include <LittleFS.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#define CFGFILE "/config.json"

struct iData{
  char name[33] = "Densimètre num 1";
  uint32_t sleeptime = 15 * 60;
  int16_t Offset[6];    //axOffsetInternal, ayOffsetInternal, azOffsetInternal, axOffset, ayOffset, azOffset
};

iData myData;


// Accéléromètre MPU6050
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
const int16_t moyNbVal = 10 ;
int16_t axOffset, ayOffset, azOffset;



// Deep Sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;


// Temperature sensor DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
// ATTENTION, c'est le brochage en VCC -> 0 pour le densimètre où il n'y a PAS de mesure de la tension de la batterie !
const int vccPin = 0;       // the number of the VCC pin
const int pullupPin = 1;    // the number of the PULLUP pin
const int oneWireBus = 2;   // GPIO where the DS18B20 is connected to
const int gndPin = 3;       // the number of the GND pin
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);



// Machine à état pour faire pulser deux fois la petite LED sans bloquer le système
void sonarPulse(){
  if (zSonarPulseNextMillis < millis()){
    switch (zSonarPulseState){
      // 1ère pulse allumée que l'on doit éteindre !
      case 1:
        digitalWrite(ledPin, HIGH);
        zSonarPulseNextMillis = millis() + zSonarPulseOff;
        zSonarPulseState = 2;
        break;
      // 1ère pulse éteinte que l'on doit allumer !
      case 2:
        digitalWrite(ledPin, LOW);
        zSonarPulseNextMillis = millis() + zSonarPulseOn;
        zSonarPulseState = 3;
        break;
      // 2e pulse allumée que l'on doit éteindre et attendre le wait !
      case 3:
        digitalWrite(ledPin, HIGH);
        zSonarPulseNextMillis = millis() + zSonarPulseWait;
        zSonarPulseState = 4;
        break;
      // 2e pulse éteinte pendant le wait que l'on doit allumer !
      case 4:
        digitalWrite(ledPin, LOW);
        zSonarPulseNextMillis = millis() + zSonarPulseOn;
        zSonarPulseState = 1;
        break;
    }
  }
}


// WIFI
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include "secrets.h"
WiFiClient client;
HTTPClient http;

static void ConnectWiFi() {
    WiFi.mode(WIFI_STA); //Optional    
    WiFiManager wm;
    bool res;
    res = wm.autoConnect("esp32_wifi_config",""); // password protected ap
    if(!res) {
        USBSerial.println("Failed to connect");
        // ESP.restart();
    }
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  //c'est pour le Lolin esp32-c3 mini V1 ! https://www.wemos.cc/en/latest/c3/c3_mini_1_0_0.html
    int txPower = WiFi.getTxPower();
    USBSerial.print("TX power: ");
    USBSerial.println(txPower);
    USBSerial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED){
        USBSerial.print(".");
        delay(100);
    }
    USBSerial.println("\nConnected to the WiFi network");
    USBSerial.print("SSID: ");
    USBSerial.println(WiFi.SSID());
    USBSerial.print("RSSI: ");
    USBSerial.println(WiFi.RSSI());
    USBSerial.print("IP: ");
    USBSerial.println(WiFi.localIP());
}


// OTA WEB server
const char* host = "densimetre_1";
#include "otaWebServer.h"


// MQTT
#include <ArduinoHA.h>
#define DEVICE_NAME      "densimetre1"
#define SENSOR_NAME1     "Temp_Internal"
#define SENSOR_NAME2     "Battery"
#define SENSOR_NAME3     "RSSI"
#define SENSOR_NAME4     "bootCount"
#define SENSOR_NAME5     "Temp_DS18B20"

HADevice device(DEVICE_NAME);                // c'est le ID du device, il doit être unique !
HAMqtt mqtt(client, device);
unsigned long lastUpdateAt = 0;

// c'est le ID du sensor, il doit être unique !
HASensorNumber Sensor1(DEVICE_NAME SENSOR_NAME1, HASensorNumber::PrecisionP1);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)
HASensorNumber Sensor2(DEVICE_NAME SENSOR_NAME2, HASensorNumber::PrecisionP2);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)
HASensorNumber Sensor3(DEVICE_NAME SENSOR_NAME3);   // c'est le nom du sensor sur MQTT !
HASensorNumber Sensor4(DEVICE_NAME SENSOR_NAME4);   // c'est le nom du sensor sur MQTT !
HASensorNumber Sensor5(DEVICE_NAME SENSOR_NAME5, HASensorNumber::PrecisionP1);   // c'est le nom du sensor sur MQTT ! (PrecisionP1=x.1, PrecisionP2=x.01, ...)

static void ConnectMQTT() {
    device.setName(DEVICE_NAME);                // c'est le nom du device sur Home Assistant !
    mqtt.setDataPrefix(DEVICE_NAME);             // c'est le nom du device sur MQTT !
    device.setSoftwareVersion(zVERSION);
    device.setManufacturer("espressif");
    device.setModel("esp32-c3 super mini");

    Sensor1.setIcon("mdi:thermometer");
    Sensor1.setName(SENSOR_NAME1);           // c'est le nom du sensor sur Home Assistant !
    Sensor1.setUnitOfMeasurement("°C");

    Sensor2.setIcon("mdi:battery-charging-wireless-outline");
    Sensor2.setName(SENSOR_NAME2);           // c'est le nom du sensor sur Home Assistant !
    Sensor2.setUnitOfMeasurement("V");

    Sensor3.setIcon("mdi:wifi-strength-1");
    Sensor3.setName(SENSOR_NAME3);           // c'est le nom du sensor sur Home Assistant !
    Sensor3.setUnitOfMeasurement("dBm");

    Sensor4.setIcon("mdi:counter");
    Sensor4.setName(SENSOR_NAME4);           // c'est le nom du sensor sur Home Assistant !
    Sensor4.setUnitOfMeasurement("sum");

    Sensor5.setIcon("mdi:thermometer");
    Sensor5.setName(SENSOR_NAME5);           // c'est le nom du sensor sur Home Assistant !
    Sensor5.setUnitOfMeasurement("°C");

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
    USBSerial.println("MQTT connected");
}







bool formatLittleFS()
{
  USBSerial.print("\nneed to format LittleFS: ");
  LittleFS.end();
  LittleFS.begin();
  USBSerial.println(LittleFS.format());
  return LittleFS.begin();
}


bool mountFS(){
#ifdef DEBUG
  USBSerial.print("mounting fs...\n");
#endif
  // if LittleFS is not usable
  if (!LittleFS.begin()){
    USBSerial.println("Failed to mount file system");
    if (!formatLittleFS()){
      USBSerial.println("Failed to format file system - hardware issues!");
      return false;
    }
  }
  return true;
}


bool saveConfig(){
#ifdef DEBUG
  USBSerial.println("saving config...");
#endif
  JsonDocument doc;
  doc["Name"] = myData.name;
  doc["Sleep"] = myData.sleeptime;
  JsonArray array = doc["Offset"].to<JsonArray>();
  for (auto &&i : myData.Offset)
  {
    array.add(i);
  }
  mountFS();
  File configFile = LittleFS.open(CFGFILE, "w");
  if (!configFile)
  {
    USBSerial.println("failed to open config file for writing");
    LittleFS.end();
    return false;
  }
  else
  {
    serializeJson(doc, configFile);
#ifdef DEBUG
    USBSerial.println("serializeJson...");
    serializeJson(doc, USBSerial);
#endif
    configFile.flush();
    configFile.close();
#ifdef DEBUG
    USBSerial.println("\nsaved successfully");
#endif
    return true;
  }
}


bool readConfig(){
#ifdef DEBUG
  USBSerial.println("read config...");
  USBSerial.print("mounting FS...");
#endif
  if (!LittleFS.begin())
  {
    USBSerial.println(" ERROR: failed to mount FS!");
    return false;
  }
  else
  {
#ifdef DEBUG
    USBSerial.println(" mounted!");
#endif
    if (!LittleFS.exists(CFGFILE))
    {
      USBSerial.println("ERROR: failed to load json config");
      return false;
    }
    else
    {
      // file exists, reading and loading
#ifdef DEBUG
      USBSerial.println("reading config file");
#endif
      File configFile = LittleFS.open(CFGFILE, "r");
      if (!configFile)
      {
        USBSerial.println("ERROR: unable to open config file");
      }
      else
      {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, configFile);
        if (error)
        {
          USBSerial.print("deserializeJson() failed: ");
          USBSerial.println(error.c_str());
        }
        else
        {
          if (doc.containsKey("Name"))
            strcpy(myData.name, doc["Name"]);
          if (doc.containsKey("Sleep"))
            myData.sleeptime = doc["Sleep"];
          if (doc.containsKey("Offset"))
          {
            for (size_t i = 0; i < (sizeof(myData.Offset) / sizeof(*myData.Offset)); i++)
            {
              myData.Offset[i] = doc["Offset"][i];
            }
          }
#ifdef DEBUG
          USBSerial.println("parsed config:");
          serializeJson(doc, USBSerial);
          USBSerial.println("");
#endif
        }
      }
    }
  }
  return true;
}


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    USBSerial.printf("Listing directory: %s\r\n", dirname);
    File root = fs.open(dirname);
    if(!root){
        USBSerial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        USBSerial.println(" - not a directory");
        return;
    }
    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            USBSerial.print("  DIR : ");
            USBSerial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            USBSerial.print("  FILE: ");
            USBSerial.print(file.name());
            USBSerial.print("\tSIZE: ");
            USBSerial.println(file.size());
        }
        file = root.openNextFile();
    }
}


void readFile(fs::FS &fs, const char * path){
#ifdef DEBUG
    USBSerial.printf("Reading file: %s\r\n", path);
#endif
    File file = fs.open(path);
    if(!file || file.isDirectory()){
        USBSerial.println("- failed to open file for reading");
        return;
    }
#ifdef DEBUG
    USBSerial.println("- read from file:");
#endif
    while(file.available()){
        USBSerial.write(file.read());
    }
    USBSerial.println("");
    file.close();
}


void calculateOffset() {
    USBSerial.println("Updating internal sensor offsets...");

    USBSerial.println("Au boot...");
    USBSerial.printf("Internal sensor offsets: %d\t%d\t%d\n", accelgyro.getXAccelOffset(), accelgyro.getYAccelOffset(), accelgyro.getZAccelOffset());

    USBSerial.println("Avant calibration...");    accelgyro.setXAccelOffset(0);   accelgyro.setYAccelOffset(0);   accelgyro.setZAccelOffset(0);
    USBSerial.printf("Internal sensor offsets: %d\t%d\t%d\n", accelgyro.getXAccelOffset(), accelgyro.getYAccelOffset(), accelgyro.getZAccelOffset());
    accelgyro.getAcceleration(&ax, &ay, &az);
    USBSerial.printf("Acceleration: x:%d,y:%d,z:%d\n", ax, ay, az);

    USBSerial.println("Après calibration...");
    accelgyro.CalibrateAccel(6);
    myData.Offset[0] =accelgyro.getXAccelOffset();    myData.Offset[1] =accelgyro.getYAccelOffset();    myData.Offset[2] =accelgyro.getZAccelOffset();
    USBSerial.printf("Internal sensor offsets: %d\t%d\t%d\n", myData.Offset[0], myData.Offset[1], myData.Offset[2]);
    accelgyro.getAcceleration(&ax, &ay, &az);
    USBSerial.printf("Acceleration: x:%d,y:%d,z:%d\n", ax, ay, az);
    myData.Offset[3] = -ax;  myData.Offset[4] = -ay;  myData.Offset[5] = -az;
    axOffset = myData.Offset[3];   ayOffset = myData.Offset[4];   azOffset = myData.Offset[5];
    ax = ax + axOffset;   ay = ay + ayOffset;   az = az + azOffset;
    USBSerial.printf("Acceleration: x:%d,y:%d,z:%d\n", ax, ay, az);
    USBSerial.println("End of updating internal sensor offsets...");
}


void setOffset() {
#ifdef DEBUG
  USBSerial.println("Set offset");
  USBSerial.println("Set accelerator offset");
  USBSerial.printf("Internal sensor offsets: %d\t%d\t%d\n", myData.Offset[0], myData.Offset[1], myData.Offset[2]);
  USBSerial.printf("Sensor offsets: %d\t%d\t%d\n", myData.Offset[3], myData.Offset[4], myData.Offset[5]);
#endif
  axOffset = myData.Offset[3];   ayOffset = myData.Offset[4];   azOffset = myData.Offset[5];
}



// #define DEBUG true


void readAcceleration() {
#ifdef DEBUG
  USBSerial.println("Read acceleration...");
#endif
  accelgyro.getAcceleration(&ax, &ay, &az);
  ax = ax + axOffset;   ay = ay + ayOffset;   az = az + azOffset;
#ifdef DEBUG
  USBSerial.printf("Acceleration: x:%d,y:%d,z:%d\n", ax, ay, az);
#endif
}


void readAccelerationMoy() {
#ifdef DEBUG
  USBSerial.println("Read acceleration moyenne...");
#endif
  long axSum = 0, aySum = 0, azSum = 0;
  for (size_t i = 0; i < moyNbVal; i++)
  {
    readAcceleration();
    axSum = axSum + ax; aySum = aySum + ay; azSum = azSum + az; 
  }
  ax = axSum / moyNbVal; ay = aySum / moyNbVal; az = azSum / moyNbVal; 
#ifdef DEBUG
  USBSerial.printf("Acceleration moyenne: x:%d,y:%d,z:%d\n", ax, ay, az);
#endif
}


#undef DEBUG



float calculateTilt() {

  if (ax == 0 && ay == 0 && az == 0)
    return 0.f;

  // return (acos(abs(ay) / (sqrt(ax * ax + ay * ay + az * az))) * 180.0 / M_PI);
  return 180 - (acos(abs(ay) / (sqrt(ax * ax + ay * ay + az * az))) * 180.0 / M_PI * 2);
}


// Temperature sensor DS18B20 initialising
void initDS18B20Sensor(){
    pinMode(gndPin, OUTPUT);   // gnd
    digitalWrite(gndPin, LOW);
    pinMode(pullupPin, INPUT_PULLUP);   // pull up
    pinMode(vccPin, OUTPUT );   // vcc
    digitalWrite(vccPin, HIGH);
    // Start the DS18B20 sensor
    sensors.begin();
}


// Lit les senseurs
void readSensor(){
    // lit la température interne
    // temp_sensor_read_celsius(&sensorValue1);
    // // lit la tension de la batterie
    // uint16_t reading = analogRead(sensorPin);
    // // fonction de conversion bit to volts de l'ADC avec le diviseur résistif et de la diode !
    // // voir https://raw.githubusercontent.com/zuzu59/esp32-c3-thermo-mqtt-dsleep/master/fonction_conversion_ADC.txt
    // // 0.001034 * (ADC - 2380) + 3.6
    // sensorValue2 = 0.001034 * (reading - 2380) + 3.6;            // 2960 pour 4.2V et 2380 pour 3.6V
    // lit la température du DS18B20
    sensors.requestTemperatures(); 
    sensorValue5 = sensors.getTempCByIndex(0);
}


// Envoie les senseurs au mqtt
void sendSensorMqtt(){
    mqtt.loop();
    Sensor1.setValue(sensorValue1);
    Sensor2.setValue(sensorValue2);
    Sensor3.setValue(sensorValue3);
    Sensor4.setValue(sensorValue4);
    Sensor5.setValue(sensorValue5);
}




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
  digitalWrite(ledPin, HIGH);
  USBSerial.println("Connect WIFI !");
  ConnectWiFi();
  digitalWrite(ledPin, LOW);

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
  // OTA loop
  server.handleClient();

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




    // Un petit coup sonar pulse sur la LED pour dire que tout fonctionne bien
    sonarPulse();

    delay(1000);
}

