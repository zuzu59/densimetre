// Tests de sauvetage et restauration de l'offset de l'accéléromètre dans le FS du esp32-c3
// Comme on va utiliser le mode dsleep, c'est important de pouvoir sauvegarder l'état de l'offset
// Envoie aussi le résultat des senseurs sur le mqtt pour home assistant (pas en fonction actuellement !)
// ATTENTION, ce code a été testé sur un esp32-c3. Pas testé sur les autres bords !
//
// zf2403120.1600
//
// Utilisation:
// Plus valable ! Au moment du Reset, il faut mettre le capteur en 'vertical' sur l'axe des Y, afin que l'inclinaison du capteur soit correcte
// 
// Installation:
// Pour l'accéléromètre, il faut installer la lib MPU6050 by Electrnoic Cats
//  depuis le library manager de l'Arduino IDE
//
// Pour MQTT, il faut installer la lib (home-assistant-integration):
// https://github.com/dawidchyrzynski/arduino-home-assistant
//
// Sources:
// https://github.com/universam1/iSpindel/tree/master
// https://components101.com/sensors/mpu6050-module
// https://github.com/electroniccats/mpu6050
// https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_raw/MPU6050_raw.ino

// https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/sensor-integer/sensor-integer.ino


// #define DEBUG true



// General
# define LED 7
// int sensorPin1 = 1;   // select the input pin for the sensor 1
// long sensorValue1 = 0;  // variable to store the value coming from the sensor 1
// int sensorPin2 = 3;   // select the input pin for the sensor 2
// long sensorValue2 = 0;  // variable to store the value coming from the sensor 2
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
int16_t axOffset, ayOffset, azOffset;


// // WIFI
// #include <WiFi.h>
// #include "secrets.h"

// static void ConnectWiFi() {
//     USBSerial.printf("WIFI_SSID: %s\nWIFI_PASSWORD: %s\n", WIFI_SSID, WIFI_PASSWORD);
//     WiFi.mode(WIFI_STA); //Optional
//     WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//     WiFi.setTxPower(WIFI_POWER_8_5dBm);  //c'est pour le Lolin esp32-c3 mini V1 ! https://www.wemos.cc/en/latest/c3/c3_mini_1_0_0.html
//     int txPower = WiFi.getTxPower();
//     USBSerial.print("TX power: ");
//     USBSerial.println(txPower);
//     USBSerial.println("Connecting");
//     while(WiFi.status() != WL_CONNECTED){
//         USBSerial.print(".");
//         delay(100);
//     }
//     USBSerial.println("\nConnected to the WiFi network");
//     USBSerial.print("Local ESP32 IP: ");
//     USBSerial.println(WiFi.localIP());
// }


// // MQTT
// #include <ArduinoHA.h>
// #define DEVICE_NAME     "gazMQ136_137"
// #define SENSOR_NAME1     "H2S"
// #define SENSOR_NAME2     "NH3"

#define PUBLISH_INTERVAL  40000 // how often image should be published to HA (milliseconds)

// WiFiClient client;
// HADevice device(DEVICE_NAME);                // c'est le IDS du device, il doit être unique !
// HAMqtt mqtt(client, device);
// unsigned long lastUpdateAt = 0;

// // You should define your own ID.
// HASensorNumber Sensor1(SENSOR_NAME1);           // c'est le nom du sensor sur MQTT !
// HASensorNumber Sensor2(SENSOR_NAME2);           // c'est le nom du sensor sur MQTT !

// static void ConnectMQTT() {
//    device.setName(DEVICE_NAME);                // c'est le nom du device sur Home Assistant !
//     // device.setSoftwareVersion("1.0.0");
//     mqtt.setDataPrefix(DEVICE_NAME);             // c'est le nom du device sur MQTT !

//     Sensor1.setIcon("mdi:radiator");
//     Sensor1.setName(SENSOR_NAME1);           // c'est le nom du sensor sur Home Assistant !
//     Sensor1.setUnitOfMeasurement("ppm");

//     Sensor2.setIcon("mdi:radiator");
//     Sensor2.setName(SENSOR_NAME2);           // c'est le nom du sensor sur Home Assistant !
//     Sensor2.setUnitOfMeasurement("ppm");

//     mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
//     USBSerial.println("MQTT connected");
// }


// Redirection de la console
#define CONSOLE(...)                                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.print(__VA_ARGS__);                                                                                         \
  } while (0)

#define CONSOLELN(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.println(__VA_ARGS__);                                                                                       \
  } while (0)

#define CONSOLEF(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    USBSerial.printf(__VA_ARGS__);                                                                                       \
  } while (0)





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
    axOffset = -ax;   ayOffset = -ay;   azOffset = -az;
    myData.Offset[3] = axOffset;  myData.Offset[4] = ayOffset;  myData.Offset[5] = azOffset;
    ax = ax + axOffset;   ay = ay + ayOffset;   az = az + azOffset;
    USBSerial.printf("Acceleration: x:%d,y:%d,z:%d\n", ax, ay, az);
    USBSerial.println("End of updating internal sensor offsets...");
}



float calculateTilt() {

  if (ax == 0 && ay == 0 && az == 0)
    return 0.f;

  // return (acos(abs(ay) / (sqrt(ax * ax + ay * ay + az * az))) * 180.0 / M_PI);
  return (acos(abs(ay) / (sqrt(ax * ax + ay * ay + az * az))) * 180.0 / M_PI * 2) -90 ;
}







bool formatLittleFS()
{
  CONSOLE("\nneed to format LittleFS: ");
  LittleFS.end();
  LittleFS.begin();
  CONSOLELN(LittleFS.format());
  return LittleFS.begin();
}


bool mountFS(){
#ifdef DEBUG
  CONSOLE("mounting fs...\n");
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
  CONSOLELN("saving config...");
#endif
  JsonDocument doc;
  doc["Name"] = myData.name;
  doc["Sleep"] = myData.sleeptime;
  JsonArray array = doc["Offset"].to<JsonArray>();
  for (auto &&i : myData.Offset)
  {
    array.add(i);
  }
  File configFile = LittleFS.open(CFGFILE, "w");
  if (!configFile)
  {
    CONSOLELN("failed to open config file for writing");
    LittleFS.end();
    return false;
  }
  else
  {
    serializeJson(doc, configFile);
#ifdef DEBUG
    CONSOLELN("serializeJson...");
    serializeJson(doc, USBSerial);
#endif
    configFile.flush();
    configFile.close();
#ifdef DEBUG
    CONSOLELN("\nsaved successfully");
#endif
    return true;
  }
}




bool readConfig(){
  CONSOLE("mounting FS...");

  if (!LittleFS.begin())
  {
    CONSOLELN(" ERROR: failed to mount FS!");
    return false;
  }
  else
  {
    CONSOLELN(" mounted!");
    if (!LittleFS.exists(CFGFILE))
    {
      CONSOLELN("ERROR: failed to load json config");
      return false;
    }
    else
    {
      // file exists, reading and loading
      CONSOLELN("reading config file");
      File configFile = LittleFS.open(CFGFILE, "r");
      if (!configFile)
      {
        CONSOLELN("ERROR: unable to open config file");
      }
      else
      {
        // size_t size = configFile.size();
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, configFile);
        if (error)
        {
          CONSOLE("deserializeJson() failed: ");
          CONSOLELN(error.c_str());
        }
        else
        {
          if (doc.containsKey("Name"))
            strcpy(myData.name, doc["Name"]);
          // if (doc.containsKey("Token"))
          //   strcpy(myData.token, doc["Token"]);
          // if (doc.containsKey("Server"))
          //   strcpy(myData.server, doc["Server"]);
          // if (doc.containsKey("Sleep"))
          //   myData.sleeptime = doc["Sleep"];
          // if (doc.containsKey("API"))
          //   myData.api = doc["API"];
          // if (doc.containsKey("Port"))
          //   myData.port = doc["Port"];
          // if (doc.containsKey("Channel"))
          //   myData.channel = doc["Channel"];
          // if (doc.containsKey("URI"))
          //   strcpy(myData.uri, doc["URI"]);
          // if (doc.containsKey("Username"))
          //   strcpy(myData.username, doc["Username"]);
          // if (doc.containsKey("Password"))
          //   strcpy(myData.password, doc["Password"]);
          // if (doc.containsKey("Job"))
          //   strcpy(myData.job, doc["Job"]);
          // if (doc.containsKey("Instance"))
          //   strcpy(myData.instance, doc["Instance"]);
          // if (doc.containsKey("Vfact"))
          //   myData.vfact = doc["Vfact"];
          // if (doc.containsKey("TS"))
          //   myData.tempscale = doc["TS"];
          // if (doc.containsKey("OWpin"))
          //   myData.OWpin = doc["OWpin"];
          // if (doc.containsKey("SSID"))
          //   myData.ssid = (const char *)doc["SSID"];
          // if (doc.containsKey("PSK"))
          //   myData.psk = (const char *)doc["PSK"];
          // if (doc.containsKey("POLY"))
          //   strcpy(myData.polynominal, doc["POLY"]);
// #if API_MQTT_HASSIO
//           if (doc.containsKey("Hassio"))
//             myData.hassio = doc["Hassio"];
// #endif
          // if (doc.containsKey("UseHTTPS"))
          //   myData.usehttps = doc["UseHTTPS"];
          if (doc.containsKey("Offset"))
          {
            for (size_t i = 0; i < (sizeof(myData.Offset) / sizeof(*myData.Offset)); i++)
            {
              myData.Offset[i] = doc["Offset"][i];
            }
          }

          CONSOLELN("parsed config:");
#ifdef DEBUG
          serializeJson(doc, Serial);
          CONSOLELN();
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
    file.close();
}



void writeFile(fs::FS &fs, const char * path, const char * message){
    USBSerial.printf("Writing file: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        USBSerial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        USBSerial.println("- file written");
    } else {
        USBSerial.println("- write failed");
    }
    file.close();
}





void setup() {
    USBSerial.begin(19200);
    USBSerial.setDebugOutput(true);       //pour voir les messages de debug des libs sur la console série !
    delay(3000);  //le temps de passer sur la Serial Monitor ;-)
    USBSerial.println("\n\n\n\n**************************************\nCa commence !\n");

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    delay(500); 
    digitalWrite(LED, LOW);

    // initialize device
    Wire.begin(4, 5);     // J'ai branché mon sensor sur les pins 4 (DATA) et 5 (SLCK) de mon esp32c3 !
    USBSerial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
    accelgyro.setTempSensorEnabled(true);
    // verify connection
    USBSerial.println("Testing device connections...");
    USBSerial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    delay(500);
    // Calculate  offset
    calculateOffset();


    mountFS();
    saveConfig();
#ifdef DEBUG
    listDir(LittleFS, "/", 0); // List the directories up to one level beginning at the root directory
#endif

    readFile(LittleFS, "/config.json"); // Read the complete file

    // readConfig();


    // delay(500);
    // // Calculate  offset
    // calculateOffset();




    // USBSerial.println("Connect WIFI !");
    // ConnectWiFi();
    // digitalWrite(LED, HIGH);
    // delay(500); 

    // USBSerial.println("\n\nConnect MQTT !\n");
    // ConnectMQTT();

    USBSerial.println("\nC'est parti !\n");
}


void loop() {
    digitalWrite(LED, HIGH);
    delay(100); 
    digitalWrite(LED, LOW);

    // read raw accel measurements from device
    // accelgyro.getAcceleration(&ax, &ay, &az);

    // Offset correction
    ax = ax+axOffset;
    ay = ay+ayOffset;
    az = az+azOffset;

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
    delay(300000);
}

