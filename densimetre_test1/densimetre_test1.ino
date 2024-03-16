// Simple tests du capteur accéléromètre MPU6050 que je vais utiliser pour mon densimètre électronique
// Envoie aussi le résultat des senseurs sur le mqtt pour home assistant (pas en fonction actuellement !)
// ATTENTION, ce code a été testé sur un esp32-c3. Pas testé sur les autres bords !
//
// zf240316.1831
//
// Installation:

// Pour l'accéléromètre, il faut installer la lib MPU6050 by Electrnoic Cats
//  depuis le library manager de l'Arduino IDE
//
// Pour MQTT, il faut installer la lib (home-assistant-integration):
// https://github.com/dawidchyrzynski/arduino-home-assistant
//
// Sources:
// https://components101.com/sensors/mpu6050-module
// https://github.com/electroniccats/mpu6050
// https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_raw/MPU6050_raw.ino

// https://github.com/dawidchyrzynski/arduino-home-assistant/blob/main/examples/sensor-integer/sensor-integer.ino

// MPU6050
#include "MPU6050.h"
#include "Wire.h"
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


// int sensorPin1 = 1;   // select the input pin for the sensor 1
// long sensorValue1 = 0;  // variable to store the value coming from the sensor 1
// int sensorPin2 = 3;   // select the input pin for the sensor 2
// long sensorValue2 = 0;  // variable to store the value coming from the sensor 2




// // WIFI
// #include <WiFi.h>
// #include "secrets.h"


// // MQTT
// #include <ArduinoHA.h>
// #define DEVICE_NAME     "gazMQ136_137"
// #define SENSOR_NAME1     "H2S"
// #define SENSOR_NAME2     "NH3"

#define PUBLISH_INTERVAL  4000 // how often image should be published to HA (milliseconds)

// WiFiClient client;
// HADevice device(DEVICE_NAME);                // c'est le IDS du device, il doit être unique !
// HAMqtt mqtt(client, device);
// unsigned long lastUpdateAt = 0;

// // You should define your own ID.
// HASensorNumber Sensor1(SENSOR_NAME1);           // c'est le nom du sensor sur MQTT !
// HASensorNumber Sensor2(SENSOR_NAME2);           // c'est le nom du sensor sur MQTT !


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




void setup() {
    USBSerial.begin(19200);
    USBSerial.setDebugOutput(true);       //pour voir les messages de debug sur la console série !
    delay(3000);  //le temps de passer sur la Serial Monitor ;-)
    USBSerial.println("\n\n\n\n**************************************\nCa commence !\n");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500); 
    digitalWrite(LED_BUILTIN, LOW);






    // USBSerial.println("Connect WIFI !");
    // ConnectWiFi();
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(500); 

    // USBSerial.println("\n\nConnect MQTT !\n");
    // ConnectMQTT();

    USBSerial.println("C'est parti !\n");
}


void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100); 
    digitalWrite(LED_BUILTIN, LOW);

    // sensorValue1 = analogRead(sensorPin1);
    // sensorValue2 = analogRead(sensorPin2);

    // mqtt.loop();

    // Sensor1.setValue(sensorValue1);
    // Sensor2.setValue(sensorValue2);

    // USBSerial.printf("sensor1:%d,sensor2:%d\n", sensorValue1, sensorValue2);

    delay(PUBLISH_INTERVAL);
}

