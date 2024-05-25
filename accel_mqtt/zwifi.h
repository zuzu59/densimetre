// zf240525.1015


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


// start WIFI
void zStartWifi(){
    digitalWrite(ledPin, HIGH);
    USBSerial.println("Connect WIFI !");
    ConnectWiFi();
    digitalWrite(ledPin, LOW);
}
