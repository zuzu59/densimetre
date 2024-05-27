// zf240525.1718

// MQTT
#include <ArduinoHA.h>
#define DEVICE_NAME      "densimetre1"       // toujours en minuscules !
#define SENSOR_NAME1     "Temp_Internal"
#define SENSOR_NAME2     "Tilt"
#define SENSOR_NAME3     "RSSI"
#define SENSOR_NAME4     "BootCount"
#define SENSOR_NAME5     "Temp_DS18B20"

#define PUBLISH_INTERVAL  2000 // how often image should be published to HA (milliseconds)

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

    Sensor2.setIcon("mdi:angle-acute");
    Sensor2.setName(SENSOR_NAME2);           // c'est le nom du sensor sur Home Assistant !
    Sensor2.setUnitOfMeasurement("°");

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

// Envoie les senseurs au mqtt
void sendSensorMqtt(){
    mqtt.loop();
    Sensor1.setValue(sensorValue1);
    Sensor2.setValue(sensorValue2);
    Sensor3.setValue(sensorValue3);
    Sensor4.setValue(sensorValue4);
    Sensor5.setValue(sensorValue5);
}

