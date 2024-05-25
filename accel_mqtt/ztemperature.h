// zf240525.1040

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
