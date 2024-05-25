// zf240525.1042

// Accéléromètre MPU6050
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
const int16_t moyNbVal = 10 ;
int16_t axOffset, ayOffset, azOffset;


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
