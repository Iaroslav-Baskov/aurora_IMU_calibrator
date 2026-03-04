
#include "GY521.h"
#include <SPI.h>
#include <Wire.h>

#define CS_PIN    4

#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23

#define HOT_PIN   32
#define TEST_PIN  35

#define QMC5883L_ADDRESS 0x0D

GY521 sensor(0x68);

const char* labels[]={
  "now[ms]",
  "gx","gy","gz",
  "ax","ay","az",
  "gtemp",
  "magx","magy","magz",
  "volt",
};


struct float3d{
  float x,y,z;};
struct int16_t3d{
  int16_t x,y,z;};
  
struct SensorData {
  unsigned long now;
  float3d accel;
  float3d gyro;
  float gtemp;
  float3d mag;
};
struct device{
  bool OK;
  unsigned long last;
  unsigned long last_alive;
  unsigned long timeout;
};
struct SystemInfo {
  device gyro,mag;
};

SensorData data;
SystemInfo check;

char row[2048];
int unitime;
uint8_t payload[sizeof(SensorData) + 2];

void getMag(float3d &output,float t) {
    check.mag.last = millis();
    int16_t3d result;
  
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(QMC5883L_ADDRESS, 6);
  
    if (Wire.available() >= 6) {
      result.x = Wire.read() | (Wire.read() << 8);
      result.y = Wire.read() | (Wire.read() << 8);
      result.z = Wire.read() | (Wire.read() << 8);
      output.x=(float)result.x;
      output.y=(float)result.y;
      output.z=(float)result.z;
   }
}
void checkI2CDevices() {
    check.gyro.OK = check.gyro.OK && i2cDevicePresent(0x68);
//    Serial.print(check.gyro.OK);
    check.mag.OK =check.mag.OK && i2cDevicePresent(QMC5883L_ADDRESS);
//    Serial.println(check.mag.OK);
}

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}
void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{%s:%d,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.2f}\n",
    labels[i++],data.now,
    labels[i++],data.gyro.x,labels[i++],data.gyro.y,labels[i++],data.gyro.z,
    labels[i++],data.accel.x,labels[i++],data.accel.y,labels[i++],data.accel.z,
    labels[i++],data.gtemp,
    labels[i++],data.mag.x,labels[i++],data.mag.y,labels[i++],data.mag.z);
}
void dataToCsv(SensorData data,char buffer[],int len){
  snprintf(buffer, len,
    "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f\n",
    data.now,
    data.gyro.x,data.gyro.y,data.gyro.z,
    data.accel.x,data.accel.y,data.accel.z,
    data.gtemp,
    data.mag.x,data.mag.y,data.mag.z
  );
}
void accelConnect(){
//  Serial.println("accel connect");
  check.gyro.last=millis();
  if(i2cDevicePresent(0x68)){
    check.gyro.OK=sensor.wakeup();
    if(check.gyro.OK) {
      sensor.setAccelSensitivity(0);  //  2g
      sensor.setGyroSensitivity(0);   //  250 degrees/s
      sensor.setThrottle();
    }
  }
//  else Serial.println("accel not found");
}
void magnConnect(){
//  Serial.println("magn connect");
  check.mag.last=millis();
  
  check.mag.OK = i2cDevicePresent(QMC5883L_ADDRESS);
  if(check.mag.OK){
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x09);
    Wire.write(0b00011101); // OSR=512, 8G, 200Hz, continuous
    Wire.endTransmission();
  
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();
  }
//  else Serial.println("magn no found");
}
void startI2CDevices(){
  delay(500);
  Wire.begin();
  accelConnect();
  magnConnect();
}
void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  startI2CDevices();
  
  pinMode(HOT_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  
  check.gyro.timeout=1000;
  check.mag.timeout=1000;
  
  digitalWrite(CS_PIN, HIGH);
  
}

void loop() {
  
  checkI2CDevices();
  if(!(check.gyro.OK || check.mag.OK)){
    startI2CDevices();
  }
  if(check.gyro.OK){
      sensor.read();
      data.accel.x = sensor.getAccelX();
      data.accel.y = sensor.getAccelY();
      data.accel.z = sensor.getAccelZ();
    
      data.gyro.x = sensor.getGyroX();
      data.gyro.y = sensor.getGyroY();
      data.gyro.z = sensor.getGyroZ();
      data.gtemp =sensor.getTemperature();
      check.gyro.last=millis();
  }
  else if (millis()-check.gyro.last>check.gyro.timeout){
    accelConnect();
  }
  if(check.mag.OK){
    getMag(data.mag,data.gtemp);}
  else if (millis()-check.mag.last>check.mag.timeout){
    magnConnect();
  }
  
  data.now=millis();
  dataToCsv(data,row,sizeof(row));
  Serial.print(row);
  delay(200);

}
