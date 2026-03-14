#include "FS.h"
#include "SD.h"
#include "GY521.h"
#include <SPI.h>
#include <BMP280.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define CS_PIN    4

#define SD_CS     33
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23

#define HOT_PIN   32
#define TEST_PIN  35

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

GY521 sensor(0x68);
BMP280 bmp280;
Adafruit_AHTX0 aht;

const char* labels[]={
  "now[ms]",
  "AHT_tmp[C]","AHT_hum",
  "BMP_temp[C]","BMP_pres",
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
  float AHT_temp, AHT_hum;
  float BMP_temp, BMP_pres;
  float3d accel;
  float3d gyro;
  float gtemp;
  float3d mag;
  float volt;
};
struct device{
  bool OK;
  unsigned long last;
  unsigned long last_alive;
  unsigned long timeout;
};
struct SystemInfo {
  device SD,AHT,BMP,gyro,mag;
};

SensorData data;
SystemInfo check;

char row[2048];
int unitime;
uint8_t payload[sizeof(SensorData) + 2];



void checkI2CDevices() {
    check.gyro.OK = check.gyro.OK && i2cDevicePresent(0x68);
    Serial.print(check.gyro.OK);
    check.BMP.OK = check.BMP.OK && i2cDevicePresent(0x77);
    Serial.print(check.BMP.OK);
    check.AHT.OK =check.AHT.OK && i2cDevicePresent(0x38);
    Serial.print(check.AHT.OK);
    check.mag.OK =check.mag.OK && i2cDevicePresent(0x1E);
    Serial.println(check.mag.OK);
}

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}
void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{%s:%d,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.0f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.4f}\n",
    labels[i++],data.now,
    labels[i++],data.AHT_temp,labels[i++],data.AHT_hum,
    labels[i++],data.BMP_temp,labels[i++],data.BMP_pres,
    labels[i++],data.gyro.x,labels[i++],data.gyro.y,labels[i++],data.gyro.z,
    labels[i++],data.accel.x,labels[i++],data.accel.y,labels[i++],data.accel.z,
    labels[i++],data.gtemp,
    labels[i++],data.mag.x,labels[i++],data.mag.y,labels[i++],data.mag.z,
    labels[i++],data.volt);
}
void dataToCsv(SensorData data,char buffer[],int len){
  snprintf(buffer, len,
    "%d,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f\n",
    data.now,
    data.AHT_temp,data.AHT_hum,
    data.BMP_temp,data.BMP_pres,
    data.gyro.x,data.gyro.y,data.gyro.z,
    data.accel.x,data.accel.y,data.accel.z,
    data.gtemp,
    data.mag.x,data.mag.y,data.mag.z,
    data.volt
  );
}
void sdConnect(){
  Serial.println("sd connect");
  check.SD.OK=SD.begin(SD_CS);
  if(check.SD.OK){
    check.SD.last=millis();
    if (!SD.exists("/final-calibration.csv")) {
      File file = SD.open("/final-calibration.csv",FILE_WRITE);
      if(file){
        for(int i=0;i<sizeof(labels)/sizeof(labels[0]);i++){
          file.print(labels[i]);
          if(i<sizeof(labels)/sizeof(labels[0])-1)file.print(",");
          else file.print("\n");
        }
        file.close();}
    }
  }
}

void ahtConnect(){
  Serial.println("aht connect");
  check.AHT.last=millis();
  if(i2cDevicePresent(0x38))check.AHT.OK=aht.begin();
  else Serial.println("aht not found");
}
void bmpConnect(){
  Serial.println("bmp connect");
  check.BMP.last=millis();
  if(i2cDevicePresent(0x77))check.BMP.OK=bmp280.begin()==0;
  else Serial.println("bmp not found");
}
void accelConnect(){
  Serial.println("accel connect");
  check.gyro.last=millis();
  if(i2cDevicePresent(0x68)){
    check.gyro.OK=sensor.wakeup();
    if(check.gyro.OK) {
      sensor.setAccelSensitivity(0);  //  2g
      sensor.setGyroSensitivity(0);   //  250 degrees/s
      sensor.setThrottle();
    }
  }else Serial.println("accel not found");
}
void magnConnect() {
  Serial.println("HMC5883L connect...");
  check.mag.last = millis();
  check.mag.OK =mag.begin();
}
void startI2CDevices(){
  delay(500);
  Wire.begin();
  ahtConnect();
  bmpConnect();
  accelConnect();
  magnConnect();
}
void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  startI2CDevices();
  
  pinMode(HOT_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  
  check.AHT.timeout=1000;
  check.BMP.timeout=1000;
  check.gyro.timeout=1000;
  check.mag.timeout=1000;
  
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  sdConnect();
}

void loop() {
  
  checkI2CDevices();
  if(!(check.AHT.OK || check.BMP.OK || check.gyro.OK || check.mag.OK)){
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
  if(check.AHT.OK){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    data.AHT_temp = temp.temperature;
    data.AHT_hum = humidity.relative_humidity;
  }
  else if (millis()-check.AHT.last>check.AHT.timeout){
    ahtConnect();
  }
  if(check.BMP.OK){
    data.BMP_temp = bmp280.getTemperature();
    data.BMP_pres = bmp280.getPressure();
  }
  else if (millis()-check.BMP.last>check.BMP.timeout){
    bmpConnect();
  }
  if(check.mag.OK){
      sensors_event_t event; 
      mag.getEvent(&event);
      data.mag.x=event.magnetic.x;
      data.mag.y=event.magnetic.y;
      data.mag.z=event.magnetic.z;
  }
  else if (millis()-check.mag.last>check.mag.timeout){
    magnConnect();
  }

  data.volt =analogRead(TEST_PIN);
  
  data.now=millis();
  
  
  dataToCsv(data,row,sizeof(row));
  Serial.print(row);
  if(check.SD.OK){
    File file = SD.open("/final-calibration.csv", FILE_APPEND);
    if (file) {
      check.SD.last=millis();
      file.print(row);
      file.close();
      check.SD.last=millis();
    }else{
      check.SD.OK=false;
    }
  }else sdConnect();

}
