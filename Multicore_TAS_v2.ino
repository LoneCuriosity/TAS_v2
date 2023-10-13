#include "BMI088.h"
#include <Smoothed.h> 
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>
#include "LittleFS.h"
#include <LoRa.h>

#define SEALEVELPRESSURE_HPA (873.60)
#define GPSSerial Serial1
#define GPSECHO false

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BME280 bme;
RTC_PCF8523 rtc;
File SDFile;

uint32_t timer = millis();

const int csPin = 17;        
const int resetPin = 21;    
const int irqPin = 20;       

const int _MISO = 8;
const int _MOSI = 11;
const int _CS = 9;
const int _SCK = 10;
const int MAX_BUFFER_SIZE = 64;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
float lastMillis, currentMillis, gz, gx, gy, gzS, gxS, gyS;
String dateStamp, Packet, IMU, BME, GPSVal, outgoing, receivedData, myGPS;
int status;

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

void setup() {
  Serial.begin(115200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {
  char c = GPS.read();

  if (GPSECHO)
    if (c) Serial.print(c);

  if (GPS.newNMEAreceived()) {
    //Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 2000) {
    timer = millis();

    if (GPS.fix) {

      GPSVal = String((bool)GPS.fix) + "," + String((int)GPS.fixquality) + "," + ddmmToDD(String(GPS.latitude,4),GPS.lat) + "," + ddmmToDD(String(GPS.longitude,4),GPS.lon) + "," + String(GPS.speed) + "," + String(GPS.altitude) + "," + String((int)GPS.satellites);
      char buffer[GPSVal.length() + 1];

      GPSVal.toCharArray(buffer, sizeof(buffer));

      for (int i = 0; i < GPSVal.length(); i++) {
        rp2040.fifo.push(static_cast<uint8_t>(buffer[i]));
      }
    }
  }
}

// Running on core1
void setup1() {
  Serial.begin(115200);

  pinMode(12, OUTPUT); // Red
  pinMode(13, OUTPUT); // Blue
  pinMode(14, OUTPUT); // Green

  SPI.setRX(16);
  SPI.setTX(19);
  SPI.setSCK(18);

  LoRa.setSPI(SPI);
  LoRa.enableCrc();
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6)) {                                      
    Serial.println("LoRa init failed. Check your connections.");    
    digitalWrite(12, HIGH);
    return;                 
  }

  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  
  if(!bme.begin(0x76, &Wire)){
    Serial.println("initialization failed bme280!");
    digitalWrite(13, HIGH);
    return;
  }

  if (!SD.begin(_CS, SPI1)) {
    Serial.println("initialization failed!");
    digitalWrite(14, HIGH);
    return;
  }

  SDFile = SD.open("data.txt", FILE_WRITE);

  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  rtc.start();
}

void loop1() {
  DateTime now = rtc.now();

  currentMillis = millis();
  accel.readSensor();
  gyro.readSensor();
  
  gxS = (1.0-0.01)*gxS + 0.01*gyro.getGyroX_raw();
  gyS = (1.0-0.01)*gyS + 0.01*gyro.getGyroY_raw();
  gzS = (1.0-0.01)*gzS + 0.01*gyro.getGyroZ_raw();

  gx += (gxS < 0.0006 && gxS > -0.0006 ? 0.00f : gxS) * 2000 * ((currentMillis - lastMillis)/1000);
  gy += (gyS < 0.0006 && gyS > -0.0006 ? 0.00f : gyS) * 2000 * ((currentMillis - lastMillis)/1000);
  gz += (gzS < 0.0006 && gzS > -0.0006 ? 0.00f : gzS) * 2000 * ((currentMillis - lastMillis)/1000);
  
  dateStamp = "[" + String(now.year(),DEC) + "/" + String(now.month(),DEC) + "/" + String(now.day(),DEC) + " (" + daysOfTheWeek[now.dayOfTheWeek()] + ") " + String(now.hour(),DEC) + ":" + String(now.minute(),DEC) + ":" + String(now.second(),DEC) + "]";
  IMU = String(accel.getAccelX_mss()) + "," + String(accel.getAccelY_mss()) + "," + String(accel.getAccelZ_mss()) + "," + String(round(gx)) + "," + String(round(gy)) + "," + String(round(gz)) + "," + String(accel.getTemperature_C());
  BME = String(bme.readTemperature()) + "," + String(bme.readPressure() / 100.0F) + "," + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + "," + String(bme.readHumidity());

  lastMillis = currentMillis;
  
  receivedData = "";

  while (rp2040.fifo.available()) {
    size_t bytesRead = rp2040.fifo.pop();

    receivedData += char(bytesRead);
  }

  if(receivedData != myGPS && receivedData != ""){
    myGPS = receivedData;
  }

  Packet = dateStamp + " " + IMU + "," + BME + "," + myGPS;

  sendMessage(Packet);
  //Serial.println(Packet);

  SDFile.println(Packet);
  SDFile.flush();

  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  byte incomingLength = LoRa.read();     // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}

String ddmmToDD(String raw_gps, char lat){
  if(lat == 'N' || lat == 'E'){
    return String((int(raw_gps.toFloat()/100) + ((((raw_gps.toFloat()/100)-int(raw_gps.toFloat()/100))*100)/60)),6);
  } else {
    return String(-(int(raw_gps.toFloat()/100) + ((((raw_gps.toFloat()/100)-int(raw_gps.toFloat()/100))*100)/60)),6);
  }

  return "-1";
}