#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SparkFunMPL3115A2.h>
#include "TinyGPS++.h"

#define GPSSerial Serial1
#define XBee Serial2

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MPL3115A2 pressure;
TinyGPSPlus gps;

const int chipSelect = 3;
const int currentPin = 4;  
const int buzzerPin = 15;
const int lightPin = A2;
float lightOffset = 0.0;

float accelThreshold = 1.5;
float xOffset = 0, yOffset = 0, zOffset = 0;
float accelData[5][3], accelAverage[] = {0, 0, 0};
int accelCounter = 0, accelTicker = 0;

uint32_t timer1 = millis();
uint32_t timer2 = millis();

File logfile;

void setup()
{ 
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Wire.begin();
  Serial.begin(115200);
  XBee.begin(9600);
  Serial.println("Team Kerbal Systems");

  // Buzzer Setup
  pinMode(buzzerPin, OUTPUT);

  // Light Sensor Setup
  pinMode(lightPin, INPUT);
  delay(1000);
  for (int i = 0; i < 10; i++) {
    float reading = analogRead(lightPin); //Read light level
    lightOffset += reading / 1023.0;      //Get percent of maximum value (1023)
    delay(10);
  }
  lightOffset /= 10.0;
  
  // GPS Setup
  GPSSerial.begin(9600);
  
  // Accelerometer Setup
  accel.begin();
  accel.setRange(ADXL345_RANGE_16_G);
  delay(3000);
  for (int i = 0; i <= 10; i++) {
    sensors_event_t event; 
    accel.getEvent(&event);

    xOffset += event.acceleration.x;
    yOffset += event.acceleration.y;
    zOffset += event.acceleration.z;
    delay(10);
  }
  xOffset /= 10;
  yOffset /= 10;
  zOffset /= 10;

  // Pressure Setup
  pressure.begin();
  pressure.setModeAltimeter();
  pressure.setOversampleRate(7); // Set Oversample to the recommended 128
  pressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  // SD Card Setup
  if (!SD.begin(chipSelect)) {
    Serial.println("Card init. failed!");
  }
  char filename[15];
  strcpy(filename, "DATA00.TXT");  
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
  }
  
  delay(1000);
}

void buzzerControl(float accX, float accY, float accZ)
{
  accelData[accelCounter][0] = accX;
  accelData[accelCounter][1] = accY;
  accelData[accelCounter][2] = accZ;
  
  if (accelCounter == 4) {
    for (int i = 0; i <= 4; i++) {
      accelAverage[0] += accelData[i][0];
      accelAverage[1] += accelData[i][1];
      accelAverage[2] += accelData[i][2];
    }

    accelAverage[0] /= 5;
    accelAverage[1] /= 5;
    accelAverage[2] /= 5;

    accelCounter = 0;

    if (abs(accelAverage[0]) < accelThreshold && abs(accelAverage[1]) < accelThreshold && abs(accelAverage[2]) < accelThreshold) {
      accelTicker++;
    } else {
      accelTicker = 0;
    }
  }
  accelCounter++;

  if (accelTicker >= 20)
    digitalWrite(buzzerPin, HIGH);
  else if (digitalRead(buzzerPin) == HIGH)
    digitalWrite(buzzerPin, LOW);
}

String getGPSString() 
{
  String data = "";
  
    if (gps.location.isValid()) {
      data = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "." + String(gps.time.centisecond());
      data += "," + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.speed.knots()) + "," + String(gps.altitude.feet()) + "," + String(gps.satellites.value());
    
      return data; 
    }
}

String getAccelString()
{
  sensors_event_t event; 
  accel.getEvent(&event);

  String data = String(event.acceleration.x - xOffset) + "," + String(event.acceleration.y - yOffset) + "," + String(event.acceleration.z - zOffset);
  
  buzzerControl(event.acceleration.x - xOffset, event.acceleration.y - yOffset, event.acceleration.z - zOffset);
  
  return data;
}

String getPressureString()
{
  String data = String(pressure.readTempF()) + "," + String(pressure.readAltitudeFt());

  return data;
}

String getCurrentString()
{
  float value = (analogRead(currentPin) * 3.3 / 1023) / (10 * 10);
  return String(analogRead(currentPin));
}

String getLightString()
{
  float reading = analogRead(lightPin); //Read light level
  reading /= 1023.0;      //Get percent of maximum value (1023)
  reading = lightOffset - reading;

  if ((reading - lightOffset) > 0.02)
    return "1";

   return "0";
}

void loop() // run over and over again
{  
  while (GPSSerial.available() > 0) {
    // if millis() or timer wraps around, we'll just reset it
    if (timer1 > millis()) timer1 = millis();
    if (timer2 > millis()) timer2 = millis();
       
    // approximately every 100ms, print out the current stats
    String gpsString = "";
    if (gps.encode(GPSSerial.read())) {
      gpsString = getGPSString();
    }
      
    if (millis() - timer1 > 100) {
      timer1 = millis(); // reset the timer
      
      String accelString = getAccelString();
      String pressureString = getPressureString();
      String currentString = getCurrentString();
      String lightString = getLightString();
  
      String packet = currentString + "," + lightString + "," + pressureString + "," + accelString + "," + gpsString;
      logfile.println(packet);
      logfile.flush();

      XBee.println(packet);
    }
  }
}
