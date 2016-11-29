#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SparkFunMPL3115A2.h>

#define GPSSerial Serial1

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MPL3115A2 pressure;
Adafruit_GPS GPS(&GPSSerial);

const int chipSelect = 4;
const int currentPin = 5;  
const int buzzerPin = 15;
const int lightPin = A2;
float lightOffset = 0.0;

uint32_t timer = millis();

File logfile;

void setup()
{ 
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Wire.begin();
  Serial.begin(115200);
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
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  
  // Accelerometer Setup
  accel.begin();
  accel.setRange(ADXL345_RANGE_16_G);

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
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

String getGPSString() 
{
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return (char *)"\0"; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  String data = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "." + String(GPS.milliseconds) + "," + String(GPS.fix) + "," + String(GPS.fixquality);
  
  if (GPS.fix)
  {
    data += "," + String(GPS.latitude) + "," + String(GPS.longitude) + "," + String(GPS.speed) + "," + String(GPS.altitude) + "," + String(GPS.satellites);
  }
  
  return data;
}

String getAccelString()
{
  sensors_event_t event; 
  accel.getEvent(&event);

  String data = String(event.acceleration.x) + "," + String(event.acceleration.y) + "," + String(event.acceleration.z);
  
  return data;
}

String getPressureString()
{
  String data = String(pressure.readAltitudeFt()) + "," + String(pressure.readTempF());

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
  Serial.println(reading);

  if ((reading - lightOffset) > 0.02)
    return "1";

   return "0";
}

void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  GPS.read();
  
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 200ms, print out the current stats
  if (millis() - timer > 200) {
    if (digitalRead(buzzerPin) == HIGH)
      digitalWrite(buzzerPin, LOW);
    else
      digitalWrite(buzzerPin, HIGH);
    timer = millis(); // reset the timer
    String gpsString = getGPSString();
    String accelString = getAccelString();
    String pressureString = getPressureString();
    String currentString = getCurrentString();
    String lightString = getLightString();

    logfile.println(currentString + "," + lightString + "," + pressureString + "," + accelString + "," + gpsString);
  }
}
