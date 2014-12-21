#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Bridge.h>
#include <Process.h>

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

sensors_event_t accel_event, mag_event, bmp_event;
sensors_vec_t   orientation;

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
Process p;

void setup()  
{
  Bridge.begin();
  
  initSensors();

  Serial.begin(115200);
  //delay(5000);
  //Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  String gps = "XGPSNflight,";
  String att = "XATTNflight,";
  
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO)) Serial.write(c); 
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA()))   return;  
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000)
  { 
    timer = millis(); // reset the timer
    
    gps += String(GPS.longitude);
    gps += ",";
    gps += String(GPS.latitude);
    gps += ",";
    gps += String(GPS.altitude);
    gps += ",";
    gps += String(GPS.angle);
    gps += ",";
    gps += String(GPS.speed * 0.51444);
    Serial.println(gps);
    
    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      /* 'orientation' should have valid .roll and .pitch fields 
      Serial.print(F("Roll: "));
      Serial.print(orientation.roll);
      Serial.print(F("; "));
      Serial.print(F("Pitch: "));
      Serial.print(orientation.pitch);
      Serial.println(F("; "));*/
    }
    
    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
      att += String(orientation.heading);
      att += ",";
      att += String(orientation.pitch);
      att += ",";
      att += String(orientation.roll);
      /* 'orientation' should have valid .heading data now 
      Serial.print(F("Heading: "));
      Serial.print(orientation.heading);
      Serial.print(F("; "));*/
    }
    
    /* Calculate the altitude using the barometric pressure sensor */
    //bmp.getEvent(&bmp_event);
    //if (bmp_event.pressure)
    //{
      /* Get ambient temperature in C 
      float temperature;
      bmp.getTemperature(&temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude    
      Serial.print(F("Alt: "));
      Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                          bmp_event.pressure,
                                          temperature)); 
      Serial.print(F(" m; "));
      /* Display the temperature 
      Serial.print(F("Temp: "));
      Serial.print(temperature);
      Serial.print(F(" C"));*/
    //}
    Serial.println(att);
  }
  p.begin("python");
  p.addParameter("/mnt/sda1/arduino/udp_write.py");
  p.addParameter(gps);
  p.run();
}

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  /*if(!bmp.begin())
  {
    // There was a problem detecting the BMP180 ... check your connections
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }*/
}


