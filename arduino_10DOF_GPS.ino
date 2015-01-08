#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

uint8_t zero_pin = 3;

#define GPSECHO  false

sensors_event_t accel_event, mag_event, bmp_event;
sensors_vec_t   orientation;

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

uint32_t timer, timer1 = 0;
uint8_t count = 0;
float zero_pitch = 0;
float zero_roll = 0;
float last_pitch = 0;
float last_roll = 0;
//uint8_t zero_flag = 0;
uint32_t zero_timer = 0;
uint8_t gps_good = 0;
float temp_lat, temp_long;

void setup()
{
  initSensors();

  pinMode(zero_pin, 0);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  //Comm Link between Arduino and Linux
  Serial1.begin(9600);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  timer = millis();
}

void loop()                     // run over and over again
{
	zero_timer = millis();
	while(true)
	{
		if(!digitalRead(zero_pin))
		{
			if(millis() - zero_timer > 3000)
			{
				Serial1.println("Pitch, Roll Zeroed");
				//accel.getEvent(&accel_event);
				zero_pitch = last_pitch;
				zero_roll = last_roll;
				//return;
			}
			//Serial1.println(millis() - zero_timer);
			delay(100);
		}
		else
		{
			break;
		}
	}

	if(!gps_good)
	{
		char c = GPS.read();
		if((c)&&(GPS.newNMEAreceived()))
		{
			if (GPS.parse(GPS.lastNMEA()))   gps_good = 1;
		}
	}

	timer1 = millis()-timer;
	//if((millis()-timer) > 250)
	if(timer1 > 250)
	{
		timer = millis();
		// Calculate pitch and roll from the raw accelerometer data
		accel.getEvent(&accel_event);
		dof.accelGetOrientation(&accel_event, &orientation);
		// Calculate the heading using the magnetometer
		mag.getEvent(&mag_event);
		dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
		last_pitch = orientation.pitch;
		last_roll = orientation.roll;

//		String att = "XATTNflight,";
//		att += String(orientation.heading);
//		att += ",";
//		att += String(orientation.pitch - zero_pitch);
//		att += ",";
//		att += String(orientation.roll - zero_roll);
//	    Serial1.println(att);
		Serial1.print("XATTNflight,");
		Serial1.print(orientation.heading,1);
		Serial1.print(",");
		Serial1.print(orientation.pitch - zero_pitch,1);
		Serial1.print(",");
		Serial1.println(orientation.roll - zero_roll,1);

	    //Serial1.println(timer1);

	    if(count == 3)
	    //if(false)
	    {
	    	count = 0;
	    	gps_good = 0;
//	    	String gps = "XGPSNflight,";
//	    	gps += String(GPS.longitude);
//	    	gps += ",";
//	    	gps += String(GPS.latitude);
//	    	gps += ",";
//	    	gps += String(GPS.altitude);
//	    	gps += ",";
//	    	gps += String(GPS.angle);
//	    	gps += ",";
//	    	gps += String(GPS.speed * 0.51444);
//	    	Serial1.println(gps);
	    	if(GPS.lat == 'N') temp_lat = GPS.latitude;
	    	else temp_lat = GPS.latitude * -1;

	    	if(GPS.lon == 'E') temp_long = GPS.longitude;
			else temp_long = GPS.longitude * -1;

	    	Serial1.print("XGPSNflight,");
	    	Serial1.print(temp_long, 4);
	    	Serial1.print(",");
	    	Serial1.print(temp_lat, 4);
	    	Serial1.print(",");
	    	Serial1.print(GPS.altitude);
	    	Serial1.print(",");
	    	Serial1.print(GPS.angle);
	    	Serial1.print(",");
	    	Serial1.println(GPS.speed * 0.51444);
	    }
	    else
	    {
	    	count = count + 1;
	    }
	}
}

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  /*if(!bmp.begin())
  {
    // There was a problem detecting the BMP180 ... check your connections
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }*/
}


