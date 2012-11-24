#include "Wire.h"
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "ADXL345.h"

TinyGPS gps;
SoftwareSerial GPSSerial(6, 7);
static void gpsdump(TinyGPS &gps);
static bool feedgps();

File LogFile;

#define SDLed         9
#define GPSLed        8
#define FIXLed        A0
#define SaveButton    5
#define BuzzerPin     2

long PrevMillis = 0;
long startTime;          // For measuring the GPS Fix time

ADXL345 accel;
double xyz[3];
double CalibratedAccelXYZ[3];
double ThisSecondX, ThisSecondY, ThisSecondZ = 0;
int CalibratedAngleX, CalibratedAngleY;
int fooX, fooY, fooZ;
int ThisSecondAngleX = 0;
int ThisSecondAngleY = 0;

boolean beeps = true;


void setup()
{
  GPSSerial.begin(9600);
  pinMode(SDLed, OUTPUT);
  pinMode(GPSLed, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(SaveButton, INPUT_PULLUP);
//  pinMode(CustomButton, INPUT_PULLUP);
  pinMode(FIXLed, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
  digitalWrite(SDLed, LOW);
  digitalWrite(GPSLed, LOW);
  digitalWrite(FIXLed, LOW);
  GPSSerial.println("$PMTK301,2*2E");              // DGPS = WAAS
  delay(1000);
  GPSSerial.println("$PMTK397,0.4*39");            // Do not count speed less than 0,4m/s (1.4km/h)
  delay(1000);
  // ADXL345 Start
  accel.powerOn();
  delay(100);
  accel.setFullResBit(1);
  accel.setRangeSetting(16);
  accel.get_Gxyz(xyz); 
  // Get default values to remove earth acceleration
  CalibratedAccelXYZ[0] = xyz[0];
  CalibratedAccelXYZ[1] = xyz[1];
  CalibratedAccelXYZ[2] = xyz[2];
  
  // Get default angle
  accel.readAccel(&fooX, &fooY, &fooZ);
  CalibratedAngleX = (atan2(fooY,fooZ)+PI)*RAD_TO_DEG;
  CalibratedAngleY = (atan2(fooX,fooZ)+PI)*RAD_TO_DEG;
  // Start SD Card
  pinMode(10, OUTPUT);
  if (!SD.begin(4)) 
  {
    // Oops, Error. No SD card?
    return;
  }
  // open the file. 
  LogFile = SD.open("GPSLOG.CSV", FILE_WRITE);
  if (LogFile)
  {
    delay(100);
    // Write the header -- explanation of the log file
    LogFile.println("");
    LogFile.println(":: GPS/Acceleration Logger ::");
    LogFile.flush();
    LogFile.println(" (c)2012 - Antonis Maglaras");
    LogFile.println("");
    LogFile.flush();
    LogFile.print("Default Acceleration on X: ");
    LogFile.print(xyz[0],2);
    LogFile.print(" - Y: ");
    LogFile.print(xyz[1],2);
    LogFile.print(" - Z: ");
    LogFile.println(xyz[2],2);
    LogFile.flush();
    LogFile.print("Default Angle on X: ");
    LogFile.print(CalibratedAngleX);
    LogFile.print(" - Y: ");
    LogFile.println(CalibratedAngleY);
    LogFile.flush();
    // Turn on the SD Led if everything is OK
    digitalWrite(SDLed, HIGH);
  }
  digitalWrite(FIXLed, LOW);
  buzzer();  
  beeps = true;
  PrevMillis = millis();
  startTime = millis();
}


void loop()
{
  bool newdata = false;
  unsigned long Start = millis();
  // Every second we print an update
  while (millis() - Start < 1000)
  {
    if (feedgps())
      newdata = true;
   // If the button is pressed, Save the log and halt
    if (digitalRead(SaveButton)==LOW)
      CloseFile();
    // Accelerometer Reading -- G forces on X, Y, Z
    accel.get_Gxyz(xyz);
    if ((xyz[0]-CalibratedAccelXYZ[0])>ThisSecondX)
      ThisSecondX = xyz[0]-CalibratedAccelXYZ[0];
    if ((xyz[1]-CalibratedAccelXYZ[1])>ThisSecondY)
      ThisSecondY = xyz[1]-CalibratedAccelXYZ[1];
    if ((xyz[2]-CalibratedAccelXYZ[2])>ThisSecondZ)
      ThisSecondZ = xyz[2]-CalibratedAccelXYZ[2];
    // Accelerometer Reading -- Raw values for leaning calculations
    accel.readAccel(&fooX, &fooY, &fooZ);
    if ((CalibratedAngleX - ((atan2(fooY,fooZ)+PI)*RAD_TO_DEG))>ThisSecondAngleX)
      ThisSecondAngleX = (CalibratedAngleX - ((atan2(fooY,fooZ)+PI)*RAD_TO_DEG));
    if ((CalibratedAngleY - ((atan2(fooX,fooZ)+PI)*RAD_TO_DEG))>ThisSecondY)
      ThisSecondAngleY = (CalibratedAngleY - ((atan2(fooX,fooZ)+PI)*RAD_TO_DEG));
  }
  
  // Process data from GPS -- Write them on SD card
  gpsdump(gps);
  
  // Reset max values
  ThisSecondX = 0;
  ThisSecondY = 0;
  ThisSecondZ = 0;
  ThisSecondAngleX = 0;
  ThisSecondAngleY = 0;
}


void CloseFile()
{
  LogFile.println("");
  LogFile.flush();
  delay(100);
  LogFile.close();
  delay(100);
  while (true)
  {
    digitalWrite(SDLed, HIGH);
    digitalWrite(GPSLed, LOW);
    delay(100);
    digitalWrite(SDLed, LOW);
    digitalWrite(GPSLed, HIGH);
    delay(100);
  }
}


static void gpsdump(TinyGPS &gps)
{
  // Get GPS Satellites. If less than 4, means no fix, so return
  int Sats = gps.satellites();
  if (Sats<255)
  {
    if (beeps == true)
    {
      // Write on file the GPS Fix time.
      LogFile.print("GPS Fix: ");
      LogFile.print((millis() - startTime)/1000);
      LogFile.println(" seconds.");
      LogFile.flush();
      LogFile.println("");
      LogFile.println("Time,Date,Satellites,Latitude,Longitude,Altitude,Speed,Course,AccelerationX,AccelerationY,AccelerationZ,AngleX,AngleY");
      LogFile.flush();
      // Beep when GPS locks for first time.
      buzzer();
      beeps=false;
    }
    digitalWrite(FIXLed, HIGH);
  }
  else
  {
    digitalWrite(FIXLed, LOW);
    return;
  }
  
  // Start writing data from GPS to SD  
  float flat, flon;
  unsigned long age;
  int year;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
  {
    LogFile.print("00:00:00,00/00/00");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d,%02d/%02d/%02d",
        hour, minute, second, day, month, year);
    LogFile.print(sz);
  }
  LogFile.print(",");
  LogFile.print(Sats);
  LogFile.print(",");
  gps.f_get_position(&flat, &flon, &age);
  LogFile.print(flat,7);
  LogFile.print(",");
  LogFile.print(flon,7);
  LogFile.print(",");
  LogFile.print(gps.f_altitude(),2);
  LogFile.print(",");
  LogFile.print(gps.f_speed_kmph(),2);
  LogFile.print(",");
  LogFile.print(gps.f_course());  
  LogFile.print(",");
  LogFile.print(ThisSecondX,1);
  LogFile.print(",");
  LogFile.print(ThisSecondY,1);
  LogFile.print(",");
  LogFile.print(ThisSecondZ,1);
  LogFile.print(",");
  LogFile.print(ThisSecondAngleX);
  LogFile.print(",");
  LogFile.println(ThisSecondAngleY);
  LogFile.flush();  
}


static bool feedgps()
{
  while (GPSSerial.available())
  {
    digitalWrite(GPSLed, HIGH);              // Turn on GPSLed to indicate receiving of data
    if (gps.encode(GPSSerial.read()))
    {
      digitalWrite(GPSLed, LOW);            // Turn off GPSLed when are OK
      return true;
    }
  }
  digitalWrite(GPSLed, LOW);                // Turn off GPSLed when not receiving data
  return false;
}


void buzzer()
{
  digitalWrite(BuzzerPin, HIGH);
  delay(50);
  digitalWrite(BuzzerPin, LOW);
  delay(50);
}
