//#include <LedControl.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>
#include "Wire.h"
#include "ADXL345.h"
#include <DHT.h>


TinyGPS gps;
SoftwareSerial nss(6, 7);
static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_date(TinyGPS &gps);


File myFile;

#define SDLed         9
#define GPSLed        8
#define FIXLed        A0
#define SaveButton    5
#define CustomButton  A1

int Counter = 0;
int SaveCounter = 3;      // (For saving every 3 lines)

boolean Status = true;

long PrevMillis = 0;
long UpdateInterval = 30000;


DHT dht(3, DHT11);
float Temp = 0;



ADXL345 accel;

double xyz[3];
double CalibratedAccelXYZ[3];
double currentAccelX, currentAccelY, currentAccelZ;
double ThisSecondX, ThisSecondY, ThisSecondZ = 0;



//LedControl lc=LedControl(12,11,10,1);


void setup()
{
  nss.begin(9600);
  pinMode(SDLed, OUTPUT);
  pinMode(GPSLed, OUTPUT);
  pinMode(SaveButton, INPUT_PULLUP);
  pinMode(CustomButton, INPUT_PULLUP);
  pinMode(FIXLed, OUTPUT);
  digitalWrite(SDLed, LOW);
  //Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);
  if (!SD.begin(4)) 
  {
    //Serial.println("initialization failed!");
    return;
  }
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("log.csv", FILE_WRITE);
  if (myFile)
  {
    // Turn on the SD Led if everything is OK
    digitalWrite(SDLed, HIGH);
    // Write the header explanation on the log file
    myFile.println("Time,Date,Satellites,Latitude,Longitude,Altitude,Speed,Course,Temperature,AccelerationX,AccelerationY,AccelerationZ");
  }
  else
  {
    // Turn of the SD Led to indicate problem
    digitalWrite(SDLed, LOW);
  }
  // Start Thermometer
  dht.begin();
  delay(500);
  Temp = dht.readTemperature();
  // Start Accelerometer
  accel.powerOn();
  delay(100);
  accel.setFullResBit(1);
  accel.setRangeSetting(16);
  accel.get_Gxyz(xyz); 
  // Get default values to remove earth acceleration
  CalibratedAccelXYZ[0] = xyz[0];
  CalibratedAccelXYZ[1] = xyz[1];
  CalibratedAccelXYZ[2] = xyz[2];
  digitalWrite(FIXLed, LOW);
  PrevMillis = millis();
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
    // Accelerometer Reading
    accel.get_Gxyz(xyz);       // Read Acceleration in G on X, Y, Z
    currentAccelX=abs(xyz[0]-CalibratedAccelXYZ[0]);
    currentAccelY=abs(xyz[1]-CalibratedAccelXYZ[1]);
    currentAccelZ=abs(xyz[2]-CalibratedAccelXYZ[2]);
    if (currentAccelX>ThisSecondX)
      ThisSecondX = currentAccelX;
    if (currentAccelY>ThisSecondY)
      ThisSecondY = currentAccelY;
    if (currentAccelZ>ThisSecondZ)
      ThisSecondZ = currentAccelZ;
    // Temperature Check
    if (millis() - PrevMillis > UpdateInterval)
    {
      Temp = dht.readTemperature();
      PrevMillis = millis();
    }
  }
  // Write data from GPS to SD
  gpsdump(gps);
  // Reset max values of acceleration axis
  ThisSecondX = 0;
  ThisSecondY = 0;
  ThisSecondZ = 0;
  
  // Every X seconds (SaveCounter) save data on SD (flush)
  Counter++;
  if (Counter>SaveCounter)
  {
    myFile.flush();
    Counter=0;
  }
}



void CloseFile()
{
  while (digitalRead(SaveButton)==LOW)
  {
    // do nothing
  }
  myFile.flush();
  myFile.close();
  digitalWrite(SDLed, LOW);
  digitalWrite(GPSLed, LOW);
  //Serial.println("File Saved. Terminated.");
  while (true)
  {
    if (digitalRead(SaveButton)==LOW)
    {      digitalWrite(SDLed, LOW);
      digitalWrite(GPSLed, LOW);
      digitalWrite(FIXLed, LOW);
      void(* resetFunc) (void) = 0; 
      resetFunc();
    }
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
  // Blink the GPS Led (1 second on, 1 second off)
  if (Status)
     digitalWrite(GPSLed, HIGH);
  else
     digitalWrite(GPSLed, LOW);
  Status = !Status;

  // Get GPS Satellites. If less than 4, means no fix, so return
  int Sats = gps.satellites();
  //Serial.println(Sats);

// ---------  Allos tropos gia elegxo gpsp lock -----------
//  if (fix_age == TinyGPS::GPS_INVALID_AGE)
//  {
//    digitalWrite(FIXLed, LOW);
//    return;
//  }



  if ((Sats>=4) && (Sats<255))
    digitalWrite(FIXLed, HIGH);
  else
  {
    digitalWrite(FIXLed, LOW);
    return;
  }
  // Start writing data to SD  
  float flat, flon;
  unsigned long age, date, time, chars = 0;
//  unsigned short sentences = 0, failed = 0;
  print_date(gps);
  myFile.print(",");
  myFile.print(Sats);
  myFile.print(",");
  gps.f_get_position(&flat, &flon, &age);
  myFile.print(flat,9);
  myFile.print(",");
  myFile.print(flon,9);
  myFile.print(",");
  myFile.print(gps.f_altitude());
  myFile.print(",");
  myFile.print(gps.f_speed_kmph());
  myFile.print(",");
  myFile.print(gps.f_course());  
  myFile.print(",");
  myFile.print(Temp,2);
  myFile.print(",");
  myFile.print(ThisSecondX);
  myFile.print(",");
  myFile.print(ThisSecondY);
  myFile.print(",");
  myFile.println(ThisSecondZ);
}



static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
  {
    myFile.print("00:00:00,00/00/00");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d,%02d/%02d/%02d",
        hour, minute, second, day, month, year);
    //Serial.print(sz);
    myFile.print(sz);
  }
}


static bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}
