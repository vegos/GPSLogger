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
#define DHTPin        3
#define BuzzerPin     2

int Counter = 0;
int SaveCounter = 3;      // (For saving every 3 loop times

long PrevMillis = 0;
long UpdateInterval = 30000;
long GPSUpdate = 200;   // 1000 for 1Hz, 200 for 5Hz


DHT dht(DHTPin, DHT11);
float Temp = 0;



ADXL345 accel;


double xyz[3];
double CalibratedAccelXYZ[3];
double currentAccelX, currentAccelY, currentAccelZ;
double ThisSecondX, ThisSecondY, ThisSecondZ = 0;

int CalibratedAngleX, CalibratedAngleY;
int CurrentAngleX, CurrentAngleY;
int fooX, fooY, fooZ;
int ThisSecondAngleX = 0;
int ThisSecondAngleY = 0;

boolean State = false;    // For blinking led when sats are just 4.
boolean beeps = true;



void setup()
{
  nss.begin(9600);
  pinMode(SDLed, OUTPUT);
  pinMode(GPSLed, OUTPUT);
  pinMode(BuzzerPin, OUTPUT);
  buzzer();
  pinMode(SaveButton, INPUT_PULLUP);
  pinMode(CustomButton, INPUT_PULLUP);
  pinMode(FIXLed, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
  digitalWrite(SDLed, LOW);
  digitalWrite(GPSLed, HIGH);
  digitalWrite(FIXLed, HIGH);  
  nss.println("$PMTK104*37");                // Perform FULL COLD START
  delay(1000);
  nss.println("$PMTK301,2*2E");              // DGPS = WAAS
  delay(1000);
  nss.println("$PMTK397,0.4*39");            // Do not count speed less than 0,4m/s (1.4km/h)
  delay(1000);
  nss.println("$PMTK251,38400*27");          // Set baud rate to 38400
  delay(1000);
  nss.end();
  nss.begin(38400);
  nss.println("$PMTK220,200*2C");            // Set update rate to 5KHz (0.2 sec)
  delay(1000);
  digitalWrite(FIXLed, LOW);
  digitalWrite(GPSLed, LOW);
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);
  if (!SD.begin(4)) 
  {
    // error, no sd card
    return;
  }
  // open the file. 
  myFile = SD.open("log.csv", FILE_WRITE);
  if (myFile)
  {
    // Turn on the SD Led if everything is OK
    digitalWrite(SDLed, HIGH);
    // Write the header explanation on the log file
    myFile.println("Time,Date,Satellites,Latitude,Longitude,Altitude,Speed,Course,Temperature,AccelerationX,AccelerationY,AccelerationZ,AngleX,AngleY");
  }
  // Start Thermometer
  dht.begin();
  delay(500);
  Temp = dht.readTemperature();
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
  CalibratedAngleX = (atan2(fooY,fooX)+PI)*RAD_TO_DEG;
  CalibratedAngleY = (atan2(fooX,fooZ)+PI)*RAD_TO_DEG;
  digitalWrite(FIXLed, LOW);
  for (int z=0; z<3; z++)
    buzzer();  
  beeps = true;
  PrevMillis = millis();
}


void loop()
{
  bool newdata = false;
  unsigned long Start = millis();
  // Every second we print an update
  while (millis() - Start < GPSUpdate)
  {
    if (feedgps())
      newdata = true;
   // If the button is pressed, Save the log and halt
    if (digitalRead(SaveButton)==LOW)
      CloseFile();
    // Accelerometer Reading
    accel.get_Gxyz(xyz);       // Read Acceleration in G on X, Y, Z
    currentAccelX=xyz[0]-CalibratedAccelXYZ[0];
    currentAccelY=xyz[1]-CalibratedAccelXYZ[1];
    currentAccelZ=xyz[2]-CalibratedAccelXYZ[2];
    if (currentAccelX>ThisSecondX)
      ThisSecondX = currentAccelX;
    if (currentAccelY>ThisSecondY)
      ThisSecondY = currentAccelY;
    if (currentAccelZ>ThisSecondZ)
      ThisSecondZ = currentAccelZ;
      
    accel.readAccel(&fooX, &fooY, &fooZ);
    CurrentAngleX = CalibratedAngleX - ((atan2(fooY,fooX)+PI)*RAD_TO_DEG);
    CurrentAngleY = CalibratedAngleY - ((atan2(fooX,fooZ)+PI)*RAD_TO_DEG);
    
    if (CurrentAngleX>ThisSecondAngleX)
      ThisSecondAngleX = CurrentAngleX;
    if (CurrentAngleY>ThisSecondY)
      ThisSecondAngleY = CurrentAngleY;
            
    // Temperature Check
    if (millis() - PrevMillis > UpdateInterval)
    {
      Temp = dht.readTemperature();
      PrevMillis = millis();
    }
  }
  
  // Process Data from GPS
  gpsdump(gps);
  
  // Reset max values of acceleration axis
  ThisSecondX = 0;
  ThisSecondY = 0;
  ThisSecondZ = 0;
  ThisSecondAngleX = 0;
  ThisSecondAngleY = 0;
  
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
  delay(100);
  myFile.close();
  delay(100);
  while (true)
  {
    if (digitalRead(SaveButton)==LOW)
    {      
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
  // Get GPS Satellites. If less than 4, means no fix, so return
  int Sats = gps.satellites();
  if (Sats == 255)
  {
    digitalWrite(FIXLed, LOW);
    return;
  }
  else
  {
    if (beeps)
    {
      // 3 beeps when GPS fix for first time.
      for (int z=0; z<3; z++)
        buzzer();
      beeps=false;
    }
    digitalWrite(FIXLed, HIGH);
  }
  
  // Start writing data from GPS to SD  
  float flat, flon;
  unsigned long age;
  print_date(gps);
  myFile.print(",");
  myFile.print(Sats);
  myFile.print(",");
  gps.f_get_position(&flat, &flon, &age);
  myFile.print(flat,7);
  myFile.print(",");
  myFile.print(flon,7);
  myFile.print(",");
  myFile.print(gps.f_altitude(),2);
  myFile.print(",");
  myFile.print(gps.f_speed_kmph(),2);
  myFile.print(",");
  myFile.print(gps.f_course());  
  myFile.print(",");
  myFile.print(Temp,2);
  myFile.print(",");
  myFile.print(ThisSecondX,1);
  myFile.print(",");
  myFile.print(ThisSecondY,1);
  myFile.print(",");
  myFile.print(ThisSecondZ,1);
  myFile.print(",");
  myFile.print(ThisSecondAngleX);
  myFile.print(",");
  myFile.println(ThisSecondAngleY);
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
    myFile.print(sz);
  }
}


static bool feedgps()
{
  while (nss.available())
  {
    digitalWrite(GPSLed, HIGH);              // Turn on GPSLed to indicate receiving of data
    if (gps.encode(nss.read()))
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
  delay(100);
  digitalWrite(BuzzerPin, LOW);
  delay(100);
}
