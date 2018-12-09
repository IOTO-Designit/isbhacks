#include <SD.h> 
#include <Wire.h>
#include "RTClib.h" 
#include <SPI.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <BH1750FVI.h>
#include "I2Cdev.h"
#include "HMC5883L.h"
#define BT_SERIAL_TX 1
#define BT_SERIAL_RX 0
SoftwareSerial BluetoothSerial(BT_SERIAL_TX, BT_SERIAL_RX);
#define DHTPIN1 2 
#define DHTTYPE DHT22
DHT dht1(DHTPIN1, DHTTYPE);
Adafruit_BMP280 bmp;
BH1750FVI LightSensor;
HMC5883L mag;
int16_t mx, my, mz;
int uvPin = A1;
const int CS_PIN=10;
float pressure;
float temperature;
float altimeter;
RTC_DS1307 RTC;
String year, month, day, hour, minute, second, time, date;
void setup()
{ Serial.begin(9600);
BluetoothSerial.begin(9600); 
BluetoothSerial.print("SSp BT MC C S");
//lcd.begin(16,4);
Wire.begin();
RTC.begin();  
pinMode(CS_PIN, OUTPUT);
Serial.println(F("SSp BT MC C S"));
LightSensor.begin();
LightSensor.SetAddress(Device_Address_H);
LightSensor.SetMode(Continuous_H_resolution_Mode);
mag.initialize();
bmp.begin();
Wire.endTransmission();
if (! RTC.isrunning())
{
Serial.println(F("Clock is NOT running!"));
RTC.adjust(DateTime(__DATE__, __TIME__));
}
if (!SD.begin(CS_PIN))
{
Serial.println(F("Card is NOT Installed properly"));
return;
}
Serial.println(F("System Check OK!"));
File dataFile = SD.open("log.csv", FILE_WRITE);
if (dataFile)
{
dataFile.println(F("\nNew Recording Started"));
dataFile.println(F("Date,Time,RH,T,PST,PSP,ALT,Lux,UV,MX,MY,MZ,RG"));
dataFile.close(); 
Serial.println(F("\nNew Recording Started"));
Serial.println(F("Date,Time,RH,T,PST,PSP,ALT,Lux,UV,MX,MY,MZ,RG"));
}
else
{
Serial.println(F("Card Not Installaed"));
Serial.println(" ");
}
}
void loop()
{
DateTime datetime = RTC.now();
year = String(datetime.year(),DEC);
month = String(datetime.month(),DEC);
day = String(datetime.day(),DEC);
hour = String(datetime.hour(),DEC);
minute = String(datetime.minute(),DEC);
second = String(datetime.second(),DEC);
date = day + "/" + month + "/" + year ;
time = hour + ":" + minute + ":" + second;
Serial.println(" "); 
float h1 = dht1.readHumidity();
float t1 = dht1.readTemperature();
pressure = bmp.readPressure();
temperature = bmp.readTemperature();
altimeter = bmp.readAltitude (1013.35); //Change the "1050.35" to your city current barrometric pressure
uint16_t lux = LightSensor.GetLightIntensity();
mag.getHeading(&mx,&my,&mz);
float heading = atan2(my,mx);
if(heading < 0)
heading += 2 * M_PI;
float uv = analogRead(uvPin);
uv = uv * 0.0049; //convert values to volts
uv = uv * 307; //convert to mW/m²
uv = uv/200; //calculate UV index
File dataFile = SD.open("log.csv", FILE_WRITE);
if (dataFile)
{
dataFile.print(date);
dataFile.print(F(","));
dataFile.print(time);
dataFile.print(F(","));
dataFile.print(h1);
dataFile.print(F(","));
dataFile.print(t1);
dataFile.print(F(","));
dataFile.print(bmp.readTemperature());
dataFile.print(F(","));
dataFile.print(bmp.readPressure());
dataFile.print(F(","));
dataFile.print(bmp.readAltitude(1013.25));
dataFile.print(F(","));
dataFile.print(lux);
dataFile.print(F(","));
dataFile.print(uv);
dataFile.print(F(","));
dataFile.print(mx);
dataFile.print(F(","));
dataFile.print(my);
dataFile.print(F(","));
dataFile.print(mz);
dataFile.print(F(","));
dataFile.print(heading * 180/M_PI);
dataFile.println();
dataFile.close();
}
else
{
Serial.print (F("LOG file not available"));
Serial.println(" ");
}
delay(10000);
}
