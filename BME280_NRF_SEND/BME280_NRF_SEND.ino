/* YourDuinoStarter Example: Simple nRF24L01 Transmit
  - WHAT IT DOES: Transmits simple fixed data with nRF24L01 radio
  - SEE the comments after "//" on each line below
   Start with radios about 4 feet apart.
  - SEE the comments after "//" on each line below
  - CONNECTIONS: nRF24L01 Modules See:
  http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
  Uses the RF24 Library by TMRH2o here:
  https://github.com/TMRh20/RF24
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED

   V1.02 02/06/2016
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)

#include <Wire.h>
#include "cactus_io_BME280_I2C.h"
/*-----( Declare Constants and Pin Numbers )-----*/
#define SEALEVELPRESSURE_HPA (1013.25)
//None yet
/*-----( Declare objects )-----*/
// Create the BME280 object
//BME280_I2C bme;              // I2C using default 0x77
BME280_I2C bme(0x76);  // I2C using address 0x76
// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (7, 8); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.

struct dataStruct {
 int channel=2;
 float ReelPressure, ReelHumidity, ReelTemperature;
} myData;                 // This can be accessed in the form:  myData.Xposition  etc.


void setup()   /****** SETUP: RUNS ONCE ******/
{
  // Use the serial Monitor (Symbol on far right). Set speed to 115200 (Bottom Right)
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("RF24/Simple Transmit data Test"));
  Serial.println(F("Questions: terry@yourduino.com"));
  myRadio.begin();  // Start up the physical nRF24L01 Radio
  myRadio.setChannel(108);  // Above most Wifi Channels
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  //myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setPALevel(RF24_PA_MAX);  // Uncomment for more power

  myRadio.openWritingPipe( addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  Serial.println("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  bme.setTempCal(-1);

  Serial.println("Pressure\tHumdity\t\tTemp\t\tTemp");
  delay(1000);
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  bme.readSensor();
  myData.ReelPressure = bme.getPressure_MB() + 5;
  myData.ReelHumidity = bme.getHumidity() + 4;
  myData.ReelTemperature = bme.getTemperature_C();
 
  myRadio.write( &myData, sizeof(myData) );

  delay(2000);

}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/


//*********( THE END )***********

