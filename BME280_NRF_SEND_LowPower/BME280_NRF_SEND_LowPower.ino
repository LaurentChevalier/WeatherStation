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
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED

   V1.02 02/06/2016
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

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
RF24 myRadio (7, 10); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.
volatile int f_wdt = 1;
int i = 0;

struct dataStruct {
  int channel = 2; //1 for outdoor //2for greenhouse
  float ReelPressure, ReelHumidity, ReelTemperature;
  long BatteryVoltage;
} myData;                 // This can be accessed in the form:  myData.Xposition  etc.

/***************************************************
    Name:        ISR(WDT_vect)

    Returns:     Nothing.

    Parameters:  None.

    Description: Watchdog Interrupt Service. This
                 is executed when watchdog timed out.

 ***************************************************/
ISR(WDT_vect)
{
  if (f_wdt == 0)
  {
    f_wdt = 1;
  }
  else
  {
    Serial.println("WDT Overrun!!!");
  }
}


/***************************************************
    Name:        enterSleep

    Returns:     Nothing.

    Parameters:  None.

    Description: Enters the arduino into sleep mode.

 ***************************************************/
void enterSleep(void)
{
  myRadio.powerDown();
  //set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);/* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
  }



void setup()   /****** SETUP: RUNS ONCE ******/
{
  // Use the serial Monitor (Symbol on far right). Set speed to 115200 (Bottom Right)
  Serial.begin(115200);
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

  /*** Setup the WDT ***/

  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);

  /* In order to change WDE or the prescaler, we need to
     set WDCE (This will allow updates for 4 clock cycles).
  */
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  if (f_wdt == 1)
  {
    if (i == 0) {
      myRadio.powerUp();
      myData.BatteryVoltage=readVcc();
      bme.readSensor();
      myData.ReelPressure = bme.getPressure_MB() + 5;
      myData.ReelHumidity = bme.getHumidity() + 4;
      myData.ReelTemperature = bme.getTemperature_C();

      myRadio.write( &myData, sizeof(myData) );
      Serial.print(myData.ReelPressure); Serial.print(" mbar\t");    // Pressure in millibars
      Serial.print(myData.ReelHumidity); Serial.print(" %rH\t");
      Serial.print(bme.getTemperature_C()); Serial.println(" *C\t");

      delay(1000);
    }
    else if (i == 38) {//113 for 15min or 38 or 5min
      myRadio.powerUp();
      myData.BatteryVoltage=readVcc();
      bme.readSensor();
      myData.ReelPressure = bme.getPressure_MB() + 5;
      myData.ReelHumidity = bme.getHumidity() + 4;
      myData.ReelTemperature = bme.getTemperature_C();

      myRadio.write( &myData, sizeof(myData) );
      Serial.print(myData.ReelPressure); Serial.print(" mbar\t");    // Pressure in millibars
      Serial.print(myData.ReelHumidity); Serial.print(" %rH\t");
      Serial.print(bme.getTemperature_C()); Serial.println(" *C\t");

      delay(1000);
      i=0;
    }
    i++;
    /* Don't forget to clear the flag. */
    f_wdt = 0;

    /* Re-enter sleep mode. */
    enterSleep();
  }
  else
  {

  }

}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//*********( THE END )***********

