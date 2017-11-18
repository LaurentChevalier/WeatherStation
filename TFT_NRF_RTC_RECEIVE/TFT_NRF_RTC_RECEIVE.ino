/*
   Small Program to Simulate a Numpad using a 2.4" TFT Touchscreen
   Program does not act as an USB HID Device, for reference purposes only
   Tested on Arduino UNO Only and 0x9341
   By William Tavares

   Note:
   This version is coplete with styling and numbers,
   if you want the smaller version get the "numpad-layout" program
   from my Github https://github.com/williamtavares/Arduino-Uno-NumPad

   Open the Serial Monitor to see the program running
   Enjoy!
*/

/* YourDuinoStarter Example: Simple nRF24L01 Receive
  - WHAT IT DOES: Receives simple fixed data with nRF24L01 radio
  - SEE the comments after "//" on each line below
   Start with radios about 4 feet apart.
  - SEE the comments after "//" on each line below
  - CONNECTIONS: nRF24L01 Modules See:
  http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
  Uses the RF24 Library by TMRH2o here:
  https://github.com/TMRh20/RF24
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 49
   4 - CSN to Arduino pin 53
   5 - SCK to Arduino pin 52
   6 - MOSI to Arduino pin 51
   7 - MISO to Arduino pin 50
   8 - UNUSED

   V1.02 02/06/2016
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)

#include <dht.h>
dht DHT;
//Constants
#define DHT22_PIN 23     // DHT 22  (AM2302) - what pin we're connected to

#include <Adafruit_GFX.h>
#include <TouchScreen.h>
#include <Adafruit_TFTLCD.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// calibration mins and max for raw data when touching edges of screen
// YOU CAN USE THIS SKETCH TO DETERMINE THE RAW X AND Y OF THE EDGES TO GET YOUR HIGHS AND LOWS FOR X AND Y
#define TS_MINX 210
#define TS_MINY 210
#define TS_MAXX 915
#define TS_MAXY 910

//SPI Communication
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

//Color Definitons
#define BLACK     0x0000
#define BLUE      0x001F
#define GREY      0xCE79
#define LIGHTGREY 0xDEDB
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define MINPRESSURE 1
#define MAXPRESSURE 1000



// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
// Pins A2-A6
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 364);

//Size of key containers 70px
#define BOXSIZE 70

//2.4 = 240 x 320
//Height 319 to fit on screen

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

//Container variables for touch coordinates
int X, Y, Z;

//Screen height without hidden pixel
double tHeight = tft.height() - 1;
//Centering the mid square
double center = (tft.width() / 2) - (BOXSIZE / 2);
//Space between squares
double padding = 10;
//Position of squares to the left and right of center
double fromCenter = BOXSIZE + padding;
//Second row Y-Axis position
double secondRow = BOXSIZE + padding;
//Third row Y-Axis position
double thirdRow = secondRow + BOXSIZE + padding;
//Fourth row Y-Axis position
double fourthRow = thirdRow + BOXSIZE + padding;
//Y-Axis align for all squares
double verticalAlign = (tHeight - ((BOXSIZE * 4) + (padding * 3))) / 2;
//Left column starting x posision
double leftColPositionX = center - fromCenter;
//Mid column starting x posision
double midColPositionX = center;
//Right column starting x posision
double rightColPositionX = center + fromCenter;

long previousMillis = 0;
int interval = 1000;

// VARIABLES TO HOLD COLOR VALUES
int redValue = 0;
int grnValue = 0;
int bluValue = 0;

// ARRAY TO REPLACE SINGLE DIGITS WITH LEADING ZEROES
char* leadingZero[] = {
  "00", "01", "02", "03", "04", "05", "06", "07", "08", "09"
};

/*-----( Declare objects )-----*/
// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (49, 53); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.
float dataReceived;  // Data that will be received from the transmitter

struct dataStruct_outdoor {
  int channel;
  float ReelPressure, ReelHumidity, ReelTemperature;
  long BatteryVoltage;
} myData, myData_Outdoor, myData_Greenhouse; // This can be accessed in the form:  myData.Xposition  etc.

struct dataStruct_indoor {
  float Dht_Humidity, Dht_Temperature;
} myData_Dht;

void setup() {
  Serial.begin(115200);

  myRadio.begin();  // Start up the physical nRF24L01 Radio
  myRadio.setChannel(108);  // Above most Wifi Channels
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // myRadio.setPALevel(RF24_PA_MIN);
  myRadio.setPALevel(RF24_PA_MAX);  // Uncomment for more power

  myRadio.openReadingPipe(1, addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  myRadio.startListening();

  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);
  tft.setRotation(3);

  //Background color
  tft.fillScreen(BLACK);

  // draw num pad
  /*createButtons();
    insertNumbers();
    Serial.println(F("Press any button on the TFT screen: "));
  */

  tft.setCursor(100, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  //tft.println("Hello World");

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust(DateTime(2017, 9, 30, 17, 27, 0));

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2017, 9, 30, 17, 22, 0));
  }

  DateTime now = rtc.now();

}

void loop() {

  float ReelPressure, ReelHumidity, ReelTemperature;
  if ( myRadio.available()) // Check for incoming data from transmitter
  {

    while (myRadio.available())  // While there is data ready
    {
      myRadio.read( &myData, sizeof(myData) ); // Get the data payload (You must have defined that already!)
    }

  } //END Radio available
  // Reading on DHT22
  int chk = DHT.read22(DHT22_PIN);
  myData_Dht.Dht_Humidity = DHT.humidity + 24;
  myData_Dht.Dht_Temperature = DHT.temperature - 4;

  //PERFORMS A TASK EVERY SECOND WITHOUT USING DELAY
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // UPDATES THE SCREEN
    drawTime();
    drawData();
    readVcc();
  }

}


void drawTime() {

  // SETS THE VARIABLE t TO THE CURRENT TIME - PREVENTS OVERLAPPING
  DateTime now = rtc.now();

  //MAPS THE TIME TO CORRESPOND TO COLOR VALUES
  // CONVERTS 24 HOURS TO 0 - 255 STEPS, ETC.
  redValue = map(now.hour(), 0, 23, 0, 255);
  grnValue = map(now.minute(), 0, 59, 0, 255);
  bluValue = map(now.second(), 0, 59, 0, 255);

  //SETS THE TEXT COLOR AND TEXT BACKGROUND COLOR
  //USES THE GFX FUNCTION TO CONVERT THREE INTEGER VALUES TO A HEX COLOR VALUE
  // REPLACE TFT.COLOR565 WITH WHITE OR ANOTHER COLOR IF YOU WANT A STATIC TIME COLOR
  //tft.setTextColor((tft.Color565(redValue, grnValue, bluValue)), BLACK);
  tft.setTextColor(GREEN, BLACK);
  // YOU HAVE TO SET THE TEXT SIZE BEFORE YOU START PRINTING
  tft.setTextSize(3);

  //DRAWS RECTANGLES ABOVE AND BELOW THE BLACK BAR IN THE SAME COLOR
  //tft.fillRect(0, 0, 160, 50, (tft.Color565(redValue, grnValue, bluValue)));
  //tft.fillRect(0, 80, 160, 50, (tft.Color565(redValue, grnValue, bluValue)));

  // LINES UP THE CURSOR AND DRAWS THE HOUR
  // IF THE HOUR IS LESS THAN 10 IT CALLS A VALUE FROM THE LEADING ZERO ARRAY
  tft.setCursor(89, 13);//13
  if (now.hour() >= 0 && now.hour() < 10) {
    tft.print(leadingZero[now.hour()]);
  }
  else {
    tft.print(now.hour());
  }
  // INSERTS THE FIRST DIVIDING COLON
  tft.setCursor(124, 13);//48
  tft.print(":");

  // MOVES THE CURSOR AND DRAWS THE MINUTES, LEADING ZERO, ETC
  tft.setCursor(141, 13); //65
  if (now.minute() >= 0 && now.minute() < 10) {
    tft.print(leadingZero[now.minute()]);
  }
  else {
    tft.print(now.minute());
  }

  // INSERTS THE SECOND COLON
  tft.setCursor(175, 13);//99
  tft.print(":");

  // MOVES THE CURSOR AND DRAWS THE SECONDS, LEADING ZERO, ETC
  tft.setCursor(191, 13);  //115
  if (now.second() >= 0 && now.second() < 10) {
    tft.print(leadingZero[now.second()]);
  }
  else {
    tft.print(now.second());
  }

}

void drawData() {
  tft.setTextSize(2.5);
  if (myData.channel == 1) {
    myData_Outdoor = myData;
  }
  if (myData.channel == 2) {
    myData_Greenhouse = myData;
  }
  // Print temperature/humidity/atmospheric pressure in Terminal
  Serial.println("Pressure\tHumdity\t\tTemp");
  Serial.print(myData.ReelPressure); Serial.print(" mbar\t");    // Pressure in millibars
  Serial.print(myData.ReelHumidity); Serial.print(" %rH\t");
  Serial.print(myData.ReelTemperature); Serial.println(" *C\t");

  // Print temperature/humidity/atmospheric pressure on TFT Screen

  // Indoor
  tft.setTextColor(RED, BLACK);
  //Print temperature
  tft.setCursor(10, 70);
  tft.print("Home:");
  tft.setCursor(10, 100);
  tft.print(myData_Dht.Dht_Temperature, 1); tft.print("*C");

  //Print humidity
  tft.setCursor(10, 130);
  tft.print(myData_Dht.Dht_Humidity, 1); tft.print("%rH");

  //Print Voltage
  tft.setCursor(10, 190);
  tft.print(readVcc()); tft.print("mV");

  //*************************************************************************************//
  //Outdoor
  tft.setTextColor(WHITE, BLACK);
  //Print temperature
  tft.setCursor(100, 70);
  tft.print("Outdoor:");
  tft.setCursor(100, 100);
  tft.print(myData_Outdoor.ReelTemperature, 1); tft.print("*C");

  //Print humidity
  tft.setCursor(100, 130);
  tft.print(myData_Outdoor.ReelHumidity, 1); tft.print("%rH");

  //Print Atmospheric pressure
  tft.setCursor(100, 160);
  tft.print(myData_Outdoor.ReelPressure, 0); tft.print("mbar");

  //Print Battery Voltage
  tft.setCursor(100, 190);
  tft.print(myData_Outdoor.BatteryVoltage); tft.print("mV");

  //*************************************************************************************//
  //Greenhouse
  tft.setTextColor(BLUE, BLACK);
  //Print temperature
  tft.setCursor(200, 70);
  tft.print("Greenhouse");
  tft.setCursor(200, 100);
  tft.print(myData_Greenhouse.ReelTemperature, 1); tft.print("*C");

  //Print humidity
  tft.setCursor(200, 130);
  tft.print(myData_Greenhouse.ReelHumidity, 1); tft.print("%rH");

  //Print Atmospheric pressure
  tft.setCursor(200, 160);
  tft.print(myData_Greenhouse.ReelPressure, 0); tft.print("mbar");

  //Print Battery Voltage
  tft.setCursor(200, 190);
  tft.print(myData_Greenhouse.BatteryVoltage); tft.print("mV");
}

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
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
