#include <SPI.h>
#include <SD.h>
#include "Adafruit_EPD.h"
#include <OneWire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> //For i2c with k1.0
#include <DallasTemperature.h>
#include <RTCZero.h>

#define PIN_LED_RED 13 // Red LED on Pin #13
#define PIN_LED_GREEN 8 // Green LED on Pin #8
#define PIN_VOLTAGE_BATT A7    // Battery Voltage on Pin A7
#define SAMPLE_INTERVAL_SECONDS 0 // RTC - Sample interval in seconds
#define SAMPLE_INTERVAL_MINUTES 1

const int SampleIntSeconds = 500;   //Simple Delay used for testing, ms i.e. 1000 = 1 sec

// Time info for RTC
const byte hours = 8;
const byte minutes = 20;
const byte seconds = 0;

// Date info for RTC
const byte day = 25;
const byte month = 1;
const byte year = 20;

typedef struct Timestamp {
  byte hours;
  byte minutes;
  byte seconds;
  byte day;
  byte month;
  byte year;  
} Timestamp;

RTCZero rtc;
int nextAlarmTimeMinutes;
int nextAlarmTimeSeconds;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 12

// Sets pins for SD
#define cardSelect 4

//Sets pins for ePaper display
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)
#define SD_CS       5
#define SRAM_CS     6
#define EPD_CS      9
#define EPD_DC      10

//Files on SD card
File logFile;
File errorFile;

const int LOG_DECIMAL_DIGITS = 8;
const int NUM_SAMPLES = 5;
const int TIMESTAMP_SIZE = 8 + strlen("-XX::XX");
const int LOG_DATA_STRING_SIZE = (LOG_DECIMAL_DIGITS * 5) + TIMESTAMP_SIZE + NUM_SAMPLES;
char logLine[LOG_DATA_STRING_SIZE] = "";

#define ERROR_MESSAGE_DRIFTER_INVERTED "Drifter was inverted, samples weren't taken."
#define ERROR_MESSAGE_NO_SD "No sd card was detected, couldn't log measurements."

struct SystemErrors {
  bool noSdDetected;
  bool drifterInverted;
};

struct SystemErrors systemErrors = {false, false};

const char logFileName[15] = "/DATA.CSV";
const char errorFileName[15] = "/ERRORS.TXT";

#define READ_COMMAND "R"
#define SLEEP_COMMAND "sleep"


//K1.0 stuff
#define K1_ADDRESS 100              //default I2C ID number for EZO EC Circuit.
byte k1ReturnCode;
char k1Data[32];
int k1DelayTime = 570;
char k1InChar;
char *ec;                        //char pointer used in string parsing.
char *tds;                       //char pointer used in string parsing.
char *sal;                       //char pointer used in string parsing.
char *sg;                        //char pointer used in string parsing.

//For serial input (Serial doesn't work with sleep/wake mode enabled)
bool serialInputComplete;
String serialInput;

uint8_t i;

//Accelerometer
Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH();
#define I2C_ADDRESS_ACCELEROMETER 0x18 // change this to 0x19 for alternative i2c address
#define ACCELEROMETER_RANGE LIS3DH_RANGE_4_G  // 2, 4, 8 or 16 G!
typedef struct AccelerometerReading {
  float x;
  float y;
  float z;
};

float intTempF;
float extTempF;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer = {0x28, 0x31, 0x74, 0xE3, 0x08, 0x00, 0x00, 0x52};
DeviceAddress externalThermometer = {0x28, 0x9C, 0x32, 0x42, 0x0B, 0x00, 0x00, 0xF3};

//Epaper display
Adafruit_SSD1675 epd(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(4000);
  Serial.println("\nMini Drifter version 0.0000000001");

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  Timestamp initialTime;
  initialTime.hours = 4;
  initialTime.minutes = 38;
  initialTime.seconds = 0;
  initialTime.day = 3;
  initialTime.month = 2;
  initialTime.year = 20;

  rtc_setup(initialTime);
  sd_setup();
  temp_setup();
  accelerometer_setup();
  k1_setup();
  epaper_setup();

  i = 0;
  k1_sleep();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay (200);
  blink(PIN_LED_GREEN, 2);
  rtc_debug_serial_print();
  rtc_enterDeepSleep();
  
  AccelerometerReading accel = accelerometer_read();
  systemErrors.drifterInverted = accel.z < 0;
    
  if (systemErrors.drifterInverted) {
    /* Drifter is not upright, don't take samples */
  // request to all devices on the bus
  } else {

    Serial.println("Taking measurements...");
    temp_readTemperatures();
    k1_wake();
    k1_takeMeasurement();
    k1_sleep();
    sd_logData();
  }
  

  drifter_handleSystemErrors();
  epaper_update();
  i++; 
  
  delay(3*1000);
}

void rtc_setup(Timestamp initialTime) {
  rtc.begin();
  rtc.setTime(initialTime.hours, initialTime.minutes, initialTime.seconds);
  rtc.setDate(initialTime.day, initialTime.month, initialTime.year);
  nextAlarmTimeMinutes = initialTime.minutes;
  nextAlarmTimeSeconds = initialTime.seconds;
}

void rtc_enterDeepSleep() {
  // Interval Timing and Sleep Code
  delay(200);   // just for debugging

  // It's not like a delay, setting the seconds to next alarm means that when the seconds in the real time clock matches the number we set here, 
  // the alarm goes off. This is why you have to make nextAlarmTimeSeconds be a funciton of itself. The next alarm time is not relative to
  // current time, its absolute.
  nextAlarmTimeMinutes = (nextAlarmTimeMinutes + SAMPLE_INTERVAL_MINUTES) % 60;
  nextAlarmTimeMinutes = rtc.getMinutes();
  nextAlarmTimeSeconds = rtc.getSeconds() + 8;
  rtc.setAlarmSeconds(nextAlarmTimeSeconds);
  Serial.println("Next alarm time minutes");
  Serial.println(nextAlarmTimeMinutes);
  delay(100);
  rtc.setAlarmMinutes(nextAlarmTimeMinutes); // RTC time to wake, currently seconds only
  rtc.enableAlarm(rtc.MATCH_MMSS); // Match seconds only
  rtc.attachInterrupt(alarmMatch); // Attaches function to be called, currently blank
  delay(50); // Brief delay prior to sleeping not really sure its required

  Serial.println("Entering deep sleep..");
  delay(100);
  rtc.standbyMode();    // Sleep until next alarm match
  Serial.println("Read");
  delay(100);
  // Code re-starts here after sleep !
}

void alarmMatch() {}

void rtc_debug_serial_print() {
  Serial.print(rtc.getDay());
  delay(100);
  Serial.print("/");
    delay(100);
  Serial.print(rtc.getMonth());
    delay(100);
  Serial.print("/");
    delay(100);
  Serial.print(rtc.getYear()+2000);
    delay(100);
  Serial.print(" ");
    delay(100);
  Serial.print(rtc.getHours());
    delay(100);
  Serial.print(":");
    delay(100);
  if(rtc.getMinutes() < 10)
    Serial.print('0');      // Trick to add leading zero for formatting
  Serial.print(rtc.getMinutes());
    delay(100);
  Serial.print(":");
    delay(100);
  if(rtc.getSeconds() < 10)
    Serial.print('0');      // Trick to add leading zero for formatting
  Serial.print(rtc.getSeconds());
    delay(100);
  Serial.print(",");
    delay(100);
//  Serial.println(BatteryVoltage());   // Print battery voltage  
}

// blink out an error code, Red on pin #13 or Green on pin #8
void blink(uint8_t LED, uint8_t flashes) {
  uint8_t i;
  for (i=0; i<flashes; i++) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(200);
  }
}

void sd_setup() {
   // SD setup
  if (!SD.begin(cardSelect)) {
    Serial.println("Couldn't initialize SD card");
  }

  bool logFileEmpty = !SD.exists(logFileName);

  logFile = SD.open(logFileName, FILE_WRITE);
  errorFile = SD.open(errorFileName, FILE_WRITE);

  if (!logFile) {
    Serial.println("Couldn't open log file");
  }
  if (!errorFile) {
    Serial.println("Couldn't open error file");
  }

  if (logFileEmpty) {
    /* File didn't previously exist, print header at top */
    sprintf(logLine, "yy-mm-dd-HH:MM,Int,Ext,Tds,Sal,Con\n");
    logFile.write(logLine);
  }

  logFile.flush();
}

void sd_logData() {
  sprintf(logLine, "%d-%d-%d-%d:%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", /*month*/-1, /*day*/-1, /*year*/-1, /*hours*/21, /*min*/ 21, intTempF, extTempF, tds, sal, ec);
  logFile.write(logLine);
  for (int i = 0; i < LOG_DATA_STRING_SIZE; i++) {
    logLine[i] = '\0';
  }
  logFile.flush();
}

void drifter_handleSystemErrors() {
  if (systemErrors.noSdDetected) {
    errorFile.write("PUT_DATE_HERE ");
    errorFile.write(ERROR_MESSAGE_NO_SD);
    errorFile.write("\n");
  }

  if (systemErrors.drifterInverted) {
    errorFile.write("PUT_DATE_HERE ");
    errorFile.write(ERROR_MESSAGE_DRIFTER_INVERTED);
    errorFile.write("\n");
  }

  errorFile.flush();
}

void epaper_setup() {
  // ePaper display setup
  epd.begin();
  epd.setTextWrap(true);
  epd.setTextSize(2);
}


void epaper_update() {
    epd.clearBuffer();
    epd.setCursor(10, 10);
    epd.setTextColor(EPD_BLACK);
    if (systemErrors.drifterInverted) {
      epd.print("Not upright");
      epd.print("\nInt: ");
      epd.print(intTempF);
      epd.print("\nExt: NA");
      epd.print("\nTDS: NA");
      epd.print("\nCon: NA");
      epd.print("\nSal: NA");
      epd.print("\n");
    } else {
      epd.print("MiniDrifter 0.01");
      epd.print("\nInt: ");
      epd.print(intTempF);
      epd.print("\nExt: ");
      epd.print(extTempF);
      epd.print("\nTDS: ");
      epd.print(tds);
      epd.print("\nCon: ");
      epd.print(ec);
      epd.print("\nSal: ");
      epd.print(sal);
      epd.print("\n");
      epd.print(i);
      epd.print(" samples taken");
    }
      epd.display();
}

void temp_setup() {
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);

  Serial.print("Internal temp Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC);
  Serial.println();

   Serial.print("External temp Resolution: ");
  Serial.print(sensors.getResolution(externalThermometer), DEC);
  Serial.println();
}

void temp_readTemperatures() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(insideThermometer);
  intTempF = DallasTemperature::toFahrenheit(tempC);

  tempC = sensors.getTempC(externalThermometer);
  extTempF = DallasTemperature::toFahrenheit(tempC);
}

void k1_setup() {
  Wire.begin();
}

void k1_debug_serial_parse_data() {
  //this function will break up the CSV string into its 4 individual parts. EC|TDS|SAL|SG.
                                      //this is done using the C command “strtok”.

  ec = strtok(k1Data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  Serial.print("EC:");                //we now print each value we parsed separately.
  Serial.println(ec);                 //this is the EC value.

  Serial.print("TDS:");               //we now print each value we parsed separately.
  Serial.println(tds);                //this is the TDS value.

  Serial.print("SAL:");               //we now print each value we parsed separately.
  Serial.println(sal);                //this is the salinity value.

  Serial.print("SG:");               //we now print each value we parsed separately.
  Serial.println(sg);                //this is the specific gravity.
  Serial.println();                  //this just makes the output easier to read by adding an extra blank line

  //uncomment this section if you want to take the values and convert them into floating point number.
/*
    ec_float=atof(ec);
    tds_float=atof(tds);
    sal_float=atof(sal);
    sg_float=atof(sg);
*/
}

void accelerometer_setup() {
  Serial.println("LIS3DH setup...");

  if (!accelerometer.begin(I2C_ADDRESS_ACCELEROMETER)) {
    Serial.println("Couldnt start");
    while (1) yield();
  }

  Serial.println("LIS3DH found!");

  accelerometer.setRange(ACCELEROMETER_RANGE);

  Serial.print("Range = "); Serial.print(2 << accelerometer.getRange());
  Serial.println("G");
}

AccelerometerReading accelerometer_read() {
  sensors_event_t event;
  accelerometer.getEvent(&event);

   /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.println();
  AccelerometerReading reading = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  return reading;
}

void k1_takeMeasurement() {
  k1DelayTime = 570;                        //Delay to allow the K1 to take a reading.
  Wire.beginTransmission(K1_ADDRESS);       //call the circuit by its ID number.
  Wire.write(READ_COMMAND);
  Wire.endTransmission();

  delay(k1DelayTime);

  Wire.requestFrom(K1_ADDRESS, 32, 1);      //call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
  k1ReturnCode = Wire.read();               //the first byte is the response code, we read this separately.

      switch (k1ReturnCode) {                           //switch case based on what the response code is.
        case 1:                                 //decimal 1.
          Serial.println("Success");            //means the command was successful.
          break;                                //exits the switch case.

        case 2:                                 //decimal 2.
          Serial.println("Failed");             //means the command has failed.
          break;                                //exits the switch case.

        case 254:                               //decimal 254.
          Serial.println("Pending");            //means the command has not yet been finished calculating.
          break;                                //exits the switch case.

        case 255:                               //decimal 255.
          Serial.println("No Data");            //means there is no further data to send.
          break;                                //exits the switch case.

      }

  i=0;
  while (Wire.available()) {                 //are there bytes to receive.
    k1InChar = Wire.read();                  //receive a byte.
    k1Data[i] = k1InChar;                    //load this byte into our array.
    i += 1;                                  //incur the counter for the array element.
    if (k1InChar == 0) {                     //if we see that we have been sent a null command.
      i = 0;                                 //reset the counter i to 0.
      Wire.endTransmission();                //end the I2C data transmission.
      break;                                 //exit the while loop.
    }
  }

   k1_debug_serial_parse_data();

  delay(200);
}

void k1_sleep(){
  k1DelayTime = 250;
  
  Wire.beginTransmission(K1_ADDRESS);                                            //call the circuit by its ID number.
  Wire.write(SLEEP_COMMAND);                                                   //transmit the command that was sent through the serial port.
  Wire.endTransmission();

}

void k1_wake(){                     //any request should be enough to wake the k1 as per the example
  Wire.requestFrom(K1_ADDRESS, 32, 1);        //we shouldn't actually need a wake command
  k1ReturnCode = Wire.read();
  Serial.println(k1ReturnCode);
}
