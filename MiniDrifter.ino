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
#define PIN_LED_STATUS 11
//#define PIN_VOLTAGE_BATT_BACKUP A7    // Battery Voltage on Pin A7
#define PIN_VOLTAGE_BATT 14    // Main Battery Voltage on Pin A0 (14 ide)
#define PIN_VOLTAGE_BATT_CIRC_ENABLE 11 //Enable pin for batt reading circuit
#define SAMPLE_INTERVAL_SECONDS 0 // RTC - Sample interval in seconds
#define SAMPLE_INTERVAL_MINUTES 30

const int SampleIntSeconds = 500;   //Simple Delay used for testing, ms i.e. 1000 = 1 sec

// Time info for RTC
const byte hours = 22;
const byte minutes = 50;
const byte seconds = 0;

// Date info for RTC
const byte day = 20;
const byte month = 2;
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
char currentDateAndTime[] = "mm/dd/yy-HH:MM";

#define ONE_WIRE_BUS 12 // Data wire is plugged into port 2 on the Arduino
#define cardSelect 4    //Pin for SD card

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
const int LOG_DATA_STRING_SIZE = (LOG_DECIMAL_DIGITS * 5) + TIMESTAMP_SIZE + NUM_SAMPLES + 4;
char logLine[LOG_DATA_STRING_SIZE] = "";

#define ERROR_MESSAGE_DRIFTER_INVERTED "Drifter was inverted, samples weren't taken."
#define ERROR_MESSAGE_NO_SD "No sd card was detected, couldn't log measurements."

struct SystemErrors {
  bool noSdDetected;
  bool drifterInverted;
  bool mainBatteryDepleted;
};

struct SystemErrors systemErrors = {false, false, false};

#define MAX_BATTERY_VOLTAGE 4.2
#define MIN_BATTERY_VOLTAGE 2.8
int mainBatteryLevelPercent;

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
float ec_float;
float tds_float;
float sal_float;
float sg_float;
bool k1_retry;

//For serial input (Serial doesn't work with sleep/wake mode enabled)
bool serialInputComplete;
String serialInput;

uint8_t i;
uint8_t j;

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
bool temp_retry;

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
  delay(10000);
  Serial.println("\nMini Drifter version 0.8");

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_VOLTAGE_BATT_CIRC_ENABLE, OUTPUT);

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

  k1_retry = false;
  i = 0;
  j = 0;
  k1_sleep();
}

void loop() {
  // put your main code here, to run repeatedly:
  blink(PIN_LED_GREEN, 2);
  blink(PIN_LED_STATUS, 4);
  Serial.println("setup...");
  sd_setup();
  sd_openFiles();
  Serial.println("setup...");

  //rtc_debug_serial_print();
  
  sprintf(currentDateAndTime, "%d/%d/%d-%d:%d", rtc.getMonth(), rtc.getDay(), rtc.getYear(), rtc.getHours(), rtc.getMinutes());


  float battN = (drifter_readMainBatteryVoltage() - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
  if (battN < 0) {
    battN = 0;
  }
  mainBatteryLevelPercent = (battN) * 100.0;
  
  bool needToUpdateEpaper = false;
  
  if (systemErrors.mainBatteryDepleted) {
  
    if (mainBatteryLevelPercent > 0) {
      errorFile.write(currentDateAndTime);
      errorFile.write(" succesfully regained power.\n");
      errorFile.flush();
      systemErrors.mainBatteryDepleted = false;
      needToUpdateEpaper = true;
    }
  } else {
    if (mainBatteryLevelPercent == 0) {
      systemErrors.mainBatteryDepleted = true;
      errorFile.write(currentDateAndTime);
      errorFile.write(" Entered power recovery mode.\n");
      errorFile.flush();
      epaper_clear();
      epaper_drawPowerRecoveryModeIcon();
    }
      needToUpdateEpaper = true;
  }
  
  AccelerometerReading accel = accelerometer_read();
  systemErrors.drifterInverted = accel.z < 0;

  if (!systemErrors.mainBatteryDepleted) {
  if (systemErrors.drifterInverted) {
    /* Drifter is not upright, don't take samples */
  } else {

    if (!systemErrors.noSdDetected) {
      Serial.println("Taking measurements...");
      temp_readTemperatures();
      k1_handle_measurement();
      sd_logData();
      epaper_clear();
      epaper_drawSensorData();
    }
  }


// errorfile.write("\n");   /////////////////////////////////
  drifter_handleSystemErrors();
  }
  if (needToUpdateEpaper) epaper_update();
  i++;
  j++;
  sd_closeFiles();
  rtc_enterDeepSleep();
  //delay(60*1000);
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
  nextAlarmTimeMinutes = (nextAlarmTimeMinutes + SAMPLE_INTERVAL_MINUTES) % 60;
  nextAlarmTimeSeconds = rtc.getSeconds();
  //nextAlarmTimeMinutes = rtc.getMinutes();
  //nextAlarmTimeSeconds = (rtc.getSeconds() + 8);
  //if (nextAlarmTimeSeconds > 59) {
    //nextAlarmTimeMinutes = (nextAlarmTimeMinutes + 1) % 60;
    //nextAlarmTimeSeconds %= 60;
  //}
  rtc.setAlarmSeconds(nextAlarmTimeSeconds);
  rtc.setAlarmMinutes(nextAlarmTimeMinutes); // RTC time to wake, currently seconds only
  rtc.enableAlarm(rtc.MATCH_MMSS); // Match minutes and seconds
  rtc.attachInterrupt(alarmMatch); // Attaches function to be called, currently blank
  delay(50); // Brief delay prior to sleeping not really sure its required

  blink(PIN_LED_GREEN, 2);
  rtc.standbyMode();    // Sleep until next alarm match
  // Code re-starts here after sleep!
}

void alarmMatch() {}

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
  bool logFileEmpty = false;
  if (!SD.begin(cardSelect)) {
    Serial.println("Couldn't initialize SD card");
    systemErrors.noSdDetected = true;
  } else {
    systemErrors.noSdDetected = false;
    logFileEmpty = !SD.exists(logFileName);
  }

  sd_openFiles();
  
  if(!systemErrors.noSdDetected) {

    if (logFileEmpty) {
      /* File didn't previously exist, print header at top */
      sprintf(logLine, "mm/dd/yy-HH:MM,Int,Ext,Tds,Sal,Con,Bat%\n");
      logFile.write(logLine);
  }

  logFile.flush();
  }
  
  sd_closeFiles();
}

void sd_openFiles() {

  if (!systemErrors.noSdDetected) {

    logFile = SD.open(logFileName, FILE_WRITE);
    errorFile = SD.open(errorFileName, FILE_WRITE);

    if (!logFile) {
      Serial.println("Couldn't open log file");
      systemErrors.noSdDetected = true;
    }
  
    if (!errorFile) {
      Serial.println("Couldn't open error file");
      systemErrors.noSdDetected = true;
    }
  }
}

void sd_closeFiles() {
  if (logFile)
    logFile.close();
    
  if (errorFile)
    errorFile.close();
}

void sd_logData() {
  sprintf(logLine, "%d/%d/%d-%d:%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", /*month*/rtc.getMonth(), /*day*/rtc.getDay(), /*year*/rtc.getYear(), /*hours*/rtc.getHours(), /*min*/ rtc.getMinutes(), intTempF, extTempF, tds_float, sal_float, ec_float, mainBatteryLevelPercent);
  logFile.write(logLine);
  for (int i = 0; i < LOG_DATA_STRING_SIZE; i++) {
    logLine[i] = '\0';
  }
  logFile.flush();
}

/* //function from Hugo, function to get the current date, might be useless now
void getDate(){
  //sprintf(date,"%d-%d-%d-%d:%d" , rtc.getMonth(), rtc,getDay(), rtc,getYear(), rtc.getHours(), rtc.getMinutes() );
}
*/

/*    //function from hugo, proposed way of writing errors all at once. Not implemented in any meaningful way.
void logError(string msg){
  errorFile.write(": ");
  errorFile.write(msg);
}
*/

void drifter_handleSystemErrors() {

  if (systemErrors.noSdDetected) {
    sd_closeFiles();
    epd.clearBuffer();
    epaper_drawSdCardErrorIcon();
    sd_setup();
  }

  if (systemErrors.drifterInverted) {
    errorFile.write(currentDateAndTime);
    errorFile.write(" ");
    errorFile.write(ERROR_MESSAGE_DRIFTER_INVERTED);
    errorFile.write("\n");
  }

  errorFile.flush();
}

void epaper_setup() {
  // ePaper display setup
  epd.begin();
  epd.setTextWrap(true);
  epd.setTextSize(1);
  epd.setTextColor(EPD_BLACK);
}

void epaper_drawPowerRecoveryModeIcon() {
  epd.setTextSize(2);
  epd.setCursor(0, 25);
  epd.print("ENTERED\nPOWER\nRECOVERY\nMODE\n");
  epd.print(rtc.getMonth());
  epd.print("/");
  epd.print(rtc.getDay());
  epd.print(" ");
  epd.print(rtc.getHours());
  epd.print(":");
  epd.print(rtc.getMinutes());
}

void epaper_drawSdCardErrorIcon() {
  epd.setTextSize(3);
  epd.setCursor(0, 25);
  epd.print("No SD\n");
  epd.setTextSize(1);
}

void epaper_clear() {
  epd.clearBuffer();
}

void epaper_update() {
  epd.display();
}

void epaper_drawSensorData() {
    epd.setCursor(10, 10);
    epd.setTextSize(1);
    if (systemErrors.drifterInverted) {
      epd.print("Not upright");
      epd.print("\nInt: ");
      epd.print(intTempF);
      epd.print("\nExt: NA");
      epd.print("\nTDS: NA");
      epd.print("\nCon: NA");
      epd.print("\nSal: NA");
      epd.print("\nBat: ");
      epd.print(mainBatteryLevelPercent);
      epd.print("%, ");
      epd.print(drifter_readMainBatteryVoltage());
      epd.print("V\n");
      epd.print("%\n");
    } else {
      epd.print("MiniDrifter");
      epd.print("\nInt: ");
      epd.print(intTempF);
      epd.print("\nExt: ");
      epd.print(extTempF);
      epd.print("\nTDS: ");
      epd.print(tds_float);
      epd.print("\nCon: ");
      epd.print(ec_float);
      epd.print("\nSal: ");
      epd.print(sal_float);
      epd.print("\nBat: ");
      epd.print(mainBatteryLevelPercent);
      epd.print("%, ");
      epd.print(drifter_readMainBatteryVoltage());
      epd.print("V\n");
      epd.print(j);
      epd.print(" samples taken");
    }
}

void temp_setup() {
  // locate devices on the bus
  sensors.begin();

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
}

void temp_readTemperatures() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(insideThermometer);
  intTempF = DallasTemperature::toFahrenheit(tempC);

  tempC = sensors.getTempC(externalThermometer);
  extTempF = DallasTemperature::toFahrenheit(tempC);
}

void temp_handle_measurement(){
  temp_readTemperatures();

  //connection not established error handling
  if(intTempF == -196.6 || extTempF == -196.6){
    errorFile.write("\nError 196, Value indicates no data, trying again.");
    temp_readTemperatures();
  }  

  //out of bounds error handling
  if(intTempF < 41 || intTempF > 122){
    errorFile.write("\nInternal temperature out of bounds. Resampling!");
    intTempF = -1;
    temp_retry = true;
  }
    
   if(extTempF < 41 || extTempF > 113){
    errorFile.write("\nExternal temperature out of bounds. Resampling!");
    extTempF = -1;
    temp_retry = true;
  }


  if(temp_retry) {
    k1_takeMeasurement();
    if(k1ReturnCode = 1){
      if(intTempF < 41 || intTempF > 122){
        errorFile.write("\nInternal temperature out of bounds.");
        intTempF = -1;
      }
      if(extTempF < 41 || extTempF > 113){
        errorFile.write("\nExternal temperature out of bounds.");
        extTempF = -1;
      }
    }
  }
  temp_retry = false;
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

void k1_setup() {
  Wire.begin();
}

void k1_takeMeasurement() {
  k1DelayTime = 570;                        //Delay to allow the K1 to take a reading.
  Wire.beginTransmission(K1_ADDRESS);       //call the circuit by its ID number.
  Wire.write(READ_COMMAND);
  Wire.endTransmission();

  delay(k1DelayTime);

  Wire.requestFrom(K1_ADDRESS, 32, 1);      //call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
  k1ReturnCode = Wire.read();               //the first byte is the response code, we read this separately.

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

   k1_parse_data();

  delay(200);
}

void k1_parse_data() {
  //this function will break up the CSV string into its 4 individual parts. EC|TDS|SAL|SG.

  ec = strtok(k1Data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  //uncomment this section if you want to take the values and convert them into floating point number.

    ec_float=atof(ec);
    tds_float=atof(tds);
    sal_float=atof(sal);
    sg_float=atof(sg);

}

void k1_sleep(){
  k1DelayTime = 250;

  Wire.beginTransmission(K1_ADDRESS);                                            //call the circuit by its ID number.
  Wire.write(SLEEP_COMMAND);                                                   //transmit the command that was sent through the serial port.
  Wire.endTransmission();

}

void k1_wake(){
  Wire.requestFrom(K1_ADDRESS, 32, 1);        //we shouldn't actually need a wake command
  k1ReturnCode = Wire.read();                  //any request should be enough to wake the k1 as per the example
  Serial.println(k1ReturnCode);
}

//Want this function to do all the waking, taking, sleep. And in addition check the k1ReturnCode
//for success,failure,no data, or god forbid pending. (that'd be our fault)
//Then check the actual measurements and test them against our SoW requirements and check that they are within range.
void k1_handle_measurement(){

  k1_wake();
  k1_takeMeasurement();


  switch (k1ReturnCode) {
                                              //switch case based on what the response code is.
      case 1:                                 //decimal 1.
        //Serial.println("Success");            //means the command was successful.
        break;                                //exits the switch case.
  
      case 2:                                 //decimal 2.
        //Serial.println("Failed");             //means the command has failed.
        k1_takeMeasurement();
        errorFile.write("K1 code 2, Command has failed, trying again");
        if(k1ReturnCode = 2){
          errorFile.write("K1 code 2, Command has failed");
        }
        break;                                //exits the switch case.
  
      case 254:                               //decimal 254.
        //Serial.println("Pending");            //means the command has not yet been finished calculating.
        errorFile.write("K1 code 254, Command failed to process in given delay.");
        break;                                //exits the switch case.
  
      case 255:                               //decimal 255.
        //Serial.println("No Data");            //means there is no further data to send.
        errorFile.write("K1 code 255, No data received, command did not reach K1.0");
        break;                                //exits the switch case.
    }

  if(ec_float < 5 || ec_float > 100000){
      errorFile.write("\nK1 Conductivity Out of Bounds. Resampling...");
      ec_float = -1;
      k1_retry = true;
    }
    if(tds_float < 2 || tds_float > 50000){
      errorFile.write("\nK1 TDS Out of Bounds. Resampling...");
      tds_float = -1;
      k1_retry = true;
    }
    if(sal_float < 0.01 || sal_float > 35){
      errorFile.write("\nK1 Salinity Out of Bounds. Resampling...");
      sal_float = -1;
      k1_retry = true;
    }

  if(k1_retry) {
    k1_takeMeasurement();
    if(k1ReturnCode = 1){
      if(ec_float < 5 || ec_float > 100000){
        errorFile.write("\nK1 Conductivity Out of Bounds");
        ec_float = -1;
      }
      if(tds_float < 2 || tds_float > 50000){
        errorFile.write("\nK1 TDS Out of Bounds");
        tds_float = -1;
      }
      if(sal_float < 0.01 || sal_float > 35){
        errorFile.write("\nK1 Salinity Out of Bounds");
        sal_float = -1;
      }
    }
  }
  k1_retry = false;

  k1_sleep();
}

float drifter_readMainBatteryVoltage() {
  //Enable battery reading circuit
  digitalWrite(PIN_VOLTAGE_BATT_CIRC_ENABLE, HIGH);
  const float vPin = (analogRead(PIN_VOLTAGE_BATT) / 1023.0) * 3.3;
  digitalWrite(PIN_VOLTAGE_BATT_CIRC_ENABLE, LOW);
  const float dividerFraction = 3300.0 / (3300.0 + 910.0);
  const float vBatt = (vPin / dividerFraction);
  return vBatt;
}
