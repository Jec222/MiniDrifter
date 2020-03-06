#include <SPI.h>
#include <SD.h>
#include "Adafruit_EPD.h"
#include <OneWire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> //For i2c with k1.0
#include <DallasTemperature.h>
#include <RTCZero.h>
#include "icons.h"

#define PIN_LED_RED 13 // Red LED on Pin #13
#define PIN_LED_GREEN 8 // Green LED on Pin #8
#define PIN_LED_STATUS 11
#define PIN_VOLTAGE_BATT 14    // Main Battery Voltage on Pin A0 (14 ide)
#define PIN_VOLTAGE_BATT_CIRC_ENABLE 11 //Enable pin for batt reading circuit
#define SAMPLE_INTERVAL_SECONDS 0 // RTC - Sample interval in seconds
#define SAMPLE_INTERVAL_MINUTES 1

#define PIN_VOLTAGE_BATT_BACKUP A7    // Battery Voltage on Pin A7

// Time info for RTC
const byte HOURS = 13;
const byte MINUTES = 0;
const byte SECONDS = 0;

// Date info for RTC
const byte DAY = 26;
const byte MONTH = 2;
const byte YEAR = 20;

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

const int LOG_DECIMAL_PRECISION = 8;
const int NUM_SAMPLES = 5;
const int TIMESTAMP_SIZE = 8 + strlen("-XX::XX");
const int LOG_DATA_STRING_SIZE = (LOG_DECIMAL_PRECISION * 5) + TIMESTAMP_SIZE + NUM_SAMPLES + 4;
char logLine[LOG_DATA_STRING_SIZE] = "";
char errorLine[750] = ""; // so 1024 is definitely just for now but man, I feel like for our scope, the whole calculation for log_data_string_size is overkill
                          //it does lend well to extension so i guess its alright
char tempComp[8] = "";

#define ERROR_MESSAGE_DRIFTER_INVERTED "Drifter was inverted, samples weren't taken."
#define ERROR_MESSAGE_NO_SD "No sd card was detected, couldn't log measurements."

struct SystemErrors {
  bool noSdDetected;
  bool drifterInverted;
  bool mainBatteryDepleted;
};

struct SystemErrors systemErrors = {false, false, false};

#define MAX_BATTERY_VOLTAGE 4.2
#define MIN_BATTERY_VOLTAGE 3.0
int mainBatteryLevelPercent;

const char logFileName[15] = "/DATA.CSV";
const char errorFileName[15] = "/ERRORS.TXT";

#define READ_COMMAND "R"
#define SLEEP_COMMAND "sleep"
#define TEMP_COMPENSATION "T,"

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

//For serial input (Serial doesn't work with sleep/wake mode enabled)
bool serialInputComplete;
String serialInput;

//Error Handling Codes
int internal_error_code;
int external_error_code;
int ec_OoB;
int tds_OoB;
int sal_OoB;
bool internal_extra_error;
bool external_extra_error;
bool k1_extra_error;

#define NO_ERROR 0
#define NO_CONNECTION 1
#define FAILED_ONCE   2
#define OUT_OF_BOUNDS 3
#define WITHIN_BOUNDS 4

uint8_t i;
uint8_t samples_taken;

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
DeviceAddress internalThermometer = {0x28, 0x31, 0x74, 0xE3, 0x08, 0x00, 0x00, 0x52};
DeviceAddress externalThermometer = {0x28, 0x9C, 0x32, 0x42, 0x0B, 0x00, 0x00, 0xF3};

//Epaper display
Adafruit_SSD1675 epd(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(10000);
  Serial.println("\nMini Drifter version 0.95");

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_VOLTAGE_BATT_CIRC_ENABLE, OUTPUT);

  Timestamp initialTime;
  initialTime.hours = HOURS;
  initialTime.minutes = MINUTES;
  initialTime.seconds = SECONDS;
  initialTime.day = DAY;
  initialTime.month = MONTH;
  initialTime.year = YEAR;

  rtc_setup(initialTime);
  sd_setup();
  temp_setup();
  accelerometer_setup();
  k1_setup();
  epaper_setup();

  internal_error_code = 0;
  external_error_code = 0;
  internal_extra_error = false;
  external_extra_error = false;
  k1_extra_error = false;
  ec_OoB = 0;
  tds_OoB = 0;
  sal_OoB = 0;

  i = 0;
  samples_taken = 1;
  k1_sleep();
}

void loop() {
  // put your main code here, to run repeatedly:
  blink(PIN_LED_GREEN, 2);
  blink(PIN_LED_STATUS, 4);
  sd_setup();
  sd_openFiles();
  
  getDate();

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
        temp_handle_measurement();
        k1_handle_measurement();        
        epaper_clear();
        epaper_drawSensorData();
        epaper_drawSensorIcons();
        epaper_drawDayCode();
        sd_logError();
        sd_logData();
      }
    }

    drifter_handleSystemErrors();
  }

  if (needToUpdateEpaper) epaper_update();
  i++;
  samples_taken++;
  sd_closeFiles();
  rtc_enterDeepSleep();
}

void rtc_setup(Timestamp initialTime) {
  rtc.begin();
  rtc.setTime(initialTime.hours, initialTime.minutes, initialTime.seconds);
  rtc.setDate(initialTime.day, initialTime.month, initialTime.year);
  nextAlarmTimeMinutes = initialTime.minutes;
  nextAlarmTimeSeconds = initialTime.seconds;
}

void rtc_enterDeepSleep() {
  nextAlarmTimeSeconds = nextAlarmTimeSeconds + SAMPLE_INTERVAL_SECONDS;
  if (nextAlarmTimeSeconds >= 60) {
    nextAlarmTimeMinutes += 1;
    nextAlarmTimeSeconds = nextAlarmTimeSeconds % 60;
  }
  nextAlarmTimeMinutes = (nextAlarmTimeMinutes + SAMPLE_INTERVAL_MINUTES) % 60;


  rtc.setAlarmSeconds(nextAlarmTimeSeconds);
  rtc.setAlarmMinutes(nextAlarmTimeMinutes);
  rtc.enableAlarm(rtc.MATCH_MMSS); // Match minutes and seconds
  rtc.attachInterrupt(alarmMatch); // Attaches function to be called, currently blank
  delay(50); // Brief delay prior to sleeping not really sure its required

  blink(PIN_LED_GREEN, 2);
  rtc.standbyMode();    // Sleep until next alarm match
  // Code re-starts here after sleep!
}

void alarmMatch() {}

// blink led
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
      //File didn't previously exist, print header at top
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
  sprintf(logLine, "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", currentDateAndTime, intTempF, extTempF, tds_float, sal_float, ec_float, mainBatteryLevelPercent);
  logFile.write(logLine);
  for (int i = 0; i < LOG_DATA_STRING_SIZE; i++) {
    logLine[i] = '\0';
  }
  logFile.flush();
}

void sd_logError() {
  //if there are errors log them otherwise do nothing
  if(internal_error_code != 0 || external_error_code != 0 || k1ReturnCode != 1 || ec_OoB != 0 || tds_OoB != 0 || sal_OoB != 0){

    strcpy(errorLine,currentDateAndTime);
    strcat(errorLine, "\n");

    if(internal_extra_error)
      strcat(errorLine, "\tInternal Temperature sensor connection not established. Trying again ... connection established.\n");

    switch (internal_error_code) {
      case 1:
        strcat(errorLine, "\tInternal Temperature sensor connection not established. Trying again ... could not be established.\n");
        break;

      case 3:
        strcat(errorLine, "\tInternal Temperature out of bounds. Trying again ... sample out of bounds.\n");
        break;

      case 4:
        strcat(errorLine, "\tInternal Temperature out of bounds. Trying again ... sample within bounds.\n");
        break;
      default:
        break;
    }

    if(external_extra_error)
      strcat(errorLine, "\tExternal Temperature sensor connection not established. Trying again ... connection established.\n");

    switch (external_error_code) {
      case 1:
        strcat(errorLine, "\tExternal Temperature sensor connection not established. Trying again ... could not be established.\n");
        break;

      case 3:
        strcat(errorLine, "\tExternal Temperature out of bounds. Trying again ... sample out of bounds.\n");
        break;

      case 4:
        strcat(errorLine, "\tExternal Temperature out of bounds. Trying again ... sample within bounds.\n");
        break;
      default:
        break;
    }

    switch(k1ReturnCode){
      case 0:
        strcat(errorLine, "\tK1 Probe - Data returned Null\n");
        break;

      case 2:
        strcat(errorLine, "\tK1 Probe - Command has failed. Trying again ... command failure.\n");
        break;

      case 254:
        strcat(errorLine, "\tK1 Probe - Command pending error.\n");
        break;

      case 255:
        strcat(errorLine, "\tK1 Probe - Connection not established ... could not be established.\n");
        break;
      default:
        break;
    }

    if(k1_extra_error)
      strcat(errorLine, "\tK1 Probe - Connection not established ... connection established.\n");

    switch(tds_OoB){
      case 1:
        strcat(errorLine, "\tTDS sensor out of bounds. Trying again ... sample within bounds\n");
        break;
      case 2:
        strcat(errorLine, "\tTDS sensor out of bounds. Trying again ... sample out of bounds\n");
        break;
      default:
        break;
    }
    switch(ec_OoB){
      case 1:
        strcat(errorLine, "\tConductivity sensor out of bounds. Trying again ... sample within bounds\n");
        break;
      case 2:
        strcat(errorLine, "\tConductivity sensor out of bounds. Trying again ... sample out of bounds\n");
        break;
      default:
        break;
    }
    switch(sal_OoB){
      case 1:
        strcat(errorLine, "\tSalinity sensor out of bounds. Trying again ... sample within bounds\n");
        break;
      case 2:
        strcat(errorLine, "\tSalinity sensor out of bounds. Trying again ... sample out of bounds\n");
        break;
      default:
        break;
    }


    internal_error_code = 0;
    external_error_code = 0;
    ec_OoB = 0;
    tds_OoB = 0;
    sal_OoB = 0;
    internal_extra_error = false;
    external_extra_error = false;
    k1_extra_error = false;

    errorFile.write(errorLine);
    for (int i = 0; i < 128; i++) {
      errorLine[i] = '\0';
    }
  }
}

void getDate(){
  sprintf(currentDateAndTime, "%d/%d/%d-%d:%02d", rtc.getMonth(), rtc.getDay(), rtc.getYear(), rtc.getHours(), rtc.getMinutes());
}

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
    errorFile.flush();
  }
}

void epaper_setup() {
  epd.begin();
  epd.setTextWrap(true);
  epd.setTextSize(1);
  epd.setTextColor(EPD_BLACK);
}

void epaper_draw1BitIcon(const unsigned char *bytes, const int width, const int height, const int xPos, const int yPos) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (icon_pixelIsOn(bytes, width, height, x, y)) {
        epd.drawPixel(xPos + x, yPos + y, EPD_BLACK);
      }
    }
  }
}

void epaper_drawPowerRecoveryModeIcon() {
  epd.setTextSize(1);
  epd.setCursor(0, 25);
  epd.print("Entered power\nrecovery mode\n");
  epd.print(currentDateAndTime);
  epaper_draw1BitIcon(BATTERY_ICON_BYTES, BATTERY_ICON_WIDTH, BATTERY_ICON_HEIGHT, 121, 13);
}

void epaper_drawSdCardErrorIcon() {
  epd.setTextSize(1);
  epd.setCursor(0, 25);
  epd.print("No SD card...\n\nPlease insert\nSD.");
  epaper_draw1BitIcon(NO_SD_ICON_BYTES, NO_SD_ICON_WIDTH, NO_SD_ICON_HEIGHT, 121, 13);
}

void epaper_clear() {
  epd.clearBuffer();
}

void epaper_update() {
  epd.display();
}

void epaper_drawSensorData() {
    epd.setCursor(2, 2);
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
      epd.print(samples_taken);
      epd.print(" samples taken");
    }
}

void epaper_drawSensorIcons() {
  Serial.println("Entered Sensor Icon code.");
  if(internal_error_code != 0 || external_error_code != 0 || k1ReturnCode != 1 || ec_OoB != 0 || tds_OoB != 0 || sal_OoB != 0){
    epaper_draw1BitIcon(SENSOR_ICON_BYTES, SENSOR_ICON_WIDTH, SENSOR_ICON_HEIGHT, 121, 0);

    Serial.println("Written Sensor Icon");
    //set the location to start writing
    //define offsets IF the epaper doesn't remember the x of the above location
    
    int x_dimension = 70;
    int y_dimension = 95;
    int offset = 10;

    Serial.println(internal_error_code);
    Serial.println(external_error_code);
    Serial.println(k1ReturnCode);

    epd.setCursor(x_dimension, y_dimension);
  
    switch(internal_error_code){
      case NO_ERROR:
        break;
      case NO_CONNECTION:
        epd.print("Int temp sensor disconnected");
        y_dimension += offset;
        break;
      case OUT_OF_BOUNDS:
        epd.print("Int temp out of bounds");
        y_dimension += offset; 
        break;
      default:
        break;
    }

    epd.setCursor(x_dimension, y_dimension);
    
    switch(external_error_code){
      case NO_ERROR:
        break;
      case NO_CONNECTION:
        epd.print("Ext temp sensor disconnected");
        y_dimension += offset;
        break;
      case OUT_OF_BOUNDS:
        epd.print("Ext temp out of bounds");
        y_dimension += offset;   
        break;
      default:
        break;
    }

    epd.setCursor(x_dimension, y_dimension);
        
    switch(k1ReturnCode){
      case 0:
        epd.print("K1 - Null values");
        break;
      case 2:
        epd.print("K1 - Command Failed");
        break;
      case 254:
        epd.print("K1 - Pending");
        break;
      case 255:
        epd.print("K1 Chip disconnected");
        break;
      case 1:
      case 3:
      case 256:
        if(ec_OoB == 1 || tds_OoB == 1 || sal_OoB == 1){
          epd.print("K1 - out of bounds");
          //write "K1 Probe disconnected"
        }
        break;
      default:
        break;
    }
  }
}

void epaper_drawDayCode() {}

void temp_setup() {
  // locate devices on the bus
  sensors.begin();

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(internalThermometer, 9);
}

void temp_readTemperatures() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(internalThermometer);
  intTempF = DallasTemperature::toFahrenheit(tempC);

  tempC = sensors.getTempC(externalThermometer);
  extTempF = DallasTemperature::toFahrenheit(tempC);
}

/*
 *Takes a reading from the temperature sensors
 *Checks for a valid connection by comparing the received data against a default value.
 *
 *thinking about using an error code like the K1 uses.
 *int internal_error_code;
 *int external_error_code;
 *0 = no error. should be by default
 *1 = connection failed twice
 *2 = failed but connection established
 *3 = out of bounds twice
 *4 = out of bounds but inside second try
 *internal_extra error = failed to connect once but worked second time BUT out of bounds. instead of writing 2 extra cases each time I'll write 3 bool values for each sensor.
 *error code reset in the single error write
 */
void temp_handle_measurement(){
  temp_readTemperatures();

  //connection check
  if(intTempF < -196){
    temp_readTemperatures();

    if(intTempF < -196){
      internal_error_code = 1;
      intTempF = -1;
    }
    else
      internal_error_code = 2;
  }
  //Bounds check
  if(internal_error_code == 0 || internal_error_code == 2){
    if(internal_error_code == 2)
      internal_extra_error = true;
    if(intTempF < 41 || intTempF > 122){
      temp_readTemperatures();
      if(intTempF < 41 || intTempF > 122){
        intTempF = -1;
        internal_error_code = 3;
      }
      else
        internal_error_code = 4;
    }
  }

  //connection check
  if(extTempF < -196){
    temp_readTemperatures();
    Serial.println(external_error_code);
    if(extTempF < -196){
      Serial.println("made it here too");
      external_error_code = 1;
      Serial.println(external_error_code);
      extTempF = -1;
    }
    else
      external_error_code = 2;
  }

  Serial.println(external_error_code);
  //Bounds check
  if(external_error_code == 0 || external_error_code == 2){
    if(external_error_code == 2)
      external_extra_error = true;
    if(extTempF < 41 || extTempF > 122){
      temp_readTemperatures();
      if(extTempF < 41 || extTempF > 122){
        extTempF = -1;
        external_error_code = 3;
      }
      else
        external_error_code = 4;
    }
  }
  Serial.print("final ext code:");
  Serial.println(external_error_code);
}

void accelerometer_setup() {
  if (!accelerometer.begin(I2C_ADDRESS_ACCELEROMETER)) {
    Serial.println("Couldnt start accelerometer");
  } else {
    accelerometer.setRange(ACCELEROMETER_RANGE);
  }
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

void k1_setTempCompensation(){
  float tempC = DallasTemperature::toCelsius(extTempF);
  
  //tempComp
  sprintf(tempComp,"%s,%f",TEMP_COMPENSATION, tempC);

  Serial.print("tempComp: ");
  Serial.println(tempComp);
  
  k1DelayTime = 300;
  Wire.beginTransmission(K1_ADDRESS);
  Wire.write(tempComp);              //insert tempC
  Wire.endTransmission();
  delay(k1DelayTime);
}

  //Sets k1 delay to appropriate time and sends read command to k1 circuit.
  //read the return code and retrieve the k1Data.
  //Calls the k1_parse_data function in order to set data into float values.
void k1_takeMeasurement() {
  k1_setTempCompensation();
  
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
}

  //Takes the k1data and breaks it up into its 4 individual parts. EC|TDS|SAL|SG.
  //Second half
void k1_parse_data() {
  ec = strtok(k1Data, ",");          //let's pars the string at each comma.
  tds = strtok(NULL, ",");            //let's pars the string at each comma.
  sal = strtok(NULL, ",");            //let's pars the string at each comma.
  sg = strtok(NULL, ",");             //let's pars the string at each comma.

  ec_float=atof(ec);
  tds_float=atof(tds);
  sal_float=atof(sal);
  sg_float=atof(sg);
}

  //sets delay to appropriate value and sends a sleep command to the k1 circuit.
void k1_sleep(){
  k1DelayTime = 250;

  Wire.beginTransmission(K1_ADDRESS);                                  
  Wire.write(SLEEP_COMMAND);                                                  
  Wire.endTransmission();
}

  //Sends a command to a k1 that is in sleep mode to wake it up.
void k1_wake(){
  Wire.requestFrom(K1_ADDRESS, 32, 1);
  k1ReturnCode = Wire.read();
}

  //Wakes the k1 and takes a measurements,
  //if return code determines a failure then retry. If fails again sets error code and sets k1 to sleep
  //If successful either time, checks the value if it is within bounds.
  //If Out of Bounds retry, if fails again set error code and sets k1 to sleep
void k1_handle_measurement(){
  k1_wake();
  k1_takeMeasurement();

  switch (k1ReturnCode) {
      case 1:                                 //decimal 1.
        break;                                //exits the switch case.

      case 2:                                 //decimal 2.
        k1_takeMeasurement();
        if(k1ReturnCode = 2){
          ec_float = -1;
          tds_float = -1;
          sal_float = -1;
        }
        else
          k1ReturnCode = 3; // worked second try
        break;                                //exits the switch case.

      case 254:                               //decimal 254.
        break;                                //exits the switch case.

      case 255:                               //decimal 255.
        k1_takeMeasurement();
        if(k1ReturnCode = 255){
          ec_float = -1;
          tds_float = -1;
          sal_float = -1;
        }
        else
          k1ReturnCode = 256 ; // worked second try
        break; 
      default:
        break;
   }

  if(k1ReturnCode == 1){
    if(ec_float == 0 && tds_float == 0 && sal_float == 0 ){
      k1ReturnCode = 0; // values were null/default

      ec_float = -1;
      tds_float = -1;
      sal_float = -1;
    }
  }

  if(k1ReturnCode == 1 || k1ReturnCode == 3|| k1ReturnCode == 256){
    if(k1ReturnCode != 1)
      k1_extra_error = true;
    if(ec_float < 5 || tds_float > 100000){
      ec_OoB = 1;
    }
    if(tds_float < 2 || tds_float > 50000){
      tds_OoB = 1;
    }
    if(sal_float < 0 || sal_float > 35){
      sal_OoB = 1;
    }

    if(ec_OoB == 1 || tds_OoB == 1 || sal_OoB == 1 ){
      k1_takeMeasurement();
      if(ec_float < 5 || tds_float > 100000){
        ec_float = -1;
        ec_OoB = 2;
      }
      if(tds_float < 2 || tds_float > 50000){
        tds_float = -1;
        tds_OoB = 2;
      }
      if(sal_float < 0 || sal_float > 35){
        sal_float = -1;
        sal_OoB = 2;
      }
    }
  }

  k1_sleep();
}

//added a small delay after turning high in order to account for some time to rise if it necessary.
float drifter_readMainBatteryVoltage() {
  //Enable battery reading circuit
  digitalWrite(PIN_VOLTAGE_BATT_CIRC_ENABLE, HIGH);
  delay(100);
  const float vPin = (analogRead(PIN_VOLTAGE_BATT) / 1023.0) * 3.3;
  digitalWrite(PIN_VOLTAGE_BATT_CIRC_ENABLE, LOW);
  const float dividerFraction = 3300.0 / (3300.0 + 910.0);
  const float vBatt = (vPin / dividerFraction);
  return vBatt;
}
