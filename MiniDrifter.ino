#include <SPI.h>
#include <SD.h>
#include "Adafruit_EPD.h"
#include <OneWire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> //For i2c with k1.0
#include <DallasTemperature.h>

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

const char logFileName[15] = "/DATA.CSV";
const char errorFileName[15] = "/ERRORS.TXT";

const char *READ_COMMAND = "R";

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
bool drifterNotUpright;

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
  while (!Serial) {delay(50);};
  Serial.println("\nMini Drifter version 0.0000000001");
  pinMode(13, OUTPUT);

  sd_setup();
  temp_setup();
  accelerometer_setup();
  k1_setup();
  epaper_setup();

  i = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay (200);



  AccelerometerReading accel = accelerometer_read();
  drifterNotUpright = accel.z < 0;

  if (drifterNotUpright) {
    /* Drifter is not upright, don't take samples */
  } else {
   // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting OneWire temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  // It responds almost immediately. Let's print out the data
  //  temp_printTemperature(insideThermometer); // Use a simple function to print out the data
  //  temp_printTemperature(externalThermometer);

  temp_readTemperatures();
  }

  K1_takeMeasurement();

  epaper_update();

  Serial.print(i);
  Serial.println("Writing to files...");
  logFile.print(i);
  logFile.println("data.....");
  errorFile.print(i);
  errorFile.println("errors...");
  logFile.flush();
  errorFile.flush();
  i++;

  delay(15*1000);
}

void sd_setup() {
   // SD setup
  if (!SD.begin(cardSelect)) {
    Serial.println("Couldn't initialize SD card");
  }

  logFile = SD.open(logFileName, FILE_WRITE);
  errorFile = SD.open(errorFileName, FILE_WRITE);

  if (!logFile) {
    Serial.println("Couldn't open log file");
  }
  if (!errorFile) {
    Serial.println("Couldn't open error file");
  }
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
    if (drifterNotUpright) {
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

  // show the addresses we found on the bus
  Serial.print("Internal temp address: ");
  temp_printAddress(insideThermometer);
  Serial.println();

   Serial.print("External temp address: ");
  temp_printAddress(externalThermometer);
  Serial.println();

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
  float tempC = sensors.getTempC(insideThermometer);
  intTempF = DallasTemperature::toFahrenheit(tempC);

  tempC = sensors.getTempC(externalThermometer);
  extTempF = DallasTemperature::toFahrenheit(tempC);
}

void temp_printTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("[Int] ");
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  intTempF = DallasTemperature::toFahrenheit(tempC);
  Serial.println(intTempF); // Converts tempC to Fahrenheit

  tempC = sensors.getTempC(deviceAddress);
  Serial.print("[Int] ");
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  extTempF = DallasTemperature::toFahrenheit(tempC);
  Serial.println(intTempF); // Converts tempC to Fahrenheit
}

void temp_printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void k1_setup() {
  Wire.begin();
}

void k1_debug_serial_update() {
  k1_debug_read_serial_input();

  if (serialInputComplete) {                                                     //if a command was sent to the EZO device.
    for (i = 0; i <= serialInput.length(); i++) {                               //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
      serialInput[i] = tolower(serialInput[i]);                                 //"Sleep" ≠ "sleep"
    }
    i=0;                                                                          //reset i, we will need it later
    if (serialInput[0] == 'c' || serialInput[0] == 'r') {
      k1DelayTime = 570;
    }           //if a command has been sent to calibrate or take a reading we wait 570ms so that the circuit has time to take the reading.
    else {
      k1DelayTime = 250;
    }                                                          //if any other command has been sent we wait only 250ms.



    Wire.beginTransmission(K1_ADDRESS);                                            //call the circuit by its ID number.
    Wire.write(serialInput.c_str());                                                   //transmit the command that was sent through the serial port.
    Wire.endTransmission();                                                     //end the I2C data transmission.


    if (strcmp(serialInput.c_str(), "sleep") != 0) {                                   //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                                                                //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the EC circuit.
      delay(k1DelayTime);                                                             //wait the correct amount of time for the circuit to complete its instruction.

      Wire.requestFrom(K1_ADDRESS, 32, 1);                                         //call the circuit and request 32 bytes (this could be too small, but it is the max i2c buffer size for an Arduino)
      k1ReturnCode = Wire.read();                                                       //the first byte is the response code, we read this separately.

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

      while (Wire.available()) {                 //are there bytes to receive.
        k1InChar = Wire.read();                   //receive a byte.
        k1Data[i] = k1InChar;                    //load this byte into our array.
        i += 1;                                  //incur the counter for the array element.
        if (k1InChar == 0) {                      //if we see that we have been sent a null command.
          i = 0;                                 //reset the counter i to 0.
          Wire.endTransmission();                //end the I2C data transmission.
          break;                                 //exit the while loop.
        }
      }

      Serial.println(k1Data);                  //print the data.
      Serial.println();                         //this just makes the output easier to read by adding an extra blank line
    }
    if (serialInput[0] == 'r') k1_debug_serial_parse_data(); //uncomment this function if you would like to break up the comma separated string into its individual parts.


    serialInput = "";
    serialInputComplete = false;

  } // end of that first if
  delay(200);
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

void k1_debug_read_serial_input() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      serialInputComplete = true;
    } else {
      serialInput += inChar;
    }
  }
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

void K1_takeMeasurement() {


  k1DelayTime = 570;                        //Delay to allow the K1 to take a reading.
  Wire.beginTransmission(K1_ADDRESS);       //call the circuit by its ID number.
  Wire.write(READ_COMMAND);                 //transmit the command that was sent through the serial port.
  Wire.endTransmission();                   //end the I2C data transmission
  delay(k1DelayTime);                       //wait the correct amount of time for the circuit to complete its instruction.

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

void K1_SDPrint(){
  logFile.print("EC:");
  logFile.println(ec);
  logFile.print("TDS:");
  logFile.println(tds);
  logFile.print("SAL:");
  logFile.println(sal);
  logFile.print("SG:");
  logFile.println(sg);
  logFile.println();
}
