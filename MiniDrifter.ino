#include <SPI.h>
#include <SD.h>
#include "Adafruit_EPD.h"
#include <OneWire.h>
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

File logFile;
File errorFile;

const char logFileName[15] = "/DATA.CSV";
const char errorFileName[15] = "/ERRORS.TXT";

uint8_t i;

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
  Serial.begin(115200);
  while (!Serial) {delay(50);};
  Serial.println("\nMini Drifter version 0.0000000001");
  pinMode(13, OUTPUT);

  sd_setup();
  temp_setup();
  epaper_setup();
  
  i = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay (200);
  
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting OneWire temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  // It responds almost immediately. Let's print out the data
//  temp_printTemperature(insideThermometer); // Use a simple function to print out the data
//  temp_printTemperature(externalThermometer);

  temp_readTemperatures();
  Serial.print(i);
  Serial.println("Writing to files...");
  logFile.print(i);
  logFile.println("data.....");
  errorFile.print(i);
  errorFile.println("errors...");
  logFile.flush();
  errorFile.flush();
  i++;

  epaper_update();
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
    epd.print("MiniDrifter 0.01");
    epd.print("\nInt: ");
    epd.print(intTempF);
    epd.print("\nExt: ");
    epd.print(extTempF);
    epd.print("\nTDS: ");
    epd.print(0);
    epd.print("\nCon: ");
    epd.print(0);
    epd.print("\nSal: ");
    epd.print(0);
    epd.print("\n");
    epd.print(i);
    epd.print(" samples taken");
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
