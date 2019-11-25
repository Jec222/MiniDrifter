#include <SPI.h>
#include <SD.h>
#include "Adafruit_EPD.h"

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

Adafruit_SSD1675 epd(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {delay(50);};
  Serial.println("\nMini Drifter version 0.0000000001");
  pinMode(13, OUTPUT);

  sd_setup();
  epaper_setup();
  
  i = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay (200);
  Serial.print(i);
  Serial.println("Writing to files...");
  logFile.print(i);
  logFile.println("data.....");
  errorFile.print(i);
  errorFile.println("errors...");
  logFile.flush();
  errorFile.flush();
  i++;

  epd_update();
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
    epd.print(0);
    epd.print("\nExt: ");
    epd.print(0);
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
