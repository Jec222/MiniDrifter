#include <SPI.h>
#include <SD.h>

// Set the pins used
#define cardSelect 4

File logFile;
File errorFile;

const char logFileName[15] = "/DATA.CSV";
const char errorFileName[15] = "/ERRORS.TXT";

uint8_t i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);
  Serial.println("\nMini Drifter version 0.0000000001");
  pinMode(13, OUTPUT);

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

  i = 0;
}


void loop() {
  // put your main code here, to run repeatedly:
  delay (200);
    if (i < 10) {
      Serial.print(i);
      Serial.println("Writing to files...");
      logFile.print(i);
      logFile.println("data.....");
      errorFile.print(i);
      errorFile.println("errors...");
      logFile.flush();
      errorFile.flush();
      i++;
    }
}
