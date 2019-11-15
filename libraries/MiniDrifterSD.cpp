#include "MiniDrifterSD.h"

MiniDrifterSD::MiniDrifterSD() {}

//writes char array to logFile
void MiniDrifterSD::writeData(char input[])
{
  logFile.print(input);
  logFile.flush();
}

//writes char array to errorFile
void MiniDrifterSD::writeError(char input[])
{
  errorFile.println(input);
  errorFile.flush();
}

bool MiniDrifterSD::cardIsFull()
{
  //check in the library how it checks for memory storage left and then if its under a
  //certain amount return false.
  //update... found a couple examples but it seems that SD.h doesn't have a support for it
  //and it involves using a lobrary
  /*
  #include <SdFat.h>
  // Default SD chip select pin.
  const uint8_t sdChipSelect = SS;

  // File system
  SdFat sd;

  void setup() {
   Serial.begin(9600);
   if (!sd.begin(sdChipSelect)) {
     Serial.println("sd.begin failed");
     while(1);
   }
   // Get the count of free clusters in the volume.
   uint32_t freeClusters = sd.vol()->freeClusterCount();
   Serial.print("Free clusters: ");
   Serial.println(freeClusters);

   // Calculate free space in KB.
   float freeKB = 0.512*freeClusters*sd.vol()->blocksPerCluster();
   Serial.print("Free space: ");
   Serial.print(freeKB);
   Serial.println(" KB (KB = 1000 bytes)");
  }

  void loop() {}
  */
}

//maybe move this part under constructor and get rid of this entirely.
//for now will call under constructor.
bool MiniDrifterSD::cardInitialize()
{
  if (!SD.begin(cardSelect))
  {
      Serial.println("Couldn't initialize SD card");
      return false;
  }
  else
  {
      Serial.println("SD card initialized");
      return true;
  }

//shouldn't reach here under normal circumstances
  return false;
}

//maybe change to void and call it openFiles
bool MiniDrifterSD::filesOpened()
{
  logFile = SD.open(logFileName, FILE_WRITE);
  errorFile = SD.open(errorFileName, FILE_WRITE);

  if (!logFile) {
      Serial.println("Couldn't open log file");
      return false;
  }
  if (!errorFile) {
      Serial.println("Couldn't open error file");
      return false;
  }
  return true;
}
