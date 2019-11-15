#include <SPI.h>
#include <SD.h>

//This might or might not already be included in the main as it will be defined there.
//#define cardSelect 4

class MiniDrifterSD
{

  private:
    File logFile;
    File errorFile;
    const char logFileName[15] = "/DATA.CSV";
    const char errorFileName[15] = "/ERRORS.TXT";

  public:

    MiniDrifterSD() {}

    //input is char array, may change later
    void writeData(char input[]);
    void writeError(char input[]);
    //note there was a delay in our initial run. it was placed at start of loop
    //done b/c logging wasn't working,... may have to add delay



//does nothing right now. Not implemented
    bool cardIsFull();

    bool cardInitialize();
    bool filesOpened();
};
