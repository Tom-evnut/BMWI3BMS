#pragma once
#include "config.h"
#include "BMSModule.h"
#include <FlexCAN.h>

class BMSModuleManager
{
public:
    BMSModuleManager();
    int seriescells();
    void clearmodules();
    void decodecan(CAN_message_t &msg,int debug);
    void decodetemp(CAN_message_t &msg, int debug);
    void balanceCells();
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setUnderVolt(float newVal);
    void setOverVolt(float newVal);
    void setOverTemp(float newVal);
    void setBalanceV(float newVal);
    void setBalanceHyst(float newVal);
    void setSensors(int sensor,float Ignore);
    float getPackVoltage();
    float getAvgTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();
    float getLowVoltage();
    /*
    void processCANMsg(CAN_FRAME &frame);
    */
    void printPackSummary();
    void printPackDetails();
    

private:
    float packVolt;                         // All modules added together
    int Pstring;
    float LowCellVolt;
    float HighCellVolt;
    float lowestPackVolt;
    float highestPackVolt;
    float lowestPackTemp;
    float highestPackTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules;                    // The number of modules that seem to exist
    bool isFaulted;
    int spack;
    /*
    void sendBatterySummary();
    void sendModuleSummary(int module);
    void sendCellDetails(int module, int cell);
    */
    
};
