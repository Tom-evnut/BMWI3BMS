#pragma once
#include <FlexCAN.h>

class BMSModule
{
  public:
    BMSModule();
    void decodecan(int Id, CAN_message_t &msg);
    void decodetemp(CAN_message_t &msg, int CSC);
    void clearmodule();
    void readStatus();
    int getscells();
     int getbalstat();
    bool readModuleValues();
    float getCellVoltage(int cell);
    float getLowCellV();
    float getHighCellV();
    float getAverageV();
    float getLowTemp();
    float getHighTemp();
    float getHighestModuleVolt();
    float getLowestModuleVolt();
    float getHighestCellVolt(int cell);
    float getLowestCellVolt(int cell);
    float getHighestTemp();
    float getLowestTemp();
    float getAvgTemp();
    float getModuleVoltage();
    float getTemperature(int temp);
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();
     uint32_t getError();
    void setAddress(int newAddr);
    int getAddress();
    bool isExisting();
    void setExists(bool ex);
    bool isReset();
    void setReset(bool ex);
    void settempsensor(int tempsensor);
    void setIgnoreCell(float Ignore);
    void setTempOff( int16_t tempoff);


  private:
    float cellVolt[12];          // calculated as 16 bit value * 6.250 / 16383 = volts
    float lowestCellVolt[12];
    float highestCellVolt[12];
    float moduleVolt;          // calculated as 16 bit value * 33.333 / 16383 = volts
    float temperatures[4];     // Don't know the proper scaling at this point
    float lowestTemperature;
    float highestTemperature;
    float lowestModuleVolt;
    float highestModuleVolt;
    float IgnoreCell;
    bool exists;
    bool reset;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;
    int sensor;
    uint8_t moduleAddress;     //1 to 0x3E
    int scells;
    int balstat;
    uint32_t error;
    int variant;
    int16_t TempOff;
};
