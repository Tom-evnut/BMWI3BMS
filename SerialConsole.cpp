/*
 * SerialConsole.cpp
 *
 Copyright (c) 2017 EVTV / Collin Kidder

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
#include "SerialConsole.h"
#include "Logger.h"
#include "BMSModuleManager.h"

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Lets us stream SerialUSB

extern BMSModuleManager bms;

bool printPrettyDisplay;
uint32_t prettyCounter;
int whichDisplay;

SerialConsole::SerialConsole() {
    init();
}

void SerialConsole::init() {
    //State variables for serial console
    ptrBuffer = 0;
    state = STATE_ROOT_MENU;
    loopcount=0;
    cancel=false;
    printPrettyDisplay = false;
    prettyCounter = 0;
    whichDisplay = 0;
}

void SerialConsole::loop() {  
    if (SERIALCONSOLE.available()) {
        serialEvent();
    }
    if (printPrettyDisplay && (millis() > (prettyCounter + 3000)))
    {
        prettyCounter = millis();
        if (whichDisplay == 0) bms.printPackSummary();
        if (whichDisplay == 1) bms.printPackDetails();
    }
}
              
void SerialConsole::printMenu() {   
    Logger::console("\n*************SYSTEM MENU *****************");
    Logger::console("Enable line endings of some sort (LF, CR, CRLF)");
    Logger::console("Most commands case sensitive\n");
    Logger::console("GENERAL SYSTEM CONFIGURATION\n");
    Logger::console("   h = help (displays this message)");
    Logger::console("   S = Sleep all boards");
    Logger::console("   W = Wake up all boards");
    Logger::console("   C = Clear all board faults");
    Logger::console("   F = Find all connected boards");
    Logger::console("   R = Renumber connected boards in sequence");
    Logger::console("   B = Attempt balancing for 5 seconds");
    Logger::console("   p = Toggle output of pack summary every 3 seconds");
    Logger::console("   d = Toggle output of pack details every 3 seconds");
  
    Logger::console("   LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", Logger::getLogLevel());

    float OverVSetpoint;
    float UnderVSetpoint;
    float OverTSetpoint;
    float UnderTSetpoint;
    float balanceVoltage;
    float balanceHyst;
}

/*	There is a help menu (press H or h or ?)

    Commands are submitted by sending line ending (LF, CR, or both)
 */
void SerialConsole::serialEvent() {
    int incoming;
    incoming = SERIALCONSOLE.read();
    if (incoming == -1) { //false alarm....
        return;
    }

    if (incoming == 10 || incoming == 13) { //command done. Parse it.
        handleConsoleCmd();
        ptrBuffer = 0; //reset line counter once the line has been processed
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char) incoming;
        if (ptrBuffer > 79)
            ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd() {

    if (state == STATE_ROOT_MENU) {
        if (ptrBuffer == 1) { //command is a single ascii character
            handleShortCmd();
        } else { //if cmd over 1 char then assume (for now) that it is a config line
            //handleConfigCmd();
        }
    }
}

void SerialConsole::handleShortCmd() 
{
    uint8_t val;

    switch (cmdBuffer[0]) 
    {
    case 'h':
    case '?':
    case 'H':
        printMenu();
        break;
    case 'S':
        Logger::console("Sleeping all connected boards");
        bms.sleepBoards();
        break;
    case 'W':
        Logger::console("Waking up all connected boards");
        bms.wakeBoards();
        break;
    case 'C':
        Logger::console("Clearing all faults");
        bms.clearFaults();
        break;
    case 'F':
        bms.findBoards();
        break;
    case 'R':
        Logger::console("Renumbering all boards.");
        bms.renumberBoardIDs();
        break;
    case 'B':
        bms.balanceCells();
        break;    
    case 'p':
        if (whichDisplay == 1 && printPrettyDisplay) whichDisplay = 0;
        else
        {
            printPrettyDisplay = !printPrettyDisplay;
            if (printPrettyDisplay)
            {
                Logger::console("Enabling pack summary display, 5 second interval");
            }
            else
            {
                Logger::console("No longer displaying pack summary.");
            }
        }
        break;
    case 'd':
        if (whichDisplay == 0 && printPrettyDisplay) whichDisplay = 1;
        else
        {
            printPrettyDisplay = !printPrettyDisplay;
            whichDisplay = 1;
            if (printPrettyDisplay)
            {
                Logger::console("Enabling pack details display, 5 second interval");
            }
            else
            {
                Logger::console("No longer displaying pack details.");
            }            
        }
        break;
    }
}

/*
    if (SERIALCONSOLE.available()) 
    {     
        char y = SERIALCONSOLE.read();
        switch (y)
        {
        case '1': //ascii 1
            renumberBoardIDs();  // force renumber and read out
            break;
        case '2': //ascii 2
            SERIALCONSOLE.println();
            findBoards();
            break;
        case '3': //activate cell balance for 5 seconds 
            SERIALCONSOLE.println();
            SERIALCONSOLE.println("Balancing");
            cellBalance();
            break;
      case '4': //clear all faults on all boards, required after Reset or FPO (first power on)
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Clearing Faults");
       clearFaults();
      break;

      case '5': //read out the status of first board
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Reading status");
       readStatus(1);
      break;

      case '6': //Read out the limit setpoints of first board
       SERIALCONSOLE.println();
       SERIALCONSOLE.println("Reading Setpoints");
       readSetpoint(1);
       SERIALCONSOLE.println(OVolt);
       SERIALCONSOLE.println(UVolt);
       SERIALCONSOLE.println(Tset);
      break; 
                
      case '0': //Send all boards into Sleep state
       Serial.println();
       Serial.println("Sleep Mode");
       sleepBoards();
      break;

      case '9'://Pull all boards out of Sleep state
       Serial.println();
       Serial.println("Wake Boards");
       wakeBoards();
      break;          
                      
        }
    }     
 */



