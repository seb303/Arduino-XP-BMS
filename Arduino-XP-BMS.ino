// Arduino-XP-BMS  v1.2 2020-09-02
// -------------------------------
// A BMS for Valence XP batteries, designed to run on Arduino or similar hardware.
// by Seb Francis -> https://diysolarforum.com/members/seb303.13166/
//
// https://github.com/seb303/Arduino-XP-BMS
//
//
// Inspired by:
//  ____                _  _____  ___  __  _______
// / __ \___  ___ ___  | |/_/ _ \/ _ )/  |/  / __/
/// /_/ / _ \/ -_) _ \_>  </ ___/ _  / /|_/ /\ \  
//\____/ .__/\__/_//_/_/|_/_/  /____/_/  /_/___/
//    /_/
// https://github.com/t3chN0Mad/OpenXPBMS
//
//
// Overview
// --------
// Designed to provide monitoring of Valence XP batteries in order to:
// * Keep the Valence internal BMS awake so the intra-module balancing is active.
// * Provide a signal to a charge controller to disable charging in case of individual cell over-voltage or over-temperature.
// * Provide a signal to a load disconnect relay in case of individual cell under-voltage or over-temperature.
// * Provide warning and shutdown status outputs for over-temperature, over-voltage, under-voltage and communication error.
// * Provide basic event logging to EEPROM.
// * Have a mode for long term storage / not in use, where it will let the batteries rest at a lower SOC.
//
// Current Limitations
// -------------------
// * Does not handle inter-battery balancing, so only suitable for parallel installations.
// * Only supports 4 cell / 12V batteries.
// * No checking of low temperature (ideally the charge controller should have an external sensor to reduce current at low temperatures).
//
// Hardware
// --------
// This sketch has been primarily written for and tested on Teensy 3.2 hardware, but should run on any Arduino or similar board that
// has a dedicated hardware serial port. In order to view the console output you'll need either native-USB support (e.g. Teensy) or
// in the case of Arduino a board with multiple serial ports, such as the Mega or Due (since Arduino USB uses one of the serial ports).
//
// The clock speed will need to be adequate for good serial timing at 115200 baud. Unless using a specific crystal which is an exact
// multiple of 115200, a good rule of thumb would be to have a clock speed around 20Mhz or more to ensure good enough timing. This
// does also depend on how the hardware is implemented - for example, the Teensy 3.2 has a high resolution baud rate for the hardware
// UART, and so is particularly accurate. The exact serial timing error depends on the clock rate:
// At 24 MHz: -0.08%  <- plenty accurate enough, and uses the least power
// At 48 Mhz: +0.04%
// At 96 MHz: -0.02%
//
// Requires the following additional components:
// * A 5V voltage regulator
// * An external RS485 transceiver such as the MAX485 (if the MCU inputs are not 5V tolerant run the RO/RX through a potential divider)
// * Some LEDs/resistors for the status display
// * Some MOSFETs/resistors to convert the Enable outputs to the voltage/current levels required for the charging & load control
//
// Installation & Configuration
// ----------------------------
// * Install Arduino IDE, and if using Teensy hardware: Teensyduino - https://www.pjrc.com/teensy/teensyduino.html
// * Select board and clock rate (e.g. 24 MHz is plenty fast enough).
// * Define the pin numbers where the outputs and RS485 driver are connected.
// * Define other board-specific parameters, such as port for Serial Monitor, EEPROM size, etc.
// * List the battery ids in the batteries array.
// * Configure the desired thresholds for voltage, temperature and SOC.
//   The default settings are quite conservative, chosen to maximise battery life rather than squeeze out every last Ah of
//   capacity. The values used by the official Valence U-BMS are much less conservative, and are shown in the comments.
//
// Console interface
// -----------------
// Commands can be entered via the Serial Monitor.
// debug 0      - turn off debugging output
// debug 1      - debugging output shows errors, status changes and other occasional info
// debug 2      - in addition to the above, debugging output shows continuous status and readings from batteries
// debug 21     - show status and readings from batteries once, then switch to debug level 1
// debug 2 <n>  - show status and readings from batteries every <n> seconds, otherwise as debug level 1
// mode normal  - enter normal mode
// mode storage - enter long term storage mode
// log read     - read events log from EEPROM
// log clear    - clear events log
// reset cw     - resets CommsWarning status (otherwise this stays on once triggered)
//
// EEPROM data
// -----------
// The top 32 bytes are reserved for storing settings persistently:
// Byte 0: debug level (0, 1, 2)
// Byte 1: mode (0 = normal, 1 = storage)
//
// The rest of the EEPROM stores a 32 byte data packet whenever the status changes:
// 0  PO   0   0   ST  STC  EC  EL  OTW OTS OVW OVS UVW UVS CW  CS (uint16_t bitmap)
//      PO is set for the first event after power on
//      ST is set when storage mode is active
//      STC is set when storage mode is active and charging (i.e. storageMinSOC has been reached)
//
//
// Timestamp of event = number of seconds since power-on (uint32_t)
//
// Values from the battery, only if the status change was triggered by a value from a specific battery:
// Battery id (uint8_t)  (0 if no battery values)
// V1 (int16_t)
// V2 (int16_t)
// V3 (int16_t)
// V4 (int16_t)
// T1 (int16_t)
// T2 (int16_t)
// T3 (int16_t)
// T4 (int16_t)
// PCBA (int16_t)
// SOC (uint16_t)
// CURRENT (int16_t)
//
// 3 bytes unused
//
// License
// -------
// This sketch is released under GPLv3 and comes with no warranty or guarantees. Use at your own risk!
// Libraries used in this sketch may have licenses that differ from the one governing this sketch.
// Please consult the repositories of those libraries for information.


#include <Arduino.h>
#include <EEPROM.h>

// Digital out pin numbers for external control
// High when enabled
#define EnableCharging 3
#define EnableLoad 4

// Digital out pin numbers for external status display
// High when triggered
#define OverTemperatureWarning 5
#define OverTemperatureShutdown 6
#define OverVoltageWarning 7
#define OverVoltageShutdown 8
#define UnderVoltageWarning 9
#define UnderVoltageShutdown 10
#define CommsWarning 11     // Indicates at least 1 read error has occurred since system start
#define CommsShutdown 12    // Indicates shutdown due to too many consecutive read errors

// If defined, the Warning status is turned off when in Shutdown status
// This allows use of bi-colour LEDs for each Warning/Shutdown pair
// Comment out to allow independent Warning/Shutdown status outputs
#define ShutdownTurnsOffWarning

// RS485 interface
// Which serial port to use
#define RS485 Serial1
// Enable RS485 transmission
#define enableRS485Tx 2     // Can be any pin
#define hasAutoTxEnable     // Comment this out if the underlying serial does not have automatic transmitterEnable
// Serial out to RS485
#define RS485Tx 1           // On Teensy 3.2: 1 or 5 for Serial1, 10 or 31 for Serial2, 8 for Serial3
// Serial in from RS485
#define RS485Rx 0           // On Teensy 3.2: 0 or 21 for Serial1, 9 or 26 for Serial2, 7 for Serial3

// Serial port to use for Serial Monitor console
#define Console Serial

// Size in bytes of EEPROM (for storing settings and status logs)
#define EEPROMSize 2048
#define EEPROMSettings (EEPROMSize-32)
// Comment out to disabled EEPROM usage
#define EnableSaveSettingsToEEPROM
#define EnableLogToEEPROM

// Set ids numbers for batteries in system
uint8_t batteries[] = {17, 19};
// Number of cells per battery (not fully implemented - other code changes required to support a number other than 4)
#define NumberOfCells 4

// Over temperature thresholds (0.01C)
// A shutdown condition disables charging and load
int16_t cellOT_Warning = 5500;  // Valence U-BMS value = 6000
int16_t cellOT_Shutdown = 6000; // Valence U-BMS value = 6500
int16_t PCBAOT_Warning = 8000;  // Valence U-BMS value = 8000
int16_t PCBAOT_Shutdown = 8500; // Valence U-BMS value = 8500
int16_t OT_Hysteresis = 200;    // Temperature of all cells must drop below threshold by this amount before warning/shutdown disabled

// Over voltage thresholds (mV)
// A shutdown condition disables charging
int16_t cellOV_Warning = 3600;  // Valence U-BMS value = 3900  [maybe 3600 will trigger warnings too easily, 3700 might be better]
int16_t cellOV_Shutdown = 3900; // Valence U-BMS value = 4000
int16_t OV_Hysteresis = 200;    // Voltage of all cells must drop below threshold by this amount before warning/shutdown disabled

// Under voltage thresholds (mV)
// A shutdown condition disables load
int16_t cellUV_Warning = 3000;  // Valence U-BMS value = 2800
int16_t cellUV_Shutdown = 2800; // Valence U-BMS value = 2300
int16_t UV_Hysteresis = 200;    // Voltage of all cells must rise above threshold by this amount before warning/shutdown disabled

// Long term storage SOC range (%)
// These parameters only have an effect when in long term storage mode
uint16_t storageMinSOC = 50;     // If at least 1 battery drops to this SOC level, charging is enabled
uint16_t storageMaxSOC = 80;     // Once charging is enabled, when at least 1 battery reaches this SOC level, charging is disabled again


// Not much to configure beyond this point....


// Debug level
#define InitialDebugLevel 1
uint8_t debugLevel;
uint32_t debugInterval = 0;
uint32_t lastDebugOutput;
// Mode
#define InitialMode 0

// Comms params
uint32_t initalPause = 500;    // How long to pause after sending wakeup / writeSingleCoil messages
uint32_t readPause = 50;       // How long to wait after read request before trying to read response from battery
uint32_t loopPause = 500;      // How long to wait in between each loop of the batteries
unsigned int maxReadErrors = 2;     // Max number of consecutive loops in which read errors occurred, at which point charging and load is disabled
unsigned int consecutiveReadErrorCount = 0;

// Commands to send to the batteries.
uint8_t messageW[] = {0x00, 0x00, 0x01, 0x01, 0xc0, 0x74, 0x0d, 0x0a, 0x00, 0x00};
uint8_t writeSingleCoil1[] = {0x00, 0x05, 0x00, 0x00, 0x10, 0x00, 0x00, 0x0d, 0x0a};
uint8_t writeSingleCoil2[] = {0x00, 0x05, 0x00, 0x00, 0x08, 0x00, 0x00, 0x0d, 0x0a};
uint8_t readVolts[] = {0x00, 0x03, 0x00, 0x45, 0x00, 0x09, 0x00, 0x00, 0x0d, 0x0a};
#define readVoltsResLen 25
uint8_t readTemps[] = {0x00, 0x03, 0x00, 0x50, 0x00, 0x07, 0x00, 0x00, 0x0d, 0x0a};
#define readTempsResLen 21
uint8_t readSOC[] = {0x00, 0x03, 0x00, 0x6a, 0x00, 0x0c, 0x00, 0x00, 0x0d, 0x0a};
#define readSOCResLen 31
uint8_t readCurrent[] = {0x00, 0x03, 0x00, 0x39, 0x00, 0x0a, 0x00, 0x00, 0x0d, 0x0a};
#define readCurrentResLen 27
uint8_t readBalance[] = {0x00, 0x03, 0x00, 0x1e, 0x00, 0x01, 0x00, 0x00, 0x0d, 0x0a};
#define readBalanceResLen 9

// Status
#define STATUS_PO 14
#define STATUS_ST 11
#define STATUS_STC 10
#define STATUS_EC 9
#define STATUS_EL 8
#define STATUS_OTW 7
#define STATUS_OTS 6
#define STATUS_OVW 5
#define STATUS_OVS 4
#define STATUS_UVW 3
#define STATUS_UVS 2
#define STATUS_CW 1
#define STATUS_CS 0
uint16_t previousStatus = 0;
bool firstEventAfterPowerOn = 1;
unsigned int nextEEPROMAddress;

// Timing
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define numberOfDays(_time_) ( _time_ / SECS_PER_DAY)  


void setup() {
    // Set up debug console serial
    Console.begin(115200);
    delay(200);

    // Set up pins, etc.
    pinMode(EnableCharging, OUTPUT);
    pinMode(EnableLoad, OUTPUT);
    pinMode(OverTemperatureWarning, OUTPUT);
    pinMode(OverTemperatureShutdown, OUTPUT);
    pinMode(OverVoltageWarning, OUTPUT);
    pinMode(OverVoltageShutdown, OUTPUT);
    pinMode(UnderVoltageWarning, OUTPUT);
    pinMode(UnderVoltageShutdown, OUTPUT);
    pinMode(CommsWarning, OUTPUT);
    pinMode(CommsShutdown, OUTPUT);
    digitalWrite(CommsShutdown, 1);         // Initial state is comms shutdown until successful communication with the batteries
    bitSet(previousStatus, STATUS_CS);
    #ifdef hasAutoTxEnable
        RS485.transmitterEnable(enableRS485Tx);
    #else
        pinMode(enableRS485Tx, OUTPUT);
    #endif
    RS485.setTX(RS485Tx);
    RS485.setRX(RS485Rx);
    
    #ifdef EnableSaveSettingsToEEPROM
        // Load settings from EEPROM
        debugLevel = EEPROM.read(EEPROMSettings);
        if (debugLevel > 2) {
            debugLevel = InitialDebugLevel;
        }
        uint8_t mode = EEPROM.read(EEPROMSettings+1);
        if (mode > 1) {
            mode = InitialMode;
        }
    #else
        debugLevel = InitialDebugLevel;
        uint8_t mode = InitialMode;
    #endif
    bitWrite(previousStatus, STATUS_ST, mode);  // Mode according to saved setting, STC always starts as 0
    
    logln("Starting up with debug "+ String(debugLevel) +", mode "+ (mode==1?"storage":"normal") +"\r\n");
    
    #ifdef EnableLogToEEPROM
        // Find next free EEPROM address for logging
        for (nextEEPROMAddress = 0; nextEEPROMAddress < EEPROMSettings; nextEEPROMAddress+=32) {
            if (EEPROM.read(nextEEPROMAddress) == 255) {
                break;
            }
        }
        if (nextEEPROMAddress >= EEPROMSettings) {
            logln("ERROR: Next free EEPROM address not found!\r\n");
            nextEEPROMAddress = 0;
        }
    #endif
    
    initialiseComms();
}

void initialiseComms() {
    logln("Wake up batteries / Initialise comms");
    // Wake up batteries
    wakeUpBatteries();
    // Maybe Write Single Coil messages?
    //writeSingleCoil(writeSingleCoil1);
    //writeSingleCoil(writeSingleCoil2);
    
    // Initial pause
    uint32_t currentTime;
    uint32_t startTime = millis();
    do {
        currentTime = millis();
    } while (currentTime - startTime < initalPause);
}

void loop() {
    uint32_t startTime, currentTime;
    unsigned int i, j, bytesReceived;
    bool allClear_OverVoltageWarning = 1;
    bool allClear_OverVoltageShutdown = 1;
    bool allClear_UnderVoltageWarning = 1;
    bool allClear_UnderVoltageShutdown = 1;
    bool allClear_OverTemperatureWarning = 1;
    bool allClear_OverTemperatureShutdown = 1;
    unsigned int readErrorCount = 0;
    uint16_t currentStatus = previousStatus;
    uint8_t res[31];  // Longest response is 31 bytes
    int16_t volts[4] = {0,0,0,0};
    int16_t temps[5] = {0,0,0,0,0};
    uint16_t soc = 0;
    int16_t current = 0;
    uint8_t balance = 0;

    // Header row
    if (debugLevel >= 2) {
        logln("             V1   V2   V3   V4   VT    T1   T2   T3   T4   PCBA SOC   CURRENT BAL");
    }
        
    // Iterate through all of the batteries connected to the BMS.
    for (i = 0; i < sizeof(batteries); i++) {
        bool statusChangeTriggered = 0;
        
        char battStr[14];
        sprintf(battStr, "Battery %-5u", batteries[i]);
        
        // Ensure read buffer is empty
        while (RS485.available()) {
            RS485.read();
        }
            
        readVolts[0] = batteries[i];
        readVolts[6] = lowByte(ModRTU_CRC(readVolts, 6));
        readVolts[7] = highByte(ModRTU_CRC(readVolts, 6));

        writeToRS485(readVolts, sizeof(readVolts));

        // Allow time for response
        startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);

        bytesReceived = RS485.available();
        if (bytesReceived == readVoltsResLen) {
            for (j = 0; j < readVoltsResLen; j++) {
                uint8_t b = RS485.read();
                res[j] = b;
            }
            // Check response is as expected
            if (res[0] != batteries[i] || res[1] != 0x03 || res[2] != 0x12 || res[23] != 0x0d || res[24] != 0x0a) {
                readErrorCount++;
                log(String(battStr) +"Invalid readVolts response: ");
                logBytes(res, readVoltsResLen);
                logln("");
                continue;
            }
            
            volts[0] = (res[9] << 8) + res[10];  // V1
            volts[1] = (res[11] << 8) + res[12]; // V2
            volts[2] = (res[13] << 8) + res[14]; // V3
            volts[3] = (res[15] << 8) + res[16]; // V4
            
            // Output voltages
            if (debugLevel >= 2) {
                log(battStr);
                logVolts(volts);
            }

            // Check cell voltages

            // Over voltage?
            if (bitRead(previousStatus, STATUS_OVW) == 0) {
                // Currently no warning
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] > cellOV_Warning) {
                        bitSet(currentStatus, STATUS_OVW);
                        statusChangeTriggered = 1;
                    }
                }
            } else {
                // Currently in warning state
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] > cellOV_Warning-OV_Hysteresis) {
                        allClear_OverVoltageWarning = 0;
                    }
                }
            }
            if (bitRead(previousStatus, STATUS_OVS) == 0) {
                // Currently no shutdown
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] > cellOV_Shutdown) {
                        bitSet(currentStatus, STATUS_OVS);
                        statusChangeTriggered = 1;
                    }
                }
            } else {
                // Currently in shutdown state
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] > cellOV_Shutdown-OV_Hysteresis) {
                        allClear_OverVoltageShutdown = 0;
                    }
                }
            }

            // Under voltage?
            if (bitRead(previousStatus, STATUS_UVW) == 0) {
                // Currently no warning
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] < cellUV_Warning) {
                        bitSet(currentStatus, STATUS_UVW);
                        statusChangeTriggered = 1;
                    }
                }
            } else {
                // Currently in warning state
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] < cellUV_Warning+UV_Hysteresis) {
                        allClear_UnderVoltageWarning = 0;
                    }
                }
            }
            if (bitRead(previousStatus, STATUS_UVS) == 0) {
                // Currently no shutdown
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] < cellUV_Shutdown) {
                        bitSet(currentStatus, STATUS_UVS);
                        statusChangeTriggered = 1;
                    }
                }
            } else {
                // Currently in shutdown state
                for (j = 0; j < NumberOfCells; j++) {
                    if (volts[j] < cellUV_Shutdown+UV_Hysteresis) {
                        allClear_UnderVoltageShutdown = 0;
                    }
                }
            }
        } else { // Didn't receive expected response
            readErrorCount++;
            log(String(battStr) +"Invalid readVolts response: ");
            if (bytesReceived == 0) {
                logln("0 bytes received");
            } else {
                logBytes();
                logln("");
            }
            continue;
        }
        
        
        // Ensure read buffer is empty
        while (RS485.available()) {
            RS485.read();
        }
    
        readTemps[0] = batteries[i];
        readTemps[6] = lowByte(ModRTU_CRC(readTemps, 6));
        readTemps[7] = highByte(ModRTU_CRC(readTemps, 6));

        writeToRS485(readTemps, sizeof(readTemps));

        // Allow time for response
        startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);

        bytesReceived = RS485.available();
        if (bytesReceived == readTempsResLen) {
            for (j = 0; j < readTempsResLen; j++) {
                uint8_t b = RS485.read();
                res[j] = b;
            }
            // Check response is as expected
            if (res[0] != batteries[i] || res[1] != 0x03 || res[2] != 0x0e || res[19] != 0x0d || res[20] != 0x0a) {
                readErrorCount++;
                log(String(battStr) +"Invalid readTemps response: ");
                logBytes(res, readTempsResLen);
                logln("");
                continue;
            }
            
            temps[0] = (res[5] << 8) + res[6];   // T1
            temps[1] = (res[7] << 8) + res[8];   // T2
            temps[2] = (res[9] << 8) + res[10];  // T3
            temps[3] = (res[11] << 8) + res[12]; // T4
            temps[4] = (res[3] << 8) + res[4];   // PCBA
            
            // Output temperatures
            if (debugLevel >= 2) {
                logTemps(temps);
            }
            
            // Check cell & PCBA temperatures
            
            // Over temperature?
            if (bitRead(previousStatus, STATUS_OTW) == 0) {
                // Currently no warning
                for (j = 0; j < NumberOfCells+1; j++) {
                    if (j <= 4) {
                        if (temps[j] > cellOT_Warning) {
                            bitSet(currentStatus, STATUS_OTW);
                            statusChangeTriggered = 1;
                        }
                    } else {
                        if (temps[j] > PCBAOT_Warning) {
                            bitSet(currentStatus, STATUS_OTW);
                            statusChangeTriggered = 1;
                        }
                    }
                }
            } else {
                // Currently in warning state
                for (j = 0; j < NumberOfCells+1; j++) {
                    if (j <= 4) {
                        if (temps[j] > cellOT_Warning-OT_Hysteresis) {
                            allClear_OverTemperatureWarning = 0;
                        }
                    } else {
                        if (temps[j] > PCBAOT_Warning-OT_Hysteresis) {
                            allClear_OverTemperatureWarning = 0;
                        }
                    }
                }
            }
            if (bitRead(previousStatus, STATUS_OTS) == 0) {
                // Currently no shutdown
                for (j = 0; j < NumberOfCells+1; j++) {
                    if (j <= 4) {
                        if (temps[j] > cellOT_Shutdown) {
                            bitSet(currentStatus, STATUS_OTS);
                            statusChangeTriggered = 1;
                        }
                    } else {
                        if (temps[j] > PCBAOT_Shutdown) {
                            bitSet(currentStatus, STATUS_OTS);
                            statusChangeTriggered = 1;
                        }
                    }
                }
            } else {
                // Currently in shutdown state
                for (j = 0; j < NumberOfCells+1; j++) {
                    if (j <= 4) {
                        if (temps[j] > cellOT_Shutdown-OT_Hysteresis) {
                            allClear_OverTemperatureShutdown = 0;
                        }
                    } else {
                        if (temps[j] > PCBAOT_Shutdown-OT_Hysteresis) {
                            allClear_OverTemperatureShutdown = 0;
                        }
                    }
                }
            }
        } else { // Didn't receive expected response
            readErrorCount++;
            log(String(battStr) +"Invalid readTemps response: ");
            if (bytesReceived == 0) {
                logln("0 bytes received");
            } else {
                logBytes();
                logln("");
            }
            continue;
        }
        
        
        // Ensure read buffer is empty
        while (RS485.available()) {
            RS485.read();
        }
    
        readSOC[0] = batteries[i];
        readSOC[6] = lowByte(ModRTU_CRC(readSOC, 6));
        readSOC[7] = highByte(ModRTU_CRC(readSOC, 6));

        writeToRS485(readSOC, sizeof(readSOC));

        // Allow time for response
        startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);

        bytesReceived = RS485.available();
        if (bytesReceived == readSOCResLen) {
            for (j = 0; j < readSOCResLen; j++) {
                uint8_t b = RS485.read();
                res[j] = b;
            }
            // Check response is as expected
            if (res[0] != batteries[i] || res[1] != 0x03 || res[2] != 0x18 || res[29] != 0x0d || res[30] != 0x0a) {
                readErrorCount++;
                log(String(battStr) +"Invalid readSOC response: ");
                logBytes(res, readSOCResLen);
                logln("");
                continue;
            }
            
            soc = (res[3] << 8) + res[4];   // SOC
            
            // Output SOC
            if (debugLevel >= 2) {
                logSOC(soc);
            }
            
            // Check SOC if in storage mode
            if (bitRead(previousStatus, STATUS_ST) == 1) {
                if (bitRead(previousStatus, STATUS_STC) == 0) {
                    if (soc <= storageMinSOC*10) {
                        bitSet(currentStatus, STATUS_STC);
                        statusChangeTriggered = 1;
                    }
                } else {
                    if (soc >= storageMaxSOC*10) {
                        bitClear(currentStatus, STATUS_STC);
                        statusChangeTriggered = 1;
                    }
                }
            }
        } else { // Didn't receive expected response
            readErrorCount++;
            log(String(battStr) +"Invalid readSOC response: ");
            if (bytesReceived == 0) {
                logln("0 bytes received");
            } else {
                logBytes();
                logln("");
            }
            continue;
        }
        
        
        // Ensure read buffer is empty
        while (RS485.available()) {
            RS485.read();
        }
    
        readCurrent[0] = batteries[i];
        readCurrent[6] = lowByte(ModRTU_CRC(readCurrent, 6));
        readCurrent[7] = highByte(ModRTU_CRC(readCurrent, 6));

        writeToRS485(readCurrent, sizeof(readCurrent));

        // Allow time for response
        startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);

        bytesReceived = RS485.available();
        if (bytesReceived == readCurrentResLen) {
            for (j = 0; j < readCurrentResLen; j++) {
                uint8_t b = RS485.read();
                res[j] = b;
            }
            // Check response is as expected
            if (res[0] != batteries[i] || res[1] != 0x03 || res[2] != 0x14 || res[25] != 0x0d || res[26] != 0x0a) {
                readErrorCount++;
                log(String(battStr) +"Invalid readCurrent response: ");
                logBytes(res, readCurrentResLen);
                logln("");
                continue;
            }
            
            current = (res[17] << 8) + res[18];     // Current
            
            // Output SOC
            if (debugLevel >= 2) {
                logCurrent(current);
            }
        } else { // Didn't receive expected response
            readErrorCount++;
            log(String(battStr) +"Invalid readCurrent response: ");
            if (bytesReceived == 0) {
                logln("0 bytes received");
            } else {
                logBytes();
                logln("");
            }
            continue;
        }
        
        
        // Ensure read buffer is empty
        while (RS485.available()) {
            RS485.read();
        }
    
        readBalance[0] = batteries[i];
        readBalance[6] = lowByte(ModRTU_CRC(readBalance, 6));
        readBalance[7] = highByte(ModRTU_CRC(readBalance, 6));

        writeToRS485(readBalance, sizeof(readBalance));

        // Allow time for response
        startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);

        bytesReceived = RS485.available();
        if (bytesReceived == readBalanceResLen) {
            for (j = 0; j < readBalanceResLen; j++) {
                uint8_t b = RS485.read();
                res[j] = b;
            }
            // Check response is as expected
            if (res[0] != batteries[i] || res[1] != 0x03 || res[2] != 0x02 || res[7] != 0x0d || res[8] != 0x0a) {
                readErrorCount++;
                log(String(battStr) +"Invalid readBalance response: ");
                logBytes(res, readBalanceResLen);
                logln("");
                continue;
            }
            
            balance = bitRead(res[3], 0);  // Balance Status
            
            // Output SOC
            if (debugLevel >= 2) {
                logBalance(balance);
            }
            
        } else { // Didn't receive expected response
            readErrorCount++;
            log(String(battStr) +"Invalid readBalance response: ");
            if (bytesReceived == 0) {
                logln("0 bytes received");
            } else {
                logBytes();
                logln("");
            }
            continue;
        }
        
        if (debugLevel >= 2) {
            logln("");
        }
        
        // Status changed?
        if (statusChangeTriggered) {
            // Enabled/disable charging and load based on current status
            setECEL(currentStatus);
            
            // Handle the status change
            handleStatusChange(currentStatus, batteries[i], volts, temps, soc, current);
        }
    }
    
    // Clear warnings or shutdowns if everything is ok now
    bool statusChangeTriggered = 0;
    if (bitRead(previousStatus, STATUS_OVW) == 1 && allClear_OverVoltageWarning) {
        bitClear(currentStatus, STATUS_OVW);
        statusChangeTriggered = 1;
    }
    if (bitRead(previousStatus, STATUS_OVS) == 1 && allClear_OverVoltageShutdown) {
        bitClear(currentStatus, STATUS_OVS);
        statusChangeTriggered = 1;
    }
    if (bitRead(previousStatus, STATUS_UVW) == 1 && allClear_UnderVoltageWarning) {
        bitClear(currentStatus, STATUS_UVW);
        statusChangeTriggered = 1;
    }
    if (bitRead(previousStatus, STATUS_UVS) == 1 && allClear_UnderVoltageShutdown) {
        bitClear(currentStatus, STATUS_UVS);
        statusChangeTriggered = 1;
    }
    if (bitRead(previousStatus, STATUS_OTW) == 1 && allClear_OverTemperatureWarning) {
        bitClear(currentStatus, STATUS_OTW);
        statusChangeTriggered = 1;
    }
    if (bitRead(previousStatus, STATUS_OTS) == 1 && allClear_OverTemperatureShutdown) {
        bitClear(currentStatus, STATUS_OTS);
        statusChangeTriggered = 1;
    }
    
    // Handle communication errors
    if (readErrorCount == 0) {
        consecutiveReadErrorCount = 0;
        
        if (bitRead(previousStatus, STATUS_CS) == 1) {
            bitClear(currentStatus, STATUS_CS);
            statusChangeTriggered = 1;
        }
    } else {
        consecutiveReadErrorCount ++;
        if (debugLevel >= 2) {
            logln("Read errors this loop: " +String(readErrorCount)+ ", Consecutive: " +String(consecutiveReadErrorCount));
        }
        
        if (bitRead(previousStatus, STATUS_CS) == 0 && consecutiveReadErrorCount >= maxReadErrors) {
            bitSet(currentStatus, STATUS_CS);
            statusChangeTriggered = 1;
        }
        if (bitRead(previousStatus, STATUS_CW) == 0) {
            bitSet(currentStatus, STATUS_CW); // This indicator will now stay on permanently
            statusChangeTriggered = 1;
        }
    }
    
    // Output current status
    if (debugLevel >= 2) {
        logln("Current status:  ST   STC  EC   EL   OTW  OTS  OVW  OVS  UVW  UVS  CW   CS");
        log(  "                 ");
        logStatusLn(currentStatus);
        logln("");
    }
    
    
    // Set Warning/Shutdown output indicators
    #ifdef ShutdownTurnsOffWarning
        digitalWrite(OverVoltageWarning, bitRead(currentStatus, STATUS_OVW) && !bitRead(currentStatus, STATUS_OVS));
        digitalWrite(UnderVoltageWarning, bitRead(currentStatus, STATUS_UVW) && !bitRead(currentStatus, STATUS_UVS));
        digitalWrite(OverTemperatureWarning, bitRead(currentStatus, STATUS_OTW) && !bitRead(currentStatus, STATUS_OTS));
        digitalWrite(CommsWarning, bitRead(currentStatus, STATUS_CW) && !bitRead(currentStatus, STATUS_CS));
    #else
        digitalWrite(OverVoltageWarning, bitRead(currentStatus, STATUS_OVW));
        digitalWrite(UnderVoltageWarning, bitRead(currentStatus, STATUS_UVW));
        digitalWrite(OverTemperatureWarning, bitRead(currentStatus, STATUS_OTW));
        digitalWrite(CommsWarning, bitRead(currentStatus, STATUS_CW));
    #endif
    digitalWrite(OverVoltageShutdown, bitRead(currentStatus, STATUS_OVS));
    digitalWrite(UnderVoltageShutdown, bitRead(currentStatus, STATUS_UVS));
    digitalWrite(OverTemperatureShutdown, bitRead(currentStatus, STATUS_OTS));
    digitalWrite(CommsShutdown, bitRead(currentStatus, STATUS_CS));
    
    
    // debugLevel 21->1
    if (debugLevel == 21) {
        debugLevel = 1;
        if (debugInterval > 0) {
            lastDebugOutput = seconds();
        }
    } else if (debugInterval > 0) {
        if (seconds() - lastDebugOutput >= debugInterval) {
            debugLevel = 21;
        }
    } else {
        // Ensure we call seconds() to keep track when millis() wraps
        seconds();
    }
    
    // Check for commands from serial console
    char input[32];
    unsigned int len = 0;
    while (Console.available() && len+1 < sizeof(input)) {
        input[len] = Console.read();
        if (input[len] == 13 || input[len] == 10) {
            input[len] = 0;
        }
        len++;
    }
    // Ensure string is terminated
    input[len] = 0;
    // Discard rest of buffer
    while (Console.available()) {
        Console.read();
    }
    if (len > 0) {
        if (strcmp(input,"debug 0") == 0) {
            Console.println("debug 0");
            debugLevel = 0;
            debugInterval = 0;
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings, debugLevel);
            #endif
            
        } else if (strcmp(input,"debug 1") == 0) {
            Console.println("debug 1");
            debugLevel = 1;
            debugInterval = 0;
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings, debugLevel);
            #endif
            
        } else if (strcmp(input,"debug 2") == 0) {
            Console.println("debug 2");
            debugLevel = 2;
            debugInterval = 0;
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings, debugLevel);
            #endif
            
        } else if (strcmp(input,"debug 21") == 0) {
            Console.println("debug 21");
            debugLevel = 21;
            debugInterval = 0;
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings, 1);
            #endif
            
        } else if (strncmp(input,"debug 2 ",8) == 0) {
            int n = atoi(input+8);
            if (n > 0) {
                Console.println("debug 2 "+String(n));
                debugLevel = 21;
                debugInterval = n;
                #ifdef EnableSaveSettingsToEEPROM
                    EEPROM.update(EEPROMSettings, 1);
                #endif
            } else {
                Console.println("debug 2 <n>");
                Console.println("<n> must be a positive integer");
            }
            
        } else if (strcmp(input,"mode normal") == 0) {
            Console.println("mode normal");
            if (bitRead(currentStatus, STATUS_ST) == 1) {
                bitClear(currentStatus, STATUS_ST);
                bitClear(currentStatus, STATUS_STC);
                statusChangeTriggered = 1;
            }
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings+1, 0);
            #endif
            
        } else if (strcmp(input,"mode storage") == 0) {
            Console.println("mode storage");
            if (bitRead(currentStatus, STATUS_ST) == 0) {
                bitSet(currentStatus, STATUS_ST);
                statusChangeTriggered = 1;
            }
            #ifdef EnableSaveSettingsToEEPROM
                EEPROM.update(EEPROMSettings+1, 1);
            #endif
            
        } else if (strcmp(input,"log read") == 0) {
            #ifdef EnableLogToEEPROM
                Console.println("log read");
                // Work backwards to find start of log
                unsigned int a;
                for (unsigned int offset = 32; offset < EEPROMSettings+32; offset+=32) {
                    if (offset > nextEEPROMAddress) {
                        a = nextEEPROMAddress + EEPROMSettings - offset;
                    } else {
                        a = nextEEPROMAddress - offset;
                    }
                    if (EEPROM.read(a) == 255) {
                        break;
                    }
                }
                for (unsigned int offset = 0; offset < EEPROMSettings; offset+=32) {
                    unsigned int b;
                    if (a+32 + offset >= EEPROMSettings) {
                        b = a+32 + offset - EEPROMSettings;
                    } else {
                        b = a+32 + offset;
                    }
                    if (EEPROM.read(b) == 255) {
                        break;
                    }
                    
                    uint16_t status;
                    EEPROM.get(b, status);
                    
                    if (bitRead(status, STATUS_PO)) {
                        Console.println("___________________________________________________________________________________");
                    }
                    Console.println("");
                    
                    uint32_t secondsSincePowerOn;
                    EEPROM.get(b+2, secondsSincePowerOn);
                    Console.print(numberOfDays(secondsSincePowerOn), DEC);
                    printDigits(numberOfHours(secondsSincePowerOn));
                    printDigits(numberOfMinutes(secondsSincePowerOn));
                    printDigits(numberOfSeconds(secondsSincePowerOn));
                    Console.println(" since power on");
                    
                    Console.println("PO   ST   STC  EC   EL   OTW  OTS  OVW  OVS  UVW  UVS  CW   CS");
                    Console.println(
                        String(bitRead(status, STATUS_PO))+"    "+
                        String(bitRead(status, STATUS_ST))+"    "+String(bitRead(status, STATUS_STC))+"    "+
                        String(bitRead(status, STATUS_EC))+"    "+String(bitRead(status, STATUS_EL))+"    "+
                        String(bitRead(status, STATUS_OTW))+"    "+String(bitRead(status, STATUS_OTS))+"    "+
                        String(bitRead(status, STATUS_OVW))+"    "+String(bitRead(status, STATUS_OVS))+"    "+
                        String(bitRead(status, STATUS_UVW))+"    "+String(bitRead(status, STATUS_UVS))+"    "+
                        String(bitRead(status, STATUS_CW))+"    "+String(bitRead(status, STATUS_CS)));
                        
                    uint8_t batteryId;
                    EEPROM.get(b+6, batteryId);
                    if (batteryId != 0) {
                        char str[9];
                        
                        sprintf(str, "%-5u", batteryId);
                        Console.println("Triggered by battery "+ String(str) +" V1   V2   V3   V4   VT    T1   T2   T3   T4   PCBA SOC   CURRENT");
                        Console.print(  "                           ");
                    
                        int total = 0;
                        for (j = 0; j < NumberOfCells*2+1; j++) {
                            int16_t val;
                            EEPROM.get(b+7+j*2, val);
                            sprintf(str, "%-5d", val);
                            Console.print(str);
                            total += val;
                            if (j == NumberOfCells-1) {
                                sprintf(str, "%-6d", val);
                                Console.print(str);
                            }
                        }
                        EEPROM.get(b+7+j*2, soc);
                        sprintf(str, "%-6.1f", soc/10.0);
                        Console.print(str);
                        
                        EEPROM.get(b+7+(j+1)*2, current);
                        sprintf(str, "%-8.2f", current/100.0);
                        Console.println(str);
                    }
                }
                Console.println("___________________________________________________________________________________\r\n");
                
            #else
                Console.println("logging to EEPROM is disabled");
            #endif
            
        } else if (strcmp(input,"log clear") == 0) {
            #ifdef EnableLogToEEPROM
                Console.println("log clear");
                nextEEPROMAddress = 0;
                EEPROM.update(nextEEPROMAddress, 255);
                EEPROM.update(EEPROMSettings-32, 255);
                
            #else
                Console.println("logging to EEPROM is disabled");
            #endif
            
        } else if (strcmp(input,"reset cw") == 0) {
            Console.println("reset cw");
            if (bitRead(currentStatus, STATUS_CW) == 1) {
                bitClear(currentStatus, STATUS_CW);
                statusChangeTriggered = 1;
            }
            
        } else {
            Console.println("Unrecognised command: "+ String(input));
            Console.println("\r\nAvailable commands:");
            Console.println(    "-------------------");
            Console.println("debug 0");
            Console.println("debug 1");
            Console.println("debug 2");
            Console.println("debug 21");
            Console.println("debug 2 <n>");
            Console.println("mode normal");
            Console.println("mode storage");
            Console.println("log read");
            Console.println("log clear");
            Console.println("reset cw");
            Console.println("");
        }
    }
    
    // Status changed?
    if (statusChangeTriggered) {
        // Enabled/disable charging and load based on current status
        setECEL(currentStatus);
            
        // Handle the status change
        handleStatusChange(currentStatus, 0, volts, temps, soc, current);
    }

    // Pause between loops
    startTime = millis();
    do {
        currentTime = millis();
    } while (currentTime - startTime < loopPause);
    
    
    // If too many read errors, try to initialise communications again
    if (bitRead(currentStatus, STATUS_CS) == 1) {
        initialiseComms();
    }
    
    previousStatus = currentStatus;
}

// Enabled/disable charging and load based on current status
void setECEL(uint16_t &currentStatus) {
    // Charging
    if ( bitRead(currentStatus, STATUS_OTS) == 0 && bitRead(currentStatus, STATUS_OVS) == 0 && bitRead(currentStatus, STATUS_CS) == 0 &&
            (bitRead(currentStatus, STATUS_ST) == 0 || bitRead(currentStatus, STATUS_STC) == 1) ) {
        digitalWrite(EnableCharging, 1);
        bitSet(currentStatus, STATUS_EC);
    } else {
        digitalWrite(EnableCharging, 0);
        bitClear(currentStatus, STATUS_EC);
    }
    // Load
    if (bitRead(currentStatus, STATUS_OTS) == 0 && bitRead(currentStatus, STATUS_UVS) == 0 && bitRead(currentStatus, STATUS_CS) == 0) {
        digitalWrite(EnableLoad, 1);
        bitSet(currentStatus, STATUS_EL);
    } else {
        digitalWrite(EnableLoad, 0);
        bitClear(currentStatus, STATUS_EL);
    }
}

// Outputs info on status change, and records to EEPROM
// If batteryId == 0 then volts, temps & soc are ignored
void handleStatusChange(uint16_t currentStatus, uint8_t batteryId, int16_t volts[], int16_t temps[], uint16_t soc, int16_t current) {
    logln("Status change triggered    ST   STC  EC   EL   OTW  OTS  OVW  OVS  UVW  UVS  CW   CS");
    log(  "Previous status:           ");
    logStatusLn(previousStatus);
    log(  "Current status:            ");
    logStatusLn(currentStatus);
    if (batteryId != 0) {
        char str[6];
        sprintf(str, "%-5u", batteryId);
        logln("Triggered by battery "+ String(str) +" V1   V2   V3   V4   VT    T1   T2   T3   T4   PCBA SOC   CURRENT");
        log(  "                           ");
        logVolts(volts);
        logTemps(temps);
        logSOC(soc);
        logCurrent(current);
        logln("");
    }
    logln("");
    
    // Record to EEPROM
    #ifdef EnableLogToEEPROM
        if (firstEventAfterPowerOn) {
            bitSet(currentStatus, STATUS_PO);
            firstEventAfterPowerOn = 0;
        }
        
        EEPROM.put(nextEEPROMAddress, currentStatus);  // 2 bytes
        nextEEPROMAddress += 2;
    
        EEPROM.put(nextEEPROMAddress, seconds());  // 4 bytes
        nextEEPROMAddress += 4;
    
        EEPROM.put(nextEEPROMAddress, batteryId);  // 1 byte
        nextEEPROMAddress += 1;
    
        if (batteryId != 0) {
            unsigned int j;
            for (j = 0; j < NumberOfCells; j++) {   // 8 bytes
                EEPROM.put(nextEEPROMAddress, volts[j]);
                nextEEPROMAddress += 2;
            }
            
            for (j = 0; j < NumberOfCells+1; j++) {   // 10 bytes
                EEPROM.put(nextEEPROMAddress, temps[j]);
                nextEEPROMAddress += 2;
            }
            
            EEPROM.put(nextEEPROMAddress, soc);  // 2 bytes
            nextEEPROMAddress += 2;
            
            EEPROM.put(nextEEPROMAddress, current);  // 2 bytes
            nextEEPROMAddress += 2;
        
            //for (j = 0; j < 3; j++) {  // 3 bytes unused
            //    EEPROM.update(nextEEPROMAddress, 0);
            //    nextEEPROMAddress += 1;
            //}
            nextEEPROMAddress += 3;  // 5 bytes unused
            
        } else {
            nextEEPROMAddress += 25;
        }
    
        // Wrapped?
        if (nextEEPROMAddress >= EEPROMSettings) {
            nextEEPROMAddress = 0;
        }
        // Ensure next data location is marked as unused, so can find start location on power-on
        EEPROM.update(nextEEPROMAddress, 255); //
    #endif
}

// Output to serial console
void log(const String &p) {
    if (debugLevel > 0) {
        Console.print(p);
    }
}
void logln(const String &p) {
    if (debugLevel > 0) {
        Console.println(p);
    }
}
void loghex(const uint8_t &p) {
    if (debugLevel > 0) {
        if (p < 0x10) {
            Console.print('0');
        }
        Console.print(p, HEX);
    }
}

// Outputs bytes buffer as hex
void logBytes(uint8_t buf[], unsigned int len) {
    unsigned int j;
    for (j = 0; j < len; j++) {
        loghex(buf[j]);
        log(" ");
    }
}
// Outputs bytes in RS485 buffer as hex
void logBytes() {
    while(RS485.available()) {
        loghex(RS485.read());
        log(" ");
    }
}
// Outputs status bitmap
void logStatusLn(uint16_t status) {
    logln(  String(bitRead(status, STATUS_ST))+"    "+String(bitRead(status, STATUS_STC))+"    "+
            String(bitRead(status, STATUS_EC))+"    "+String(bitRead(status, STATUS_EL))+"    "+
            String(bitRead(status, STATUS_OTW))+"    "+String(bitRead(status, STATUS_OTS))+"    "+
            String(bitRead(status, STATUS_OVW))+"    "+String(bitRead(status, STATUS_OVS))+"    "+
            String(bitRead(status, STATUS_UVW))+"    "+String(bitRead(status, STATUS_UVS))+"    "+
            String(bitRead(status, STATUS_CW))+"    "+String(bitRead(status, STATUS_CS))
    );
}
// Outputs volts and Vt
void logVolts(int16_t volts[]) {
    int total = 0;
    char str[7];
    for (unsigned int j = 0; j < NumberOfCells; j++) {
        sprintf(str, "%-5d", volts[j]);
        log(str);
        total += volts[j];
    }
    sprintf(str, "%-6d", total);
    log(str);
}
// Outputs temperatures
void logTemps(int16_t temps[]) {
    char str[6];
    for (unsigned int j = 0; j < NumberOfCells+1; j++) {
        sprintf(str, "%-5d", temps[j]);
        log(str);
    }
}
// Outputs SOC
void logSOC(uint16_t soc) {
    char str[7];
    sprintf(str, "%-6.1f", soc/10.0);
    log(str);
}
// Outputs current
void logCurrent(int16_t current) {
    char str[9];
    sprintf(str, "%-8.2f", current/100.0);
    log(str);
}
// Outputs balance
void logBalance(uint8_t balance) {
    char str[6];
    sprintf(str, "%-5d", balance);
    log(str);
}

// Format time digit
void printDigits(uint8_t n) {
    Console.print(":");
    if (n < 10) {
        Console.print('0');
    }
    Console.print(n, DEC);  
}

// Writes an array of bytes to RS485
void writeToRS485(uint8_t data[], unsigned int len) {
    #ifndef hasAutoTxEnable
        digitalWrite(enableRS485Tx, 1);
    #endif
    RS485.write(data, len);
    RS485.flush();
    #ifndef hasAutoTxEnable
        digitalWrite(enableRS485Tx, 0);
    #endif
}

// Broadcast wakeup message to batteries, and set serial mode for data transfer
void wakeUpBatteries() {
    RS485.begin(9600, SERIAL_8N2);
    writeToRS485(messageW, sizeof(messageW));
    RS485.begin(115200, SERIAL_8N2);
}

void writeSingleCoil(uint8_t msg[]) {
    for (unsigned int i = 0; i < sizeof(batteries); i++) {

        msg[0] = batteries[i];
        msg[5] = lowByte(ModRTU_CRC(msg, 5));
        msg[6] = highByte(ModRTU_CRC(msg, 5));
        writeToRS485(msg, 9);
        
        // Allow time for battery to process message and respond
        uint32_t currentTime;
        uint32_t startTime = millis();
        do {
            currentTime = millis();
        } while (currentTime - startTime < readPause);
        
        // Empty receive buffer
        while (RS485.available()) {
            RS485.read();
        }
    }
}

// Compute the MODBUS RTU CRC
// Adapted from https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
uint16_t ModRTU_CRC(uint8_t buf[], unsigned int len) {
    uint16_t crc = 0xFFFF;

    for (unsigned int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

        for (unsigned int i = 8; i != 0; i--) {      // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                  // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else {                        // Else LSB is not set
                crc >>= 1;                  // Just shift right
            }
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;  
};

uint32_t millisAtLastCall = 0;
unsigned int millisWrapCount = 0;
// Returns seconds since power-on
// Must be called occasionally to track when millis() wraps every 50 days
uint32_t seconds() {
    uint32_t now = millis();
    if (now < millisAtLastCall) {
        millisWrapCount++;
    }
    millisAtLastCall = now;
    return now/1000 + 4294967*millisWrapCount;
}