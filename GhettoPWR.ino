//#define DEBUG true
#define TEMP true // Temp sensor present
//#define REV04B true // for <= 0.4b boards (comment both for 0.4c)
#define REV05 true // for >=0.5 boards (comment both for 0.4c)
#define SIGDET 1  // Signal detect mode: 2 = Always on, 1 = I2C, 0 = Mute pin
#define WATCHDOG true // I2C Watchdog timer

// Poweroff Reason (# of blinks)
// 6 = No longer charging
// 5 = Critical battery level
// 4 = Over temperature
// 3 = Watchdog
// 2 = Inactivity
// 1 = Button push

#include <avr/wdt.h>
#include <SPI.h>
#include <Wire.h>
#include <I2C_Anything.h>
#include <EEPROM.h>
#define I2C_SLAVE_ADDRESS 0x30 // I2C address of this board
#define CONFIG_VERSION "0.1" // EEPROM schema version
#define CONFIG_START 32 // Where in EEPROM to store persistant config

// SPI debug logging
#ifdef DEBUG
#define beginDebug()  do { SPI.begin (); SPI.setClockDivider(SPI_CLOCK_DIV8); } while (0)
#define Trace(x)      SPIdebug.print   (x)
#define Trace2(x,y)   SPIdebug.print   (x,y)
#define Traceln(x)    SPIdebug.println (x)
#define Traceln2(x,y) SPIdebug.println (x,y)
#define TraceFunc()   do { SPIdebug.print (F("In function: ")); SPIdebug.println (__PRETTY_FUNCTION__); } while (0)
class tSPIdebug : public Print
{
  public:
    virtual size_t write (const byte c)
    {
      // Not needed, and SS conflicts with D10
      //digitalWrite(SS, LOW);
      SPI.transfer (c);
      //digitalWrite(SS, HIGH);
      return 1;
    }  // end of tSPIdebug::write
}; // end of tSPIdebug
// an instance of the SPIdebug object
tSPIdebug SPIdebug;
#else
#define beginDebug()  ((void) 0)
#define Trace(x)      ((void) 0)
#define Trace2(x,y)   ((void) 0)
#define Traceln(x)    ((void) 0)
#define Traceln2(x,y) ((void) 0)
#define TraceFunc()   ((void) 0)
#endif // DEBUG

// Settings object structure
struct SettingsStruct {
  char version[4];
  // The variables of your settings
  float voltageBattMaxCharge, vComp;
} settings = {
  CONFIG_VERSION,
  // The default values
  14.7, 28.2
};

// Begin configurable options
float voltageBattMinCharge = 9.0; // Minimum battery voltage to begin charging
//float voltageBattMaxCharge = 14.7; // Stop charging when battery reaches this voltage
float voltageBattResumeCharge = 13.0; // After a full charge, don't resume charging until below this threshold
float voltageBattRestingFull = 13.2; // Full battery resting voltage (for percent calc)
float voltageBattRestingEmpty = 12.4; // Empty battery resting voltage (for percent calc)
float voltageBattChargingEmpty = 13.0; // Empty battery charging voltage (for percent calc)
float voltageBattChargingFull = 13.7; // Full battery charging voltage (for percent calc)
#ifdef REV05
float voltageChargeInMin = 11.8;  // Minimum charging input voltage
#endif

float cutoffHv = 12.0; // Power off amp below this battery voltage
float cutonHv = 12.5; // Power amp back on above this battery voltage
float cutoffLv = 11.5; // System auto-off below this battery voltage

unsigned long last500mSecTask = 0;
unsigned long powerStateDelay = 5000; // Delay in power state changes (on back to off)
unsigned long autoShutOffDelay = 600000; // How long without audio until total system shutoff in mS
unsigned long standbyDelay = 60000; // How long without audio until amp (boost) power down in mS (0 = disable)
#ifdef WATCHDOG
unsigned long watchDogTimer = 8000; // Poweroff if nothing recieved from DSP in x milliseconds ( 0 = disable )
#else
unsigned long watchDogTimer = 0;
#endif
unsigned long requestEventLast = 0;
uint8_t signalDetectMode = SIGDET;  // 2 = Always on, 1 = I2C from ghettoDSP, 0 = Mute pin
// End configurable options

bool watchDogTimerDisabled = false; // Internal flag for temporary disable (DSP programming, ect)
unsigned long lastNoAudioDetected = 0; // Time of last no-audio detection for standby/shutdown calculations

uint8_t pinBoost = 2; // Boost converter turn on output (amp) / HIGH = boost on
uint8_t pinBuck = 3; // Buck converter turn on output (5v) / HIGH = buck on
uint8_t pinMute = 4; // Signal detection input / HIGH = signal, LOW = no signal
uint8_t pinLed = 5; // Status LED output
uint8_t pinButton = 6; // Power on/off button input
uint8_t pinCharge = 7; // Charge turn on output / HIGH = charge
#ifdef REV04B
  uint8_t pinFan = 0; // Fan control output // 0 for <= 0.4b, 9 for >= 0.4c rev boards
#else
  uint8_t pinFan = 9; // Fan control output // 0 for <= 0.4b, 9 for >= 0.4c rev boards
#endif
uint8_t pinVbatt = A0; // Battery voltage sense input
uint8_t pinVbuck = A1; // Buck converter voltage sense input
uint8_t pinVboost = A2; // Boost converter voltage sense input
uint8_t pinVchargeOut = A3; // Charging Boost/Buck output
uint8_t pinTemp = A6; // Temp sensor input
#ifdef REV05
  uint8_t pinVchargeIn = A7; // Charging Boost/Buck input
#endif
uint8_t pinDSP = 10;

// Temperature and fanspeed settings
unsigned long lastFanSpeedChange = 0;
uint8_t fanSpeed = 0;
uint8_t fanSpeedLow = 100;
uint8_t fanSpeedMed = 175;
uint8_t fanSpeedHigh = 255;
uint8_t fanTempLow = 35; // Temp in C for low fanspeed
uint8_t fanTempMed = 40; // Temp in C for medium fanspeed
uint8_t fanTempHigh = 45; // Temp in C for high fanspeed
uint8_t tempShutdown = 65; // Temp in C for total shutdown

bool stateLv = LOW; // Buck converter on/off
bool stateHv = LOW; // Boost converter on/off
bool stateCharging = LOW; // Charging on/off
bool statePower = LOW; // DSP board power status (to signal DSP on/off with buck staying on)
uint8_t percentBattery = 0; // Battery charge percent
uint8_t percentBatteryAvg = 0; // Battery charge percent
uint8_t percentBatteryPrevious = 0; // Last displayed battery % used for hesteris
unsigned long percentBatteryAvgCount = 0; // Related to battery percent averaging
unsigned long percentBatteryAvgSum = 0; // Related to battery percent averaging
unsigned long percentBatteryAvgLast = millis(); // Related to battery percent averaging
float tempCAvg = 0;
float tempCAvgSum = 0;
unsigned long tempCAvgLast = millis();
unsigned long tempCAvgCount = 0;

uint8_t stateBattery = 0; // Battery charge state 0 = below cutoff, 1 = low warning, 2 = good
uint8_t stateSys; // System state
int fadeLed = 0; // LED fading related
unsigned long  fadeLedLast = millis(); // LED fading related
bool fadeLedDir = 1; // LED fading related
bool audioDetected = 0; // Audio detection from mute pin or I2C
bool shitterWasFull = false; // Battery charge full or not based on user settings
bool wroteEEP = false; // Flag to not continuously write eeprom accidentally

unsigned long lastPwrStateChange = millis(); // Related to not power cycling the amp (boost) too quickly TODO do we need this?
unsigned long lastLogLine = 0; // Debug related
unsigned long chargingLast = 0; // Last time we checked charge input voltage and decided if we should charge or not

float voltageBatt = 0; // Measured power supply voltage (instantanious)
float voltageBuck = 0; // Measured buck supply voltage
float voltageBoost = 0; // Measured boost supply voltage
float voltageBattAvg = 0; // Average power supply voltage
float voltageChargeOut = 0; // Measured charging input voltage
#ifdef REV05
float voltageChargeIn = 0; // Measured charging input voltage
#endif
int unsigned voltageBattAvgCount = 0; // Related to battery voltage averaging
float voltageBattAvgSum = 0; // Related to battery voltage averaging
float voltageMax = -1; // Related to measuring battery voltage deviation
float voltageMin = -1; // Related to measuring battery voltage deviation
float voltageDev = 0; // Related to measuring battery voltage deviation
unsigned long voltageBattAvgLast = millis(); // Related to battery voltage averaging
float tempC = 0; // Sensor temperature in C

unsigned long ledTimer = millis();

void setup() {
  wdt_enable(WDTO_8S);
  wdt_reset();
  beginDebug ();
  #ifdef DEBUG
    Traceln(F("*******************************"));
  #endif

  // Load non-volatile settings
  readEEPROM();
  
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  pinMode(pinDSP, OUTPUT);
  digitalWrite(pinDSP, LOW);
  pinMode(pinFan, OUTPUT);
  analogWrite(pinFan, 0);
  pinMode(pinTemp, INPUT);
  pinMode(pinCharge, OUTPUT);
  digitalWrite(pinCharge, LOW);
  pinMode(pinBoost, OUTPUT);
  digitalWrite(pinBoost, stateHv);
  pinMode(pinBuck, OUTPUT);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  pinMode(pinMute, INPUT);
  pinMode(pinButton, INPUT);

  readVoltages();
  
  //digitalWrite(pinButton, LOW); // disable pullup
  // Was the button pressed?   If not, then we are charging or programming
  if ( digitalRead(pinButton) == LOW ) {
    stateLv = 1; // Turn on LV so we can read battery state
    digitalWrite(pinBuck, stateLv);
    digitalWrite(pinDSP, LOW);
    watchDogTimerDisabled = true;
    statePower = 0;
    //chargingLast = 1;
    delay(200);
  } else {
    // Hold MCU power on
    stateLv = 1; // Turn on LV so we can read battery state
    digitalWrite(pinBuck, stateLv);
    // Wait until power button is released to continue
    //while (digitalRead(pinButton) == HIGH);
    delay(100);
    doPowerOn();
  }
}

void receiveEvent(int nBytes) {
  // GhettoDSP is sending data
  float voltageBattMaxCharge;
  float vComp;
  bool audioDetectedLast = audioDetected;
  I2C_readAnything(audioDetected);
  I2C_readAnything(voltageBattMaxCharge);
  I2C_readAnything(vComp);
  I2C_readAnything(watchDogTimerDisabled);
  
  #ifdef DEBUG
    Trace (F("Got audioDetected / voltageBattMaxCharge / vComp: "));
    Trace (audioDetected);
    Trace (F(" / "));
    Trace (voltageBattMaxCharge);
    Trace (F(" / "));
    Traceln (vComp);
  #endif
  if ( voltageBattMaxCharge != settings.voltageBattMaxCharge || vComp != settings.vComp  ) {
    settings.voltageBattMaxCharge = voltageBattMaxCharge;
    settings.vComp = vComp;
    // Only write eeprom once
    if ( wroteEEP == false ) {
      writeEEPROM();
      wroteEEP = true;
    }
  }
  // Process no signal status
  if ( audioDetectedLast == 1 && audioDetected == 0 ) {
    lastNoAudioDetected = millis();
  } 
  if ( audioDetectedLast == 0 && audioDetected == 1 ) {
    lastNoAudioDetected = 0;
  }

  requestEventLast = millis();
  //Traceln ("receiveEvent()");

}

void requestEvent() {
  // Send voltages and status to ghettoDSP on request
  struct
  {
    float voltageBattAvg;
    //float voltageBuck;
    //float voltageBoost;
    //float voltageChargeOut;
    float tempC;
    bool statePower;
    bool stateCharging;
    uint8_t stateSys;
    uint8_t percentBattery;
    uint8_t stateBattery;
  } response;

  response.voltageBattAvg = voltageBattAvg;
  //response.voltageBuck = voltageBuck;
  //response.voltageBoost = voltageBoost;
  //response.voltageChargeOut = voltageChargeOut;
  response.tempC = tempCAvg;
  response.statePower = statePower;
  response.stateCharging = stateCharging;
  response.stateSys = stateSys;
  response.percentBattery = percentBatteryAvg;
  response.stateBattery = stateBattery;

  Wire.write ((byte *) &response, sizeof response);
  
  requestEventLast = millis(); // For watchdog
  //Traceln ("requestEvent()");
}

void handleLedState() {

  if ( stateCharging == 1 ) {
    stateSys = 5; // Charging / fade up
  } else if ( stateBattery == 0 ) {
    stateSys = 1; // Low battery shutdown / slow blink
  } else if ( stateBattery == 1 ) {
    stateSys = 2; // Low battery / fast blink
  } else if ( statePower == 1 && stateLv == 1 && stateHv == 0 ) {
    stateSys = 4; // Power on standby // fade up and down
  } else if ( statePower == 1 && stateLv == 1 && stateHv == 1 ) {
    stateSys = 3; // Power on normal / on solid
  } else if ( statePower == 0 ) {
    stateSys = 6; // Power off / slow fade up and down
  }
  switch ( stateSys ) {
    case 1:
      if (millis() - 500 >= ledTimer ) {
        digitalWrite(pinLed, !digitalRead(pinLed));
        ledTimer = millis();
      }
      break;
    case 2:
      if (millis() - 150  >= ledTimer) {
        digitalWrite(pinLed, !digitalRead(pinLed));
        ledTimer = millis();
      }
      break;
    case 3:
      digitalWrite(pinLed, HIGH);
      break;
    case 4: // Fade normal
      if ( millis() - 2  > fadeLedLast ) {
        if ( fadeLedDir == 1 )
        {
          if ( fadeLed < 255 )
          {
            fadeLed++;
          } else {
            fadeLedDir = 0;
          }
        } else {
          if ( fadeLed > 1 )
          {
            fadeLed--;
          } else {
            fadeLedDir = 1;
          }
        }
        analogWrite(pinLed, fadeLed);
        fadeLedLast = millis();
      }
      break;
    case 5: // Fade up
      if ( millis() - 4  > fadeLedLast ) {
        if ( fadeLed < 255 )
        {
          fadeLed++;
        } else {
          fadeLed = 0;
        }
        analogWrite(pinLed, fadeLed);
        fadeLedLast = millis();
      }
      break;
    case 6: // Fade slow
      if ( millis() - 8  > fadeLedLast ) {
        if ( fadeLedDir == 1 )
        {
          if ( fadeLed < 255 )
          {
            fadeLed++;
          } else {
            fadeLedDir = 0;
          }
        } else {
          if ( fadeLed > 1 )
          {
            fadeLed--;
          } else {
            fadeLedDir = 1;
          }
        }
        analogWrite(pinLed, fadeLed);
        fadeLedLast = millis();
      }
      break;
  }
}

void calcPwrState() {
  // Voltage above cutoff?
  if ( voltageBattAvg > cutonHv )
  {
    stateBattery = 2;
    // Audio signal detected?
    if ( audioDetected && statePower == 1 )
    {
      stateHv = HIGH; // Turn on HV
    } else {
      if ( lastNoAudioDetected > 0 
        && millis() > standbyDelay 
        && millis() - standbyDelay > lastNoAudioDetected 
      )
      {
        // No audio, turn off amp
        stateHv = LOW;
      }
    }
  }
  if (  stateHv == HIGH && voltageBattAvg < cutoffHv  )
  {
    stateHv = LOW; // Turn off HV
    stateBattery = 1;
  }
  // Battery critically low.  Shut everything down.
  if (  voltageBattAvg < cutoffLv && voltageBattAvg < cutoffLv && statePower == 1 && !stateCharging  )
  {
    doDebug("Shutdown: Battery critical");
    doLog();
    blinkLed(5);
    doPowerOff();
    stateBattery = 0;
  } else {
    
  }
  // Stuff to check always
  // If sufficiant charge voltage, turn on LV (if not already charged)
  if ( voltageChargeOut > settings.voltageBattMaxCharge && !shitterWasFull ) {
    stateLv = 1;
  }
  // Check for over temperature
  if (  tempCAvg > tempShutdown && millis() > 5000 ) {
    doDebug("Shutdown: Over temp");
    blinkLed(4);
    doPowerOff();
  }
  // Watchdog timeout poweroff (if DSP board is not responding, shut it off)
  if ( watchDogTimer > 0 
        && !watchDogTimerDisabled 
        && stateLv == 1 
        && millis() > watchDogTimer // Don't go negative and false trigger
        && millis() - watchDogTimer > requestEventLast 
  ) {
    doDebug("Shutdown: Watchdog!");
    blinkLed(3);
    doPowerOff();
  }

}

void handleFanState() {
  if ( millis() - 5000 >= lastFanSpeedChange ) {
    #ifdef TEMP
    if ( stateCharging == 1 ) {
      // Run full blast when charging to cool the buck/boost converter
      fanSpeed = fanSpeedHigh;
    } else if ( tempCAvg >= fanTempLow && tempCAvg < fanTempMed ) {
      // If not already on, give the fan a bump start
      if ( fanSpeed == 0 ) {
        analogWrite(pinFan,255);
        delay(50);
      }
      fanSpeed = fanSpeedLow;
    } else if ( tempCAvg >= fanTempMed && tempCAvg < fanTempHigh ) {
      fanSpeed = fanSpeedMed;
    } else if ( tempCAvg >= fanTempHigh ) {
      fanSpeed = fanSpeedHigh;
    } else {
      fanSpeed = 0;
    }
    #else
    fanSpeed = fanSpeedHigh;
    #endif
    analogWrite(pinFan,fanSpeed);
    lastFanSpeedChange = millis();
  }
}

void handlePwrState() {
  
  // Make sure low voltage supply is on during charging for fan
  if ( stateCharging ) { stateLv = HIGH; };
  
  if ( stateHv != digitalRead(pinBoost) )
  {
    digitalWrite(pinBoost, stateHv);
    lastPwrStateChange = millis();
  }
  if ( stateLv != digitalRead(pinBuck)  )
  {
    digitalWrite(pinBuck, stateLv);
    lastPwrStateChange = millis();
  }

  if ( lastNoAudioDetected > 0 && millis() - lastNoAudioDetected > autoShutOffDelay && statePower == 1 )
  {
    // Inactivity shutdown
    doDebug("Shutdown: Inactivity");
    blinkLed(2);
    doPowerOff();
  }
}

void readVoltages() {

  voltageBatt = analogRead(pinVbatt) / settings.vComp;
  voltageBuck = analogRead(pinVbuck) / settings.vComp;
  voltageBoost = analogRead(pinVboost) / settings.vComp;
  voltageChargeOut = analogRead(pinVchargeOut) / settings.vComp;

  #ifdef REV05
  voltageChargeIn = analogRead(pinVchargeIn) / settings.vComp;
  #endif

  if ( voltageBattAvg == 0 )
  {
    voltageBattAvg = voltageBatt;
  }
  // Calculate average
  if ( millis() > 5000 & millis() - 5000 > voltageBattAvgLast )
  {
    voltageBattAvg = voltageBattAvgSum / voltageBattAvgCount;
    voltageBattAvgSum = 0;
    voltageBattAvgCount = 0;
    voltageBattAvgLast = millis();
  } else {
    voltageBattAvgSum = voltageBattAvgSum + voltageBatt;
    voltageBattAvgCount++;
  }
  if ( voltageMax == -1 || voltageMin == -1 )
  {
    voltageMax = voltageBattAvg;
    voltageMin = voltageBattAvg;
  }
  if ( voltageBattAvg > voltageMax ) {
    voltageMax = voltageBattAvg;
    voltageDev = (voltageMax / voltageMin) - 1;
  }
  if ( voltageBattAvg < voltageMin ) {
    voltageMin = voltageBattAvg;
    voltageDev = (voltageMax / voltageMin) - 1;
  }

  if ( !stateCharging ) {
    if ( voltageBatt >= voltageBattRestingFull ) {
      percentBattery = 100;
    } else {
      if ( voltageBatt - voltageBattRestingEmpty > 0 ) {
        percentBattery = (voltageBatt - voltageBattRestingEmpty) / (voltageBattRestingFull - voltageBattRestingEmpty) * 100;
      } else {
        percentBattery = 0;
      }
    }
  } else {
    if ( voltageBatt >= voltageBattChargingFull ) {
      percentBattery = 100;
    } else {
      if ( voltageBatt - voltageBattChargingEmpty > 0 ) {
        percentBattery = (voltageBatt - voltageBattChargingEmpty) / (voltageBattChargingFull - voltageBattChargingEmpty) * 100;
      } else {
        percentBattery = 0;
      }
    }
  }


  // Calculate battery percentage averaged over 10 seconds to reduce fluctuations
  // Unless charging, only do this when amp is active because the voltage rises when it's off
  if ( (millis() > 10000 && millis() - 10000 > percentBatteryAvgLast) && (stateHv == 1 || stateCharging == 1) )
  {
    // Wait for things to settle before reporting battery %
    percentBatteryAvg = uint8_t(percentBatteryAvgSum / percentBatteryAvgCount);
    // Don't let battery % fluctuate up when not charging or down when charging
    // unless difference is great enough
//    if ( !stateCharging && percentBatteryAvg > percentBatteryPrevious && abs(percentBatteryAvg - percentBatteryPrevious) < 25 ) {
//      percentBatteryAvg = percentBatteryPrevious;
//    } else if ( stateCharging && percentBatteryAvg < percentBatteryPrevious && abs(percentBatteryAvg - percentBatteryPrevious) < 10 ) {
//      percentBatteryAvg = percentBatteryPrevious;
//    } else {
//      percentBatteryPrevious = percentBatteryAvg;
//    }
    
    percentBatteryAvgSum = 0;
    percentBatteryAvgCount = 0;
    percentBatteryAvgLast = millis();
  } else {
    percentBatteryAvgSum = percentBatteryAvgSum + percentBattery;
    percentBatteryAvgCount++;
  }

  // Get temperature
  #if TEMP
    tempC = ((float(analogRead(pinTemp))*5.0/1024)-0.5)*100;
  #endif

  if ( tempCAvg == 0 ) {
    tempCAvg = tempC;
  }

  if ( millis() > 5000 && millis() - 5000 > tempCAvgLast )
  {
    tempCAvg = tempCAvgSum / tempCAvgCount;
    tempCAvgSum = 0;
    tempCAvgCount = 0;
    tempCAvgLast = millis();
  } else {
    tempCAvgSum = tempCAvgSum + tempC;
    tempCAvgCount++;
  }
  
}

void doPowerOff() {
  // Pop avoidance:
  // Shut off amp first, then wait a few seconds before powering off the DSP
  digitalWrite(pinBoost, LOW);
  stateHv = 0;
  statePower = 0;
  delay(2000);
  digitalWrite(pinDSP, LOW);
  digitalWrite(pinBuck, LOW);
  stateLv = 0;
  audioDetected = 0;
  // Let the caps discharge (turnoff pop)
  delay(1500);
  lastPwrStateChange = millis();
  watchDogTimerDisabled = true;
}

void doPowerOn() {
  digitalWrite(pinDSP, HIGH);
  stateLv = HIGH;
  digitalWrite(pinBuck, HIGH);
  statePower = 1;
  delay(500);
  if ( signalDetectMode == 2 ) {
    audioDetected = 1; // Always on
  }
  lastPwrStateChange = millis();
  lastNoAudioDetected = millis();
  requestEventLast = millis();
}

void checkPowerButton() {
  if ( digitalRead(pinButton) == HIGH && millis() > 5000 ) {
    // Wait until release
    while (digitalRead(pinButton) == HIGH);
    //debounce
    delay(10);
    if ( statePower == 1 ) {
      doDebug("Shutdown: Button push");
      blinkLed(1);
      doPowerOff();
    } else {
      doPowerOn();
    }
  }
}

void handleCharging() {
  #ifdef REV05
  if ( stateCharging == 1 && voltageChargeIn < voltageChargeInMin ) {
    digitalWrite(pinCharge, LOW);
    stateCharging = 0;
    doDebug("Charge: Stopping for low input voltage");
  }
  #endif
  // Only run once per 30 seconds while charging
  if ( (( millis() > 30000 ) && ( millis() - 30000 > chargingLast )) || !chargingLast || !stateCharging ) {
    chargingLast = millis();
    // Disable charging briefly to check voltages
    digitalWrite(pinCharge, LOW);
    stateCharging = 0;
    delay(100);
    readVoltages();
    // If voltage dropped below the threshold, clear the shitterWasFull flag and allow charging again
    if ( voltageBatt < voltageBattResumeCharge && shitterWasFull == true ) {
      shitterWasFull = false;
      doDebug("Charge: Reset shitter flag");
    }
    // Is the charging voltage high enough and we're not overheating
    if (  voltageChargeOut > voltageBatt 
           && voltageBatt > voltageBattMinCharge 
           && voltageBatt < settings.voltageBattMaxCharge 
           && !shitterWasFull 
           #ifdef REV05
           && voltageChargeIn > voltageChargeInMin
           #endif
           && tempC < tempShutdown ) {
      // Start charging
      digitalWrite(pinCharge, HIGH);
      stateCharging = 1;
      //doDebug("Charge: Begin1");
    } else if ( voltageBattAvg >= settings.voltageBattMaxCharge ) {
      // Battery is full.   Quit charging until voltage drops below the resume threshold
      shitterWasFull = true;
      doDebug("Charge: Full + set shitter flag");
      // If we're not powered on, shut down so it doesn't stay secretly on after charger removal.
//      if ( !statePower ) {
//        doPowerOff();
//      }
    } else if ( !statePower && ( shitterWasFull || ( voltageChargeOut < settings.voltageBattMaxCharge )  ) && stateLv ) {
      // Charging power removed or charging finished while in power off state
      doDebug("Shutdown: No longer charging");
      blinkLed(6);
      doPowerOff();
    }
    // Shut down if not plugged into adequate charger and not charging in poweroff state.
    // This is to protect the battery when charging is done or stopped for whatever reason
    //if ( voltageChargeOut < settings.voltageBattMaxCharge && statePower == 0 ) {
      
    //}
   }
}

void loop() {
    // Measure power supply voltages
    readVoltages();

    // Calculate optimum power state
    calcPwrState();

    // Control power state
    handlePwrState();

    // Measure power supply voltages
    readVoltages();

  // Handle these 500ms tasks
  if ( millis() > 500 && millis() - 500 >= last500mSecTask  ) {
    
    // Reset HW watchdog
    wdt_reset();



    // See if we need to charge
    if ( stateLv ) {
      handleCharging();
    }



    // Control fan state
    handleFanState();
  
    last500mSecTask = millis();
  }

  // Handle LED state
  handleLedState();

  checkPowerButton();

  // Check if signal is detected in mute pin mode
  checkMutePin();

  // Debug
  #ifdef DEBUG
  if ( millis() - 1000  > lastLogLine ) {
    doLog();
    lastLogLine = millis();
  }
  #endif
}

void checkMutePin() {
  if ( signalDetectMode == 0 && standbyDelay > 0 ) {
    audioDetected = digitalRead(pinMute);
    if ( audioDetected == 0 ) {
      lastNoAudioDetected = millis();
    } else {
      lastNoAudioDetected = 0;
    }
  }
}

void doDebug(const char *text) {
  #ifdef DEBUG
    Traceln(text);
  #endif
}

void doLog() {
  #ifdef DEBUG
    #ifdef REV05
    Trace (F("Vbatt/Avg/Vbuck/Vboost/VchI/VchO: "));
    #else
    Trace (F("Vbatt/Avg/Vbuck/Vboost/Vch: "));
    #endif
    Trace (voltageBatt);
    Trace (F("/"));
    Trace (voltageBattAvg);
    Trace (F("/"));
    Trace (voltageBuck);
    Trace (F("/"));
    Trace (voltageBoost);
    #ifdef REV05
    Trace (F("/"));
    Trace (voltageChargeIn);
    #endif
    Trace (F("/"));
    Trace (voltageChargeOut);
    Trace (F(" stateHv/LV/DSP/CH/SF: "));
    //Trace (digitalRead(pinBoost));
    Trace (stateHv);
    //Trace (digitalRead(pinBuck));
    Trace (stateLv);
    Trace (digitalRead(pinDSP));
    Trace (stateCharging);
    Trace (shitterWasFull);
    Trace (F(" aD: "));
    Trace (audioDetected);
    Trace (F(" sS: "));
    Trace (stateSys);
    Trace (F(" tC: "));
    Trace (tempC);
    Trace (F(" sB: "));
    Trace (stateBattery);
    Trace (F(" mC: "));
    Trace (settings.voltageBattMaxCharge);
    Trace (F(" vC: "));
    Traceln (settings.vComp);
  #endif
}

void blinkLed(int n) {
  while(n > 0 ) {
    digitalWrite(pinLed,false);
    delay(300);
    digitalWrite(pinLed,true);
    delay(300);
    n--;
  }
  digitalWrite(pinLed,false);
}

void readEEPROM() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t = 0; t < sizeof(settings); t++) {
      *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
    }
  } else {
    #ifdef DEBUG
      Traceln("EEPROM Load failed");
    #endif
  }
}

void writeEEPROM() {
  #ifdef DEBUG
    Traceln(F("Writing EEPROM"));
  #endif
  for (unsigned int t = 0; t < sizeof(settings); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
}
