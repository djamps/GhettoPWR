// GhettoPWR_new.ino
#include "GhettoPWR.h"
#include <avr/wdt.h>
#include <SPI.h>
#include <Wire.h>
#include <I2C_Anything.h>
#include <EEPROM.h>

// Global objects
PinDefinitions pins;
Configuration sysConf;
SystemStateStruct sysState;
EEPROMSettings eepromSettings = {CONFIG_VERSION, sysConf.voltageBattMaxCharge, sysConf.vComp};

// Timing variables
unsigned long lastTaskTime = 0;
unsigned long lastNoAudioDetected = 0;
unsigned long lastFanSpeedChange = 0;
unsigned long lastPwrStateChange = millis();
unsigned long requestEventLast = 0;
unsigned long chargingLast = 0;
unsigned long ledTimer = millis();

// LED control variables
int fadeLed = 0;
unsigned long fadeLedLast = millis();
bool fadeLedDir = true;

// Averaging variables
float voltageBattAvgSum = 0;
unsigned int voltageBattAvgCount = 0;
unsigned long voltageBattAvgLast = millis();
unsigned long percentBatteryAvgSum = 0;
unsigned long percentBatteryAvgCount = 0;
unsigned long percentBatteryAvgLast = millis();
float tempCAvgSum = 0;
unsigned long tempCAvgCount = 0;
unsigned long tempCAvgLast = millis();

// Debug helpers
#if DEBUG
  #define beginDebug() do { SPI.begin(); SPI.setClockDivider(SPI_CLOCK_DIV8); } while (0)
  #define Trace(x) SPIdebug.print(x)
  #define Trace2(x, y) SPIdebug.print(x, y)
  #define Traceln(x) SPIdebug.println(x)
  #define Traceln2(x, y) SPIdebug.println(x, y)
  #define TraceFunc() do { SPIdebug.print(F("In function: ")); SPIdebug.println(__PRETTY_FUNCTION__); } while (0)
  
  class tSPIdebug : public Print {
    public:
      virtual size_t write(const byte c) {
        SPI.transfer(c);
        return 1;
      }
  };
  
  tSPIdebug SPIdebug;
#else
  #define beginDebug() ((void)0)
  #define Trace(x) ((void)0)
  #define Trace2(x, y) ((void)0)
  #define Traceln(x) ((void)0)
  #define Traceln2(x, y) ((void)0)
  #define TraceFunc() ((void)0)
#endif

void setup() {
  wdt_enable(WDTO_8S);
  wdt_reset();
  
  beginDebug();
  debugMessage("*******************************");

  if ( sysConf.useEEPROM ) {
    if ( readEEPROM() ) {
      // Load settings from EEPROM
      debugMessage("Got settings from EEPROM");
      sysConf.vComp = eepromSettings.vComp;
      sysConf.vComp = eepromSettings.voltageBattMaxCharge; 
    } else {
      // Initialize EEPROM settings
      debugMessage("Initializing EEPROM");
      eepromSettings.vComp = sysConf.vComp;
      eepromSettings.voltageBattMaxCharge = sysConf.vComp; 
      writeEEPROM();
    }
  }
  
  // Initialize I2C
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  // Initialize pins
  initializePins();
  
  // Read initial voltages
  readVoltages(true);
  
  // Check if button was pressed during startup
  if (digitalRead(pins.BUTTON) == LOW) {
    // Button not pressed - enter programming/charging mode
    sysState.lvPower = true;
    digitalWrite(pins.BUCK, sysState.lvPower);
    digitalWrite(pins.DSP, LOW);
    sysState.watchdogDisabled = true;
    sysState.dspPower = false;
    sysState.systemState = STATE_POWER_OFF;
    delay(200);
  } else {
    // Normal startup - power on system
    sysState.lvPower = true;
    digitalWrite(pins.BUCK, sysState.lvPower);
    delay(100);
    powerOn();
  }
}

void loop() {
  // Read and process voltages
  readVoltages(false);
  
  // Calculate optimum power state
  calculatePowerState();
  
  // Control power state
  handlePowerState();
  
  // Handle periodic tasks
  if (millis() - TASK_INTERVAL >= lastTaskTime) {
    wdt_reset();
    
    // Handle charging if LV power is on
    if (sysState.lvPower) {
      handleCharging();
    }
    
    // Control fan state
    handleFanState();
    
    lastTaskTime = millis();
  }
  
  // Handle LED state
  handleSystemState();
  
  // Check power button
  checkPowerButton();
  
  // Check signal detection in mute pin mode
  checkMutePin();
  
  // Debug logging
  #if DEBUG
    if (millis() - 1000 > lastTaskTime) {
      logSystemState();
      lastTaskTime = millis();
    }
  #endif
}

// Initialize all pins
void initializePins() {
  pinMode(pins.DSP, OUTPUT);
  digitalWrite(pins.DSP, LOW);
  pinMode(pins.FAN, OUTPUT);
  analogWrite(pins.FAN, 0);
  pinMode(pins.TEMP, INPUT);
  pinMode(pins.CHARGE, OUTPUT);
  digitalWrite(pins.CHARGE, LOW);
  pinMode(pins.BOOST, OUTPUT);
  digitalWrite(pins.BOOST, sysState.hvPower);
  pinMode(pins.BUCK, OUTPUT);
  pinMode(pins.LED, OUTPUT);
  digitalWrite(pins.LED, HIGH);
  pinMode(pins.MUTE, INPUT);
  pinMode(pins.BUTTON, INPUT);
  #if REV05
  pinMode(pins.VCHARGEIN,INPUT);
  #endif
}

// I2C receive event handler
void receiveEvent(int nBytes) {
  bool audioDetectedLast = sysState.audioDetected;
  float voltageBattMaxCharge_dsp;
  float vComp_dsp;
  
  I2C_readAnything(sysState.audioDetected);
  I2C_readAnything(voltageBattMaxCharge_dsp);
  I2C_readAnything(vComp_dsp);
  I2C_readAnything(sysState.watchdogDisabled);
  
  #if DEBUG
    Trace(F("Got audioDetected / voltageBattMaxCharge / vComp: "));
    Trace(sysState.audioDetected);
    Trace(F(" / "));
    Trace(voltageBattMaxCharge_dsp);
    Trace(F(" / "));
    Traceln(vComp_dsp);
  #endif
  
  // Update EEPROM settings if changed
  if (sysConf.allowOverride == true && ( sysConf.voltageBattMaxCharge != voltageBattMaxCharge_dsp || sysConf.vComp != vComp_dsp )) {
    sysConf.voltageBattMaxCharge = voltageBattMaxCharge_dsp;
    sysConf.vComp = vComp_dsp;
  }
  
  // Process audio detection status
  if (audioDetectedLast && !sysState.audioDetected) {
    lastNoAudioDetected = millis();
  } else if (!audioDetectedLast && sysState.audioDetected) {
    lastNoAudioDetected = 0;
  }
  
  requestEventLast = millis();
}

// I2C request event handler
void requestEvent() {
  struct Response {
    float voltageBattAvg;
    float temperature;
    bool dspPower;
    bool charging;
    uint8_t systemState;
    uint8_t batteryPercent;
    uint8_t batteryState;
  } response;
  
  response.voltageBattAvg = sysState.voltageBattAvg;
  response.temperature = sysState.temperatureAvg;
  response.dspPower = sysState.dspPower;
  response.charging = sysState.charging;
  response.systemState = sysState.systemState;
  response.batteryPercent = sysState.batteryPercentAvg;
  response.batteryState = sysState.batteryState;
  
  Wire.write((byte*)&response, sizeof(response));
  requestEventLast = millis();
}

// Read voltages and calculate averages
void readVoltages(bool resetAvg) {
  // Read raw voltages
  sysState.voltageBatt = analogRead(pins.VBATT) / sysConf.vComp;
  sysState.voltageBuck = analogRead(pins.VBUCK) / sysConf.vComp;
  sysState.voltageBoost = analogRead(pins.VBOOST) / sysConf.vComp;
  sysState.voltageChargeOut = analogRead(pins.VCHARGEOUT) / sysConf.vComp;
  
#if REV05
  sysState.voltageChargeIn = analogRead(pins.VCHARGEIN) / sysConf.vComp;
#endif

  // Reset averaging if requested
  if (resetAvg) {
    sysState.voltageBattAvg = sysState.voltageBatt;
    voltageBattAvgLast = millis();
    voltageBattAvgSum = 0;
    voltageBattAvgCount = 0;
  }
  
  // Calculate battery voltage average
  if (millis() > AVERAGING_INTERVAL && millis() - AVERAGING_INTERVAL > voltageBattAvgLast) {
    sysState.voltageBattAvg = voltageBattAvgSum / voltageBattAvgCount;
    voltageBattAvgSum = 0;
    voltageBattAvgCount = 0;
    voltageBattAvgLast = millis();
  } else {
    voltageBattAvgSum += sysState.voltageBatt;
    voltageBattAvgCount++;
  }
  
  // Calculate battery percentage
  if (!sysState.charging) {
    // Resting voltage calculation
    if (sysState.voltageBatt >= sysConf.voltageBattRestingFull) {
      sysState.batteryPercent = 100;
    } else {
      float voltageDiff = sysState.voltageBatt - sysConf.voltageBattRestingEmpty;
      if (voltageDiff > 0) {
        float range = sysConf.voltageBattRestingFull - sysConf.voltageBattRestingEmpty;
        sysState.batteryPercent = (voltageDiff / range) * 100;
      } else {
        sysState.batteryPercent = 0;
      }
    }
  } else {
    // Charging voltage calculation
    if (sysState.voltageBatt >= sysConf.voltageBattChargingFull) {
      sysState.batteryPercent = 100;
    } else {
      float voltageDiff = sysState.voltageBatt - sysConf.voltageBattChargingEmpty;
      if (voltageDiff > 0) {
        float range = sysConf.voltageBattChargingFull - sysConf.voltageBattChargingEmpty;
        sysState.batteryPercent = (voltageDiff / range) * 100;
      } else {
        sysState.batteryPercent = 0;
      }
    }
  }
  
  // Calculate battery percentage average
  if ((millis() > PERCENT_AVG_INTERVAL && millis() - PERCENT_AVG_INTERVAL > percentBatteryAvgLast) && 
      (sysState.hvPower || sysState.charging)) {
    sysState.batteryPercentAvg = percentBatteryAvgSum / percentBatteryAvgCount;
    percentBatteryAvgSum = 0;
    percentBatteryAvgCount = 0;
    percentBatteryAvgLast = millis();
  } else {
    percentBatteryAvgSum += sysState.batteryPercent;
    percentBatteryAvgCount++;
  }
  
  // Read temperature
  #if TEMP_SENSOR
    sysState.temperature = ((float(analogRead(pins.TEMP)) * 5.0 / 1024) - 0.5) * 100;
  #endif
  
  // Calculate temperature average
  if (millis() > AVERAGING_INTERVAL && millis() - AVERAGING_INTERVAL > tempCAvgLast) {
    sysState.temperatureAvg = tempCAvgSum / tempCAvgCount;
    tempCAvgSum = 0;
    tempCAvgCount = 0;
    tempCAvgLast = millis();
  } else {
    tempCAvgSum += sysState.temperature;
    tempCAvgCount++;
  }
}

// Calculate optimal power state based on conditions
void calculatePowerState() {
  // Voltage above cutoff?
  if (sysState.voltageBattAvg > sysConf.cutonHv) {
    sysState.batteryState = BATTERY_GOOD;
    
    // Audio signal detected?
    if (sysState.audioDetected && sysState.dspPower) {
      sysState.hvPower = HIGH; // Turn on HV
    } else {
      // No audio, turn off amp after standby delay
      if (lastNoAudioDetected > 0 && millis() > sysConf.standbyDelay && 
          millis() - sysConf.standbyDelay > lastNoAudioDetected) {
        sysState.hvPower = LOW;
      }
    }
  }
  
  // Voltage below cutoff - turn off HV and set battery state
  if (sysState.voltageBattAvg < sysConf.cutoffHv) {
    if ( sysState.hvPower ) {
      sysState.hvPower = LOW;
    }
    sysState.batteryState = BATTERY_LOW;
  }
  
  // Battery critically low - shut everything down
  if (sysState.voltageBattAvg < sysConf.cutoffLv && sysState.dspPower && !sysState.charging) {
    debugMessage("Shutdown: Battery critical");
    logSystemState();
    blinkLed(BLINK_CRITICAL_BATTERY);
    sysState.batteryState = BATTERY_CRITICAL;
    powerOff();
  }
  
  // If sufficient charge voltage, turn on LV (if not already charged)
  if (sysState.voltageChargeOut > sysConf.voltageBattMaxCharge && !sysState.chargeFull) {
    sysState.lvPower = true;
  }
  
  // Check for over temperature
  if (sysState.temperatureAvg > sysConf.tempShutdown && millis() > TEMP_READING_DELAY) {
    debugMessage("Shutdown: Over temp");
    blinkLed(BLINK_OVER_TEMPERATURE);
    powerOff();
  }
  
  // Watchdog timeout poweroff
  if (sysConf.watchDogTimer > 0 && !sysState.watchdogDisabled && sysState.lvPower && 
      millis() > sysConf.watchDogTimer && millis() - sysConf.watchDogTimer > requestEventLast) {
    debugMessage("Shutdown: Watchdog!");
    blinkLed(BLINK_WATCHDOG_TIMEOUT);
    powerOff();
  }
}

// Handle power state changes
void handlePowerState() {
  // Make sure low voltage supply is on during charging for fan
  if (sysState.charging && !sysState.lvPower) {
    sysState.lvPower = HIGH;
  }
  
  // Update boost converter if state changed
  if (sysState.hvPower != digitalRead(pins.BOOST)) {
    digitalWrite(pins.BOOST, sysState.hvPower);
    lastPwrStateChange = millis();
  }
  
  // Update buck converter if state changed
  if (sysState.lvPower != digitalRead(pins.BUCK)) {
    digitalWrite(pins.BUCK, sysState.lvPower);
    lastPwrStateChange = millis();
  }
  
  // Inactivity shutdown
  if (lastNoAudioDetected > 0 && millis() - lastNoAudioDetected > sysConf.autoShutOffDelay && sysState.dspPower) {
    debugMessage("Shutdown: Inactivity");
    blinkLed(BLINK_INACTIVITY);
    powerOff();
  }
}

// Handle charging logic
void handleCharging() {
#if REV05
  // Check for low input voltage
  if (sysState.charging && sysState.voltageChargeIn < sysConf.voltageChargeInMin) {
    digitalWrite(pins.CHARGE, LOW);
    sysState.charging = false;
    sysState.chargeFault = true;
    debugMessage("Charge: Stopping for low input voltage");
  }
#endif

  // Only run once per 30 seconds while charging
  if ((millis() > CHARGING_CHECK_DELAY && millis() - CHARGING_CHECK_DELAY > chargingLast) || !chargingLast || !sysState.charging) {
    debugMessage("Charge: Checking");
    chargingLast = millis();
    
#if REV04B
    // Disable charging briefly to check voltages
    digitalWrite(pins.CHARGE, LOW);
    sysState.charging = false;
    delay(100);
#endif

    readVoltages(false);
    
    // If voltage dropped below the threshold, clear the full flag
    if (sysState.voltageBatt < sysConf.voltageBattResumeCharge && sysState.chargeFull) {
      sysState.chargeFull = false;
      debugMessage("Charge: Reset full flag");
    }
    
    // Check if we should start charging
    sysState.canCharge = sysState.voltageChargeOut > sysState.voltageBatt &&
                       sysState.voltageBatt > sysConf.voltageBattMinCharge &&
                       sysState.voltageBatt < sysConf.voltageBattMaxCharge &&
                       !sysState.chargeFull &&
                       sysState.temperature < sysConf.tempShutdown;
#if REV05
    sysState.canCharge = sysState.canCharge && sysState.voltageChargeIn > sysConf.voltageChargeInMin;
#endif

    if (sysState.canCharge) {
      // Start charging
      if ( !sysState.charging ) {
        debugMessage("Charge: Begin");
      }
      digitalWrite(pins.CHARGE, HIGH);
      sysState.charging = true;
    } else if (sysState.voltageBattAvg >= sysConf.voltageBattMaxCharge) {
      // Battery is full
      if ( sysState.charging ) {
        debugMessage("Charge: Stopping, battery full");
      }
      digitalWrite(pins.CHARGE, LOW);
      sysState.charging = false;
      sysState.chargeFull = true;
    } else if (!sysState.dspPower && (sysState.chargeFull || sysState.voltageChargeOut < sysConf.voltageBattMaxCharge) && sysState.lvPower) {
      // Charging power removed or finished while in power off state
      debugMessage("Shutdown: No longer charging");
      blinkLed(BLINK_NO_LONGER_CHARGING);
      powerOff();
    } else {
      debugMessage("Charge: No conditions met!");
    }
  }
}

// Handle fan control based on temperature
void handleFanState() {
  if (millis() - FAN_UPDATE_INTERVAL >= lastFanSpeedChange) {
    uint8_t newFanSpeed = 0;
    
#if TEMP_SENSOR
    if (sysState.charging == true) {
          sysState.fanState = FAN_HIGH;
    } else if (sysState.temperatureAvg >= sysConf.fanTempHigh) {
          sysState.fanState = FAN_HIGH;
    } else if (sysState.temperatureAvg >= sysConf.fanTempMed) {
          sysState.fanState = FAN_MED;
    } else if (sysState.temperatureAvg >= sysConf.fanTempLow) {
      // If not already on, give the fan a bump start
      if (sysState.fanState == FAN_OFF) {
        analogWrite(pins.FAN, FAN_HIGH);
        delay(50);
      }
      sysState.fanState = FAN_LOW;
    } else {
      sysState.fanState = FAN_OFF;
    }
#else
    sysState.fanState = FAN_HIGH;
#endif
    analogWrite(pins.FAN, sysState.fanState);
    lastFanSpeedChange = millis();
  }
}

// Helper function for LED fading
void updateFade(int interval, int minValue, int maxValue, bool resetAtTop) {
  if (millis() - interval > fadeLedLast) {
    if (fadeLedDir) {
      if (fadeLed < maxValue) {
        fadeLed++;
      } else {
        if (resetAtTop) {
          fadeLed = 0;
        } else {
          fadeLedDir = false;
        }
      }
    } else {
      if (fadeLed > minValue) {
        fadeLed--;
      } else {
        fadeLedDir = true;
      }
    }
    
    analogWrite(pins.LED, fadeLed);
    fadeLedLast = millis();
  }
}

// Handle LED status indicators
void handleSystemState() {
  // Determine and set system state
  if (sysState.charging) {
    sysState.systemState = STATE_CHARGING;
  } else if (sysState.batteryState == BATTERY_CRITICAL) {
    sysState.systemState = STATE_LOW_BATTERY_SHUTDOWN;
  } else if (sysState.batteryState == BATTERY_LOW) {
    sysState.systemState = STATE_LOW_BATTERY_WARNING;
  } else if (sysState.dspPower && sysState.lvPower && !sysState.hvPower) {
    sysState.systemState = STATE_POWER_ON_STANDBY;
  } else if (sysState.dspPower && sysState.lvPower && sysState.hvPower) {
    sysState.systemState = STATE_POWER_ON_NORMAL;
  } else if (!sysState.dspPower) {
    sysState.systemState = STATE_POWER_OFF;
  }
  
  // Handle LED based on system state
  switch (sysState.systemState) {
    case STATE_LOW_BATTERY_SHUTDOWN: // Slow blink
      if (millis() - 500 >= ledTimer) {
        digitalWrite(pins.LED, !digitalRead(pins.LED));
        ledTimer = millis();
      }
      break;
      
    case STATE_LOW_BATTERY_WARNING: // Fast blink
      if (millis() - 150 >= ledTimer) {
        digitalWrite(pins.LED, !digitalRead(pins.LED));
        ledTimer = millis();
      }
      break;
      
    case STATE_POWER_ON_NORMAL: // Solid on
      digitalWrite(pins.LED, HIGH);
      break;
      
    case STATE_POWER_ON_STANDBY: // Normal fade
      updateFade(2, 1, 255);
      break;
      
    case STATE_CHARGING: // Fade up
      updateFade(4, 1, 255, true);
      break;
      
    case STATE_POWER_OFF: // Slow fade
      updateFade(8, 1, 255);
      break;
  }
}

// Power off sequence
void powerOff() {
  // Pop avoidance: Shut off amp first
  digitalWrite(pins.BOOST, LOW);
  sysState.hvPower = false;
  sysState.dspPower = false;
  sysState.systemState = STATE_POWER_OFF;
  delay(POWER_OFF_DELAY);
  wdt_reset();
  sysState.lvPower = false;
  sysState.audioDetected = false;
  lastPwrStateChange = millis();
  sysState.watchdogDisabled = true;
  digitalWrite(pins.DSP, LOW);
  digitalWrite(pins.BUCK, LOW);
  delay(CAP_DISCHARGE_DELAY);
  
  // nothing gets executed from here down unless getting power from the programmer (OR CHARGER!!! BE CAREFUL!!!).
  // re-enable buck converter and log voltages/states until power removed:
#if DEBUG
  if ( !(sysState.systemState == STATE_CHARGING) ) {
    // Let the caps discharge (turnoff pop)
    sysState.lvPower = true;
    digitalWrite(pins.BUCK, HIGH);
    while(1) {
      wdt_reset();
      readVoltages(false);
      logSystemState();
      delay(2000);
    }
  }
#endif
}

// Power on sequence
void powerOn() {
  digitalWrite(pins.DSP, HIGH);
  sysState.lvPower = true;
  digitalWrite(pins.BUCK, sysState.lvPower);
  sysState.dspPower = true;
  sysState.systemState = STATE_POWER_ON_STANDBY;
  delay(POWER_ON_DELAY);
  
  readVoltages(true);
  
  if (sysConf.signalDetectMode == 2) {
    sysState.audioDetected = true; // Always on
  }
  
  lastPwrStateChange = millis();
  lastNoAudioDetected = millis();
  requestEventLast = millis();
}

// Check power button
void checkPowerButton() {
  if (digitalRead(pins.BUTTON) == HIGH && millis() > BUTTON_READING_DELAY) {
    // Wait until release
    while (digitalRead(pins.BUTTON) == HIGH);
    
    // Debounce
    delay(DEBOUNCE_DELAY);
    
    if (sysState.dspPower) {
      debugMessage("Shutdown: Button push");
      blinkLed(BLINK_BUTTON_PUSH);
      powerOff();
    } else {
      powerOn();
    }
  }
}

// Check mute pin for signal detection
void checkMutePin() {
  if (sysConf.signalDetectMode == 0 && sysConf.standbyDelay > 0) {
    sysState.audioDetected = digitalRead(pins.MUTE);
    if ( SIGDET_MUTEPIN_INV ) {
      sysState.audioDetected = !sysState.audioDetected;
    }
    if (!sysState.audioDetected) {
      lastNoAudioDetected = millis();
    } else {
      lastNoAudioDetected = 0;
    }
  }
}

// Blink LED a specified number of times
void blinkLed(int count) {
  digitalWrite(pins.LED, false);
  delay(1000);
  
  while (count > 0) {
    digitalWrite(pins.LED, true);
    delay(300);
    digitalWrite(pins.LED, false);
    delay(300);
    count--;
  }
  
  digitalWrite(pins.LED, false);
}

// EEPROM functions
bool readEEPROM() {
  // Check if EEPROM contains valid data
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t = 0; t < sizeof(eepromSettings); t++) {
      *((char*)&eepromSettings + t) = EEPROM.read(CONFIG_START + t);
    }
    return true;
  } else {
    return false;
  }
}

void writeEEPROM() {
  #if DEBUG
    Traceln(F("Writing EEPROM"));
  #endif
  
  for (unsigned int t = 0; t < sizeof(eepromSettings); t++) {
    EEPROM.write(CONFIG_START + t, *((char*)&eepromSettings + t));
  }
}

// Debug functions
void debugMessage(const char *text) {
  #if DEBUG
    Traceln(text);
  #endif
}

void logSystemState() {
  #if DEBUG
    #if REV05
      Trace(F("Vbatt/Avg/Vbuck/Vboost/VchI/VchO: "));
    #else
      Trace(F("Vbatt/Avg/Vbuck/Vboost/Vch: "));
    #endif
    
    Trace(sysState.voltageBatt);
    Trace(F("/"));
    Trace(sysState.voltageBattAvg);
    Trace(F("/"));
    Trace(sysState.voltageBuck);
    Trace(F("/"));
    Trace(sysState.voltageBoost);
    
    #if REV05
      Trace(F("/"));
      Trace(sysState.voltageChargeIn);
    #endif
    
    Trace(F("/"));
    Trace(sysState.voltageChargeOut);
    Trace(F(" SP/HV/LV/DSP/CH/SF: "));
    Trace(sysState.dspPower);
    Trace(sysState.hvPower);
    Trace(sysState.lvPower);
    Trace(digitalRead(pins.DSP));
    Trace(sysState.charging);
    Trace(sysState.chargeFull);
    Trace(F(" aD: "));
    Trace(sysState.audioDetected);
    Trace(F(" sS: "));
    Trace(sysState.systemState);
    Trace(F(" tC: "));
    Trace(sysState.temperature);
    Trace(F(" sB: "));
    Trace(sysState.batteryState);
    Trace(F(" mC: "));
    Trace(sysConf.voltageBattMaxCharge);
    Trace(F(" vC: "));
    Traceln(sysConf.vComp);
  #endif
}
