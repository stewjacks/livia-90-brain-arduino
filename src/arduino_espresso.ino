#include <EEPROM.h>

/* constants */
const int RELAY_LOW = LOW; // four relay module board used in prototype design had inverted logic. Production board does not. 
const int RELAY_HIGH = HIGH;
const bool FINISH_ON_EMPTY = false; // optionally stop when pulling a shot and the reservoir empties. Default false as current reservoir is large enough to finish a shot on 'empty'
const int TANK_THRESH = 250; // arbitrary number for "on" for ADC based on measured value and the resistor installed
const int BOILER_MAX_THRESH = 400;
const int BOILER_MIN_THRESH = 400;
const int DEBOUNCE_DELAY = 10;
const int SIGNAL_DEBOUNCE_COUNT = 40; // Debounce count for hardware ADC signals with thresholds
const int FLOWMETER_DEBOUNCE_COUNT = 5;
const int AVERAGE_LOOP_TIME = 525; // average loop time in microseconds as calculated by doing 1000 loops. NOTE: This should be periodically checked after large changes. It would be super cool if this was dynamic maybe?
const unsigned long AUTOFILL_LOOPS = 1725000l; // given average loop time, this is about a 15 minute timeout
const unsigned long MAX_SHOT_PULL_LOOPS = 115000l; //max shot pull time is set at 1 minute
const int BLINK_LED_LOOPS = 1917; // 1 second led visual cue loop time

/* Arbitrary flowmeter max and min values for auto pulling shots */
const int SHOT_FLOW_MIN = 50; //~11.25g of water
const int SHOT_FLOW_MAX = 800; //~180g of water

const int SINGLE_SHOT_FLOW_ADDR = 4;
const int DOUBLE_SHOT_FLOW_ADDR = 6;

/* digital output pins */
const int boilerSolenoidRelayPin = 5;
const int groupSolenoidRelayPin = 6;
const int heatRelayPin = 12;

/* digital input pins */
const int ledStopPin = 13;
const int ledDoublePin = 11;
const int ledSinglePin = 10;

const int flowmeterPin = 4;
const int stopBrewButtonPin = 7;
const int doubleBrewButtonPin = 8;
const int singleBrewButtonPin = 9;

/* analog input pins */
const int analogMaxPin = 2;
const int analogMinPin = 3;
const int analogTankMinPin = 4;

const byte numChars = 32;

unsigned long lastLoopTime = 0l;
unsigned long autofillLoopCount = 0l; // the base state counts the number of standby loops to check if it should autofill at fixed intervals.
unsigned long shotTimerLoopCount = 0l;

/* LED visual cue variables */
int ledBlinkLoopCount = 0;
byte ledCycleLoopCount = 0;
int currentLEDBlinkValue = LOW;

/* Serial parsing */
char receivedChars[numChars];
bool newData = false;

/* Machine sensor signals + debounce variables */
bool boilerIsFull = true;
int boilerIsFullDebounceCount = 0;
bool boilerIsEmpty = false;
int boilerIsEmptyDebounceCount = 0;
bool tankIsEmpty = false;
int tankIsEmptyDebounceCount = 0;

bool singleButtonPressed = false;
bool doubleButtonPressed = false;
bool stopButtonPressed = false;
bool heatIsOn = false;
bool machineIsOn = true; //this is the state bit for whether the machine is active
bool shouldAutofill = false; //state change flag to trigger reservoir autofill state

int lastState = -1;
int state = -1;
int nextState = 4; //start the machine in the off state, however if the machineIsOn flag is high it will turn on automatically.

int singleBrewButtonState = HIGH;
int lastSingleBrewButtonState = HIGH;
unsigned long lastSingleBrewButtonDebounceTime = 0l;

int doubleBrewButtonState = HIGH;
int lastDoubleBrewButtonState = HIGH;
unsigned long lastDoubleBrewButtonDebounceTime = 0l;

int stopButtonState = HIGH;
int lastStopButtonState = HIGH;
unsigned long lastStopButtonDebounceTime = 0l;

/* user-configured flowmeter vals */ 
int singleShotFlowmeterCount = 0;
int doubleShotFlowmeterCount = 0;

bool singleBrewButtonLongPressFlag = false;
bool doubleBrewButtonLongPressFlag = false;
bool stopButtonLongPressFlag = false;

bool previousFlowmeterVal = false;
bool flowmeterVal = false;
int flowmeterDebounceCount = 0x0;

bool shouldSaveSingleFlowmeterValue = false;
bool shouldSaveDoubleFlowmeterValue = false;

int flowmeterCount = 0x0;

unsigned long brewButtonDownTime = 0l;
unsigned long stopButtonDownTime = 0l;

void setup()
{
  Serial.begin(9600);

  Serial.println("starting up...");

  // assigning pin inputs and using internal pull up resistors on Atmega328
  pinMode(singleBrewButtonPin, INPUT);
  digitalWrite(singleBrewButtonPin, HIGH);
  pinMode(doubleBrewButtonPin, INPUT);
  digitalWrite(doubleBrewButtonPin, HIGH);
  pinMode(stopBrewButtonPin, INPUT);
  digitalWrite(stopBrewButtonPin, HIGH);

  // LEDs are active low for this machine
  pinMode(ledSinglePin, OUTPUT);
  digitalWrite(ledSinglePin, LOW);
  pinMode(ledDoublePin, OUTPUT);
  digitalWrite(ledDoublePin, LOW);
  pinMode(ledStopPin, OUTPUT);
  digitalWrite(ledStopPin, LOW);

  pinMode(boilerSolenoidRelayPin, OUTPUT);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  pinMode(groupSolenoidRelayPin, OUTPUT);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  pinMode(heatRelayPin, OUTPUT);
  digitalWrite(heatRelayPin, RELAY_LOW);

  /* EEPROM stored flowmeter values */
  singleShotFlowmeterCount = readIntFromEEPROM(SINGLE_SHOT_FLOW_ADDR);
  doubleShotFlowmeterCount = readIntFromEEPROM(DOUBLE_SHOT_FLOW_ADDR);
  
  /* EEPROM is all 0xf from the factory. Since we're using a signed int, the default flowmeter count is -1. This bounds it to the max if not set */
  singleShotFlowmeterCount = singleShotFlowmeterCount < SHOT_FLOW_MIN ? SHOT_FLOW_MAX : singleShotFlowmeterCount;
  doubleShotFlowmeterCount = doubleShotFlowmeterCount < SHOT_FLOW_MIN ? SHOT_FLOW_MAX : doubleShotFlowmeterCount;

}

void loop() {
  // these are done first because they have the ability to effect state. I think buttons need their own state in the future
  handleSingleBrewButton();
  handleDoubleBrewButton();
  handleStopButton();
  parseSerial(); //read signals from serial that can control state (i.e. some sort of serial wireless module)
  handleNewSerialData();
  
  checkAutoRefillCounter();
  checkShotTimerCounter();

  lastState = state;
  state = nextState;

  int boilerIsFullReading = (analogRead(analogMaxPin) < BOILER_MAX_THRESH);
  int boilerIsEmptyReading = (analogRead(analogMinPin) >= BOILER_MIN_THRESH);
  int tankIsEmptyReading = (analogRead(analogTankMinPin) >= TANK_THRESH);
  
  debounceSignal(boilerIsFullReading, boilerIsFull, boilerIsFullDebounceCount, SIGNAL_DEBOUNCE_COUNT);
  debounceSignal(boilerIsEmptyReading, boilerIsEmpty, boilerIsEmptyDebounceCount, SIGNAL_DEBOUNCE_COUNT);
  debounceSignal(tankIsEmptyReading, tankIsEmpty, tankIsEmptyDebounceCount, SIGNAL_DEBOUNCE_COUNT);

  if (lastState != state) {
    printStateInfo();
  }

  switch (state) {
    case 1:
      pullAShotState();
      break;
    case 2:
      fillBoilerState();
      break;
    case 3:
      tankEmptyState();
      break;
    case 4:
      offState();
      break;
    case 5:
      programState();
      break;
    case 6: 
      resetVariablesState();
      break; 
    default:
      baseState();
      break;
  }

  lastLoopTime = millis();

  // // debug calcs for loop time approx.
  // if (loopCount == 0) {
  //   Serial.print("start millis: ");
  //   Serial.println(micros());
  // }

  // loopCount++;

  // if (loopCount == 1000) {
  //   Serial.print("1000 loop micros: ");
  //   Serial.println(micros());
  // }
  
}

/**
 * Debounce a ADC signal logic 
 * This debounce should be less time based and more signal count based. 
 */

void debounceSignal(int reading, bool & state, int & counter, int debounce_count) {
    // If we have gone on to the next millisecond
  if (millis() == lastLoopTime) { return; }
  
  if (reading == state && counter > 0) {
    counter--;
  } 
  
  if (reading != state) {
      counter++; 
  }
  // If the input has shown the same value for long enough let's switch it
  if (counter >= debounce_count) {
    counter = 0;
    state = reading;
  }
}

/**
 * When in the standby base state, a counter loops to periodically check if the boiler is empty so it can be filled
 **/

void checkAutoRefillCounter() {
  if (state != 0) {
    autofillLoopCount = 0;
    return;
  }
  
  if (autofillLoopCount >= AUTOFILL_LOOPS) {
    shouldAutofill = !boilerIsFull;
    autofillLoopCount = 0;
    return;
  }

  autofillLoopCount++;
}

/**
 * Shot timer counter for a hard stop after ~1 minute of continuous flow.
 * This is a precaution to save the pump/surrounding environment from dumping the entire reservoir through the group (from experience).
 **/
void checkShotTimerCounter() {

  if (state != 1) {
    shotTimerLoopCount = 0;
    return;
  }

  if (shotTimerLoopCount >= MAX_SHOT_PULL_LOOPS) {
    stopButtonPressed = true;
    shotTimerLoopCount = 0;
  }

  shotTimerLoopCount++;

}

// state 0
void baseState() {

  if (!machineIsOn) {
    nextState = 4; // put the machine to sleep
  } else if (tankIsEmpty) {
    nextState = 3; // tank is empty: wait until filled
  } else if (boilerIsEmpty || shouldAutofill) {
    nextState = 2; // tank has water and boiler is empty
  } else if (stopButtonLongPressFlag) {
    nextState = 5; // go into program mode
  } else if (singleButtonPressed || doubleButtonPressed) {
    nextState = 1; // pull a shot
  } else {
    nextState = 0;
  }

  stopButtonPressed = false;
  
  setLEDs(HIGH, HIGH, HIGH);
  toggleHeat(true);

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);

}

// state 1
void pullAShotState() {

  bool inProgramMode = (shouldSaveDoubleFlowmeterValue || shouldSaveSingleFlowmeterValue);
  
  setLEDs(singleButtonPressed ? LOW : HIGH, doubleButtonPressed ? LOW : HIGH, HIGH);

  if (lastState != state) {
    flowmeterCount = 0;
  }

  int flowmeterMaxValue = singleButtonPressed ? singleShotFlowmeterCount : doubleShotFlowmeterCount;

  previousFlowmeterVal = flowmeterVal;
  int tempFlowmeterVal = digitalRead(flowmeterPin);

  debounceSignal(tempFlowmeterVal, flowmeterVal, flowmeterDebounceCount, FLOWMETER_DEBOUNCE_COUNT);

  if (previousFlowmeterVal != flowmeterVal) {
    flowmeterCount++;
    Serial.print(flowmeterCount);
    Serial.print(",");
    Serial.println(millis());
  }

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_HIGH);
  toggleHeat(true);

  nextState = 1;

  if (stopButtonPressed) {
    handlePersistFlowmeterValue(flowmeterCount);
  }

  // Manual stop due to button
  if (stopButtonPressed) {
    nextState = 6;
  }

  // automatic stop in regular mode
  if (!inProgramMode && (flowmeterCount >= flowmeterMaxValue)) {
    nextState = 6;
  }

  // optionally kill the shot here if the tank is empty.
 if (tankIsEmpty && FINISH_ON_EMPTY) {
   nextState = 3;
 }

}

// state 2
void fillBoilerState() {

  cycleLEDs();

  // exit condition: tank is done filling
  shouldAutofill = false;

  nextState = 2;

  if (boilerIsFull) {
    nextState = 6;
    return;
  }

  // if the tank runs out, stop the pump
  if (tankIsEmpty) {
    nextState = 3;
    return;
  }

  digitalWrite(boilerSolenoidRelayPin, RELAY_HIGH);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false);

}

// state 3
void tankEmptyState() {

  blinkLEDs();

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(!boilerIsEmpty); // added protection for heater if tank and boiler are empty

  nextState = 3;

  // exit condition: tank is no longer empty
  if (!tankIsEmpty) {
    nextState = 6;
  }
}

// state 4
void offState() {
  setLEDs(HIGH, HIGH, HIGH);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false); // added protection for heater if tank and boiler are empty

  nextState = 4;

  // exit condition: machine is turned on
  if (singleButtonPressed || doubleButtonPressed || machineIsOn) {
    machineIsOn = true;
    autofillLoopCount = (int) AUTOFILL_LOOPS * 0.9; // want to seed the autofill loop so it triggers shortly after on, but not immediately.
    nextState = 6;
  }
}

// state 5
void programState() {

  setLEDs(LOW, LOW, HIGH);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(true);

  if (singleButtonPressed) {
    singleButtonPressed = true;
    doubleButtonPressed = false;
    stopButtonPressed = false;
    shouldSaveSingleFlowmeterValue = true;
    nextState = 1;
  } else if (doubleButtonPressed) {
    doubleButtonPressed = true;
    stopButtonPressed = false;
    shouldSaveDoubleFlowmeterValue = true;
    nextState = 1;
  } else if (stopButtonPressed) {
    stopButtonPressed = false;
    nextState = 6;
    // this is the cancel condition
  } else {
    nextState = 5;
  }
}

// state 6: reset all variables before heading into base state. Used to negate user input between states (auto fill, empty reservoir, etc)
void resetVariablesState() {
  singleButtonPressed = false;
  doubleButtonPressed = false;
  stopButtonPressed = false;
  singleBrewButtonLongPressFlag = false;
  doubleBrewButtonLongPressFlag = false;
  stopButtonLongPressFlag = false;
  nextState = 0;
}

void toggleHeat(bool heat) {
  digitalWrite(heatRelayPin, heat ? RELAY_HIGH : RELAY_LOW);
  heatIsOn = heat;
}

void setLEDs(int singleLED, int doubleLED, int stopLED) {
  digitalWrite(ledSinglePin, singleLED);
  digitalWrite(ledDoublePin, doubleLED);
  digitalWrite(ledStopPin, stopLED);
}

/**
 * Blinking LEDs -> visual indicator for an empty reservoir
 **/

void blinkLEDs() {

  ledBlinkLoopCount++;

  if (ledBlinkLoopCount >= BLINK_LED_LOOPS) {
    ledBlinkLoopCount = 0;
    currentLEDBlinkValue = !currentLEDBlinkValue;
  }

  setLEDs(currentLEDBlinkValue, currentLEDBlinkValue, currentLEDBlinkValue);

}

/**
 * Cycle LEDs -> visual indicator for boiler filling
 **/
void cycleLEDs() {
  ledBlinkLoopCount++;
  if (ledBlinkLoopCount >= BLINK_LED_LOOPS) {
    ledBlinkLoopCount = 0;
    ledCycleLoopCount = (ledCycleLoopCount + 1) % 3;
  }

  if (ledCycleLoopCount == 0) {
    setLEDs(HIGH, HIGH, LOW);
  } else if (ledCycleLoopCount == 1) {
    setLEDs(HIGH, LOW, LOW);
  } else {
    setLEDs(LOW, LOW, LOW);
  }
}

void handlePersistFlowmeterValue(int & value) {
  int constrainedValue = min(max(value, SHOT_FLOW_MIN), SHOT_FLOW_MAX);
  
  if (shouldSaveSingleFlowmeterValue) {
    writeIntIntoEEPROM(SINGLE_SHOT_FLOW_ADDR, constrainedValue);
    singleShotFlowmeterCount = constrainedValue;
  } else if (shouldSaveDoubleFlowmeterValue) {
    writeIntIntoEEPROM(DOUBLE_SHOT_FLOW_ADDR, constrainedValue);
    doubleShotFlowmeterCount = constrainedValue;
  }

  shouldSaveSingleFlowmeterValue = false;
  shouldSaveDoubleFlowmeterValue = false;
}

/**
 * Single shot brew button stateful logic, including press debounce and long press. 
 * TODO: Single & double are copy-pasta & can be unified
 **/

void handleSingleBrewButton() {

  int lastState = singleBrewButtonState;
  int reading = digitalRead(singleBrewButtonPin);

  // debounce
  if (reading != lastSingleBrewButtonState) {
    // reset the debouncing timer
    lastSingleBrewButtonDebounceTime = millis();
    lastSingleBrewButtonState = reading;
  }

  if ((millis() - lastSingleBrewButtonDebounceTime) > DEBOUNCE_DELAY) {
     singleBrewButtonState = reading;
  }
  
  if (lastState == HIGH && singleBrewButtonState == LOW) {
    brewButtonDownTime = millis();
  }

  else if (machineIsOn && singleBrewButtonState == LOW && lastSingleBrewButtonDebounceTime + 1000l < millis()) {
    singleBrewButtonLongPressFlag = true;
    shouldAutofill = true;
  }

  // this is a button toggle
  else if (lastState == LOW && singleBrewButtonState == HIGH) {
    if (!machineIsOn) {
      machineIsOn = true;
    } else if (singleBrewButtonLongPressFlag) {
      singleBrewButtonLongPressFlag = false;
    } else {
      singleButtonPressed = true;
      doubleButtonPressed = false;
    }
  }
}

/**
 * Double shot brew button stateful logic, including press debounce and long press. 
 * TODO: combine with single
 **/

void handleDoubleBrewButton() {

  int lastState = doubleBrewButtonState;
  int reading = digitalRead(doubleBrewButtonPin);

  // debounce
  if (reading != lastDoubleBrewButtonState) {
    // reset the debouncing timer
    lastDoubleBrewButtonDebounceTime = millis();
    lastDoubleBrewButtonState = reading;
  }

  if ((millis() - lastDoubleBrewButtonDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != doubleBrewButtonState) {
      doubleBrewButtonState = reading;
    }
  }
  
  if (lastState == HIGH && doubleBrewButtonState == LOW) {
    brewButtonDownTime = millis();
  }

  else if (machineIsOn && doubleBrewButtonState == LOW && lastDoubleBrewButtonDebounceTime + 1000l < millis()) {
    doubleBrewButtonLongPressFlag = true;
    machineIsOn = false;
  }

  // this is a button toggle
  else if (lastState == LOW && doubleBrewButtonState == HIGH) {
    if (!machineIsOn) {
      machineIsOn = true;
    } else if (doubleBrewButtonLongPressFlag) {
      doubleBrewButtonLongPressFlag = false;
    } else {
      singleButtonPressed = false;
      doubleButtonPressed = true;
    }
  }
}

/**
 * Stop button stateful logic, including press debounce and long press. 
 **/

void handleStopButton() {

  int lastState = stopButtonState;
  int reading = digitalRead(stopBrewButtonPin);

  // debounce
  if (reading != lastStopButtonState) {
    // reset the debouncing timer
    lastStopButtonDebounceTime = millis();
    lastStopButtonState = reading;
  }

  if ((millis() - lastStopButtonDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != stopButtonState) {
      stopButtonState = reading;
    }
  }

  if (lastState == HIGH && stopButtonState == LOW) {
    stopButtonDownTime = millis();
  } else if (stopButtonState == LOW && !singleButtonPressed && !doubleButtonPressed && machineIsOn && (lastStopButtonDebounceTime + 1000l) < millis()) {
      stopButtonLongPressFlag = true;
  }

  // this is a button toggle
  else if (lastState == LOW && stopButtonState == HIGH) {
    if (stopButtonLongPressFlag) {
      stopButtonLongPressFlag = false;
    } else {
      singleButtonPressed = false;
      doubleButtonPressed = false;
      machineIsOn = true;
      stopButtonPressed = true;
    }
  }
}
/**
 * EEPROM read/write int (2 bytes)
 **/

void writeIntIntoEEPROM(int addr, int val) { 
  byte b1 = val >> 8;
  byte b2 = val & 0xFF;
  EEPROM.write(addr, b1);
  EEPROM.write(addr + 1, b2);
}

int readIntFromEEPROM(int addr) {
  byte b1 = EEPROM.read(addr);
  byte b2 = EEPROM.read(addr + 1);
  return (b1 << 8) + b2;
}

/**
 * Debug serial print statement. This is expensive so is only triggered on state change. 
 **/
void printStateInfo() {
  Serial.println("*************************");
  Serial.print("last state: ");
  Serial.print(lastState);
  Serial.print(" state: ");
  Serial.print(state);
  Serial.print(" next state: ");
  Serial.println(nextState);
  Serial.print("boiler is empty: ");
  Serial.print(analogRead(analogMinPin));
  Serial.println(boilerIsEmpty ? " true" : " false");
  Serial.print("boiler is full: ");
  Serial.print(analogRead(analogMaxPin));
  Serial.println(boilerIsFull ? " true" : " false");
  Serial.print("tank is empty: ");
  Serial.print(analogRead(analogTankMinPin));
  Serial.println(tankIsEmpty ? " true" : " false");
  Serial.print("single is pressed: ");
  Serial.println(singleButtonPressed ? "true" : "false");
  Serial.print("double is pressed: ");
  Serial.println(doubleButtonPressed ? "true" : "false");
  Serial.print("heat is on: ");
  Serial.println(heatIsOn ? "true" : "false");
  Serial.print("machine is on: ");
  Serial.println(machineIsOn ? "true" : "false");
  Serial.print("flowmeter single count: ");
  Serial.println(singleShotFlowmeterCount);
  Serial.print("flowmeter double count: ");
  Serial.println(doubleShotFlowmeterCount);
  Serial.println("*************************");
}

/**
 * Serial input parsing. Check the serial line for understood signals.
 **/

void parseSerial() {
    static bool recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    // strings max at 32 bits

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

/**
 * Parse and decode any signals received on the serial line.
 **/

void handleNewSerialData() {
    if (newData == true) {
      newData = false;
      String a = String(receivedChars);

      if (a == "on") {
        machineIsOn = true;
      } else if (a == "off") {
        machineIsOn = false;
      } else if (a == "f") {
        if (machineIsOn) {
          nextState = 2;
        } else {
          Serial.println("<ERROR>");
          return;
        }
      } else if (a == "s") {
        Serial.println(state);
        return;
      } else if (a == "v") {
        printStateInfo();
        return;
      } else {
        Serial.println("<UNKNOWN>");
         return;
      }
      Serial.println("<OK>");
    }
}
