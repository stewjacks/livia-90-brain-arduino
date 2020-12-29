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
const int FLOWMETER_DEBOUNCE_COUNT = 10;
const int AVERAGE_LOOP_TIME = 525; // average loop time in microseconds as calculated by doing 1000 loops. NOTE: This should be periodically checked after large changes
const unsigned long AUTOFILL_LOOPS = 1725000l; // given average loop time, this is about a 15 minute timeout
const unsigned long MAX_SHOT_PULL_LOOPS = 115000l; //max shot pull time is set at 1 minute
const int BLINK_LED_LOOPS = 1917; // 1 second led visual cue loop time

/* Arbitrary flowmeter max and min values for auto pulling shots */
const int SHOT_FLOW_MIN = 50; //~11.25g of water
const int SHOT_FLOW_MAX = 600; //~135g of water

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

bool buttonIsPressed = false;
bool heatIsOn = false;
bool machineIsOn = true; //this is the state bit for whether the machine is active
bool shouldAutofill = false; //state change flag to trigger reservoir autofill state

int lastState = -1;
int state = -1;
int nextState = 4; //start the machine in the off state, however if the machineIsOn flag is high it will turn on automatically.

int brewButtonState = HIGH;
int lastBrewButtonState = HIGH;
unsigned long lastBrewButtonDebounceTime = 0l;

int stopButtonState = HIGH;
int lastStopButtonState = HIGH;
unsigned long lastStopButtonDebounceTime = 0l;

/* user-configured flowmeter vals */ 
bool userHasSetSingleShotCount = false;
bool userHasSetDoubleShotCount = false;
int singleShotFlowmeterCount = 0;
int doubleShotFlowmeterCount = 0;

bool brewButtonLongPressFlag = false;
bool stopButtonLongPressFlag = false;

bool previousFlowmeterVal = false;
bool flowmeterVal = false;
int flowmeterDebounceCount = 0x0;

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
  
  userHasSetSingleShotCount = SHOT_FLOW_MIN <= singleShotFlowmeterCount && singleShotFlowmeterCount <= SHOT_FLOW_MAX;
  userHasSetDoubleShotCount = SHOT_FLOW_MIN <= doubleShotFlowmeterCount && doubleShotFlowmeterCount <= SHOT_FLOW_MAX;

  Serial.println("EEPROM:");
  Serial.print("single val ");
  Serial.println(singleShotFlowmeterCount);
  Serial.print("double val ");
  Serial.println(doubleShotFlowmeterCount);

}

void loop() {
  // these are done first because they have the ability to effect state. I think buttons need their own state in the future
  handleBrewButton();
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
  //   Serial.print("1000 loop millis: ");
  //   Serial.println(micros());
  // }
  
}

/**
 * Debounce a ADC signal logic 
 * This debounce should be less time based and more signal count based. 
 */

void debounceSignal(int reading, bool & state, int & counter, int & debounce_count) {
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
    shouldAutofill = true;
    autofillLoopCount = 0;
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
    buttonIsPressed = false;
    shotTimerLoopCount = 0;
  }

  shotTimerLoopCount++;

}

// state 0
void baseState() {

  if (!machineIsOn) {
    nextState = 4;
  } else if ((!tankIsEmpty && boilerIsEmpty) || shouldAutofill) {
    nextState = 2; // tank has water and boiler is empty
  } else if (!tankIsEmpty && buttonIsPressed) {
    nextState = 1;
  } else if (tankIsEmpty) {
    nextState = 3;
  } else {
    nextState = 0;
  }
  
  setLEDs(HIGH, HIGH, HIGH);
  toggleHeat(true);

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);

}

// state 1
void pullAShotState() {
  
  setLEDs(LOW, HIGH, HIGH);

  if (lastState != state) {
    flowmeterCount = 0;
  }

  previousFlowmeterVal = flowmeterVal;
  int tempFlowmeterVal = digitalRead(flowmeterPin);

  debounceSignal(tempFlowmeterVal, flowmeterVal, flowmeterDebounceCount, FLOWMETER_DEBOUNCE_COUNT);

  if (previousFlowmeterVal != flowmeterVal) {
    flowmeterCount++;
    Serial.print(flowmeterCount);
    Serial.print(",");
    Serial.println(millis());
  }

  // TODO: this should be the user-provided value OR shot_flow_max
  if (flowmeterCount >= SHOT_FLOW_MAX) {
    buttonIsPressed = false;
    nextState = 0;
    return;
  }

  // shot pulling is done.
  if (!buttonIsPressed) {
    nextState = 0;
    return;
  }

  // optionally kill the shot here if the tank is empty.
 if (tankIsEmpty && FINISH_ON_EMPTY) {
   nextState = 3;
   return;
 }

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_HIGH);
  toggleHeat(true);

  nextState = 1;

}

// state 2
void fillBoilerState() {

  cycleLEDs();

  // exit condition: tank is done filling
  shouldAutofill = false;

  if (boilerIsFull) {
    buttonIsPressed = false; // want to return to the base state without resuming anything
    nextState = 0;
    return;
  }

  // if the tank empties, go back to base state to deal with shutting off pump
  if (tankIsEmpty) {
    nextState = 3;
    return;
  }

  digitalWrite(boilerSolenoidRelayPin, RELAY_HIGH);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false);

  nextState = 2;

}

// state 3
void tankEmptyState() {

  blinkLEDs();

  // exit condition: tank is no longer empty
  if (!tankIsEmpty) {
    buttonIsPressed = false; // want to return to the base state without resuming anything
    nextState = 0;
    return;
  }

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(!boilerIsEmpty); // added protection for heater if tank and boiler are empty

  nextState = 3;

}

// state 4
void offState() {
  setLEDs(HIGH, HIGH, HIGH);
  
  // exit condition: machine is turned on
  if (buttonIsPressed || machineIsOn) {
    machineIsOn = true;
    buttonIsPressed = false;
    nextState = 0;
    shouldAutofill = true;
    return;
  }

  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false); // added protection for heater if tank and boiler are empty

  nextState = 4;
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

/**
 * Brew button stateful logic, including press debounce and long press. 
 **/

void handleBrewButton() {

  int lastState = brewButtonState;
  int reading = digitalRead(singleBrewButtonPin);

  // debounce
  if (reading != lastBrewButtonState) {
    // reset the debouncing timer
    lastBrewButtonDebounceTime = millis();
    lastBrewButtonState = reading;
  }

  if ((millis() - lastBrewButtonDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != brewButtonState) {
      brewButtonState = reading;
    }
  }
  
  if (lastState == HIGH && brewButtonState == LOW) {
    brewButtonDownTime = millis();
  }

  else if (machineIsOn && brewButtonState == LOW && lastBrewButtonDebounceTime + 1000l < millis()) {
    brewButtonLongPressFlag = true;
    shouldAutofill = true;
  }

  // this is a button toggle
  else if (lastState == LOW && brewButtonState == HIGH) {
    if (!machineIsOn) {
      machineIsOn = true;
    } else if (brewButtonLongPressFlag) {
      brewButtonLongPressFlag = false;
    } else {
      buttonIsPressed = true;
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
  } else if (stopButtonState == LOW && !buttonIsPressed && machineIsOn && (lastStopButtonDebounceTime + 1000l) < millis()) {
      machineIsOn = false;
      stopButtonLongPressFlag = true;
  }

  // this is a button toggle
  else if (lastState == LOW && stopButtonState == HIGH) {
    if (stopButtonLongPressFlag) {
      stopButtonLongPressFlag = false;
    } else {
      buttonIsPressed = false;
      machineIsOn = true;
    }
  }
}

void writeIntIntoEEPROM(int addr, int number)
{ 
  byte b1 = number >> 8;
  byte b2 = number & 0xFF;
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
  Serial.print("button is pressed: ");
  Serial.println(buttonIsPressed ? "true" : "false");
  Serial.print("heat is on: ");
  Serial.println(heatIsOn ? "true" : "false");
  Serial.print("machine is on: ");
  Serial.println(machineIsOn ? "true" : "false");
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
