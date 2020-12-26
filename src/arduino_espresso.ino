/* constants */
const int RELAY_LOW = LOW; // current four relay module board has inverted logic
const int RELAY_HIGH = HIGH;
const bool FINISH_ON_EMPTY = false; // optionally stop when pulling a shot and the reservoir empties. Default false as current reservoir is large enough to finish a shot on 'empty'
const int TANK_THRESH = 160; // arbitrary number for "on" for ADC based on randomly chosen resistors and measurements
const int BOILER_MAX_THRESH = 400;
const int BOILER_MIN_THRESH = 400;
const int DEBOUNCE_DELAY = 10;
const int SIGNAL_DEBOUNCE_COUNT = 40; // Debounce count for hardware ADC signals with thresholds

/* digital output pins */
const int boilerSolenoidRelayPin = 5;
const int groupSolenoidRelayPin = 6;
const int pumpRelayPin = 11;
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

unsigned long lastLoopTime = 0;

/* Serial parsing */
char receivedChars[numChars];
bool newData = false;

bool boilerIsFull = false;
int boilerIsFullDebounceCount = 0;

bool boilerIsEmpty = false;
int boilerIsEmptyDebounceCount = 0;

bool tankIsEmpty = false;
int tankIsEmptyDebounceCount = 0;

bool buttonIsPressed = false;
bool heatIsOn = false;
bool machineIsOn = false; //this is the state bit for whether the machine is active

int lastState = -1;
int state = -1;
int nextState = 4; //start 'off' until button

int brewButtonState = HIGH;
int lastBrewButtonState = HIGH;
unsigned long lastBrewButtonDebounceTime = 0;

int stopButtonState = HIGH;
int lastStopButtonState = HIGH;
unsigned long lastStopButtonDebounceTime = 0;

bool brewFlag = false;
bool stopFlag = false;

int previousFlowmeterVal = 0x0;
int flowmeterVal = 0x0;

int flowmeterCount = 0x0;

unsigned long brewButtonDownTime = 0l;
unsigned long stopButtonDownTime = 0l;

void setup()
{
  Serial.begin(9600);

  // assigning pin inputs and using internal pull up resistors on Atmega328
  pinMode(singleBrewButtonPin, INPUT);
  digitalWrite(singleBrewButtonPin, HIGH);
  pinMode(doubleBrewButtonPin, INPUT);
  digitalWrite(doubleBrewButtonPin, HIGH);
  pinMode(stopBrewButtonPin, INPUT);
  digitalWrite(stopBrewButtonPin, HIGH);

  // LEDs are active low for this machine
  pinMode(ledSinglePin, INPUT);
  digitalWrite(ledSinglePin, HIGH);
  pinMode(ledDoublePin, OUTPUT);
  digitalWrite(ledDoublePin, LOW);
  // digitalWrite(ledDoublePin, LOW);
  pinMode(ledStopPin, OUTPUT);
  digitalWrite(ledStopPin, HIGH);

  pinMode(boilerSolenoidRelayPin, OUTPUT);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  pinMode(groupSolenoidRelayPin, OUTPUT);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  pinMode(pumpRelayPin, OUTPUT);
  digitalWrite(pumpRelayPin, RELAY_LOW);
  pinMode(heatRelayPin, OUTPUT);
  digitalWrite(heatRelayPin, RELAY_LOW);

  Serial.println("starting up...");
}

void loop() {
  // these are done first because they have the ability to effect state. I think buttons need their own state in the future
  parseBrewButton();
  parseStopButton();
  parseSerial();
  showNewData();

  lastState = state;
  state = nextState;

  int boilerIsFullReading = (analogRead(analogMaxPin) < BOILER_MAX_THRESH);
  int boilerIsEmptyReading = (analogRead(analogMinPin) >= BOILER_MIN_THRESH);
  int tankIsEmptyReading = (analogRead(analogTankMinPin) >= TANK_THRESH);
  int boilerIsEmptyRaw = analogRead(analogMaxPin);
  
  debounceSignal(boilerIsFullReading, boilerIsFull, boilerIsFullDebounceCount);
  debounceSignal(boilerIsEmptyReading, boilerIsEmpty, boilerIsEmptyDebounceCount);
  debounceSignal(tankIsEmptyReading, tankIsEmpty, tankIsEmptyDebounceCount);

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
}

/**
 * Debounce a digital logic button
*/
void debounceButton(int reading, int & state, int lastState, unsigned long & lastDebounceTime) {
  if(millis() == lastLoopTime) { return; }
  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != state) {
      state = reading;
    }
  }
}

/**
 * Debounce a ADC signal logic 
 * This debounce should be less time based and more signal count based. 
 */

void debounceSignal(int reading, bool & state, int & counter) {
    // If we have gone on to the next millisecond
  if (millis() == lastLoopTime) { return; }
  
  if (reading == state && counter > 0) {
    counter--;
  } 
  
  if (reading != state) {
      counter++; 
  }
  // If the input has shown the same value for long enough let's switch it
  if (counter >= SIGNAL_DEBOUNCE_COUNT) {
    counter = 0;
    state = reading;
  }
}

// state 0
void baseState() {
  digitalWrite(pumpRelayPin, RELAY_LOW);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(true);

  // NOTE: tank min value should prevent a shot from being pulled, but shouldn't immediately kill a shot being poured

  if (!machineIsOn) {
    nextState = 4;
  } else if (!tankIsEmpty && boilerIsEmpty) {
    nextState = 2; // tank has water and boiler is empty
  } else if (!tankIsEmpty && buttonIsPressed) {
    nextState = 1;
  } else if (tankIsEmpty) {
    nextState = 3;
  } else {
    nextState = 0;
  }

}

// state 1
void pullAShotState() {
  if (lastState != state) {
    flowmeterCount = 0;
  }

  previousFlowmeterVal = flowmeterVal;
  flowmeterVal = digitalRead(flowmeterPin);

  if ((previousFlowmeterVal != flowmeterVal) && flowmeterVal) {
    flowmeterCount++;
    Serial.println(flowmeterCount);
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

  digitalWrite(pumpRelayPin, RELAY_HIGH);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_HIGH);
  toggleHeat(true);

  nextState = 1;

}

// state 2
void fillBoilerState() {
  // exit condition: tank is done filling
  if (boilerIsFull) {
    nextState = 0;
    return;
  }

  // if the tank empties, go back to base state to deal with shutting off pump
  if (tankIsEmpty) {
    nextState = 3;
    return;
  }

  digitalWrite(pumpRelayPin, RELAY_HIGH);
  digitalWrite(boilerSolenoidRelayPin, RELAY_HIGH);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false);

  nextState = 2;

}

// state 3
void tankEmptyState() {
  // exit condition: tank is no longer empty
  if (!tankIsEmpty) {
    nextState = 0;
    return;
  }

  digitalWrite(pumpRelayPin, RELAY_LOW);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(!boilerIsEmpty); // added protection for heater if tank and boiler are empty

  nextState = 3;

}

// state 4
void offState() {
  // exit condition: machine is turned on
  if (buttonIsPressed || machineIsOn) {
    machineIsOn = true;
    buttonIsPressed = false;
    nextState = 0;
    return;
  }

  digitalWrite(pumpRelayPin, RELAY_LOW);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(false); // added protection for heater if tank and boiler are empty

  nextState = 4;
}

void toggleHeat(bool heat) {
  digitalWrite(heatRelayPin, heat ? RELAY_HIGH : RELAY_LOW);
  heatIsOn = heat;
}

// high to low will toggle the "button pressed" state

void parseBrewButton() {

  int lastState = brewButtonState;
  int reading = digitalRead(singleBrewButtonPin);

  // debounce
  if (reading != lastBrewButtonState) {
    // reset the debouncing timer
    lastBrewButtonDebounceTime = millis();
    lastBrewButtonState = reading;
    Serial.print("Debounce time: ");
    Serial.println(lastBrewButtonDebounceTime);
  }

  if ((millis() - lastBrewButtonDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != brewButtonState) {
      brewButtonState = reading;
      Serial.print("Debounce satisfied at: ");
      Serial.println(millis());
    }
  }
  
  // if (lastState == brewButtonState) { return; }

  if (lastState == HIGH && brewButtonState == LOW) {
    brewButtonDownTime = millis();
  }

  else if (machineIsOn && brewButtonState == LOW && lastBrewButtonDebounceTime + 1000l < millis()) {
    Serial.print("long press at: ");
    Serial.println(millis());
    nextState = 2;
    brewFlag = true;
  }

  // this is a button toggle
  else if (lastState == LOW && brewButtonState == HIGH) {
    if (!machineIsOn) {
      machineIsOn = true;
    } else if (brewFlag) {
      brewFlag = false;
    } else {
      buttonIsPressed = true;
    }
  }
}

void parseStopButton() {

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

  // if (stopButtonState == lastState) { return; }

  if (lastState == HIGH && stopButtonState == LOW) {
    stopButtonDownTime = millis();
  } else if (stopButtonState == LOW && !buttonIsPressed && machineIsOn && (lastStopButtonDebounceTime + 1000l) < millis()) {
      machineIsOn = false;
      stopFlag = true;
  }

  // this is a button toggle
  else if (lastState == LOW && stopButtonState == HIGH) {
    if (stopFlag) {
      stopFlag = false;
      return;
    }
      buttonIsPressed = false;
      machineIsOn = true;
  }
}

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

/* Serial parsing */

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

void showNewData() {
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
