/* constants */
const int RELAY_LOW = LOW; // current four relay module board has inverted logic
const int RELAY_HIGH = HIGH;
const boolean FINISH_ON_EMPTY = false; // optionally stop when pulling a shot and the reservoir empties. Default false as current reservoir is large enough to finish a shot on 'empty'
const int TANK_THRESH = 400; // arbitrary number for "on" for ADC based on randomly chosen resistors and measurements
const int BOILER_MAX_THRESH = 400;
const int BOILER_MIN_THRESH = 400;

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

/* Serial parsing */
char receivedChars[numChars];
boolean newData = false;

boolean boilerIsFull = false;
boolean boilerIsEmpty = false;
boolean tankIsEmpty = false;
boolean buttonIsPressed = false;
boolean heatIsOn = false;
boolean machineIsOn = false; //this is the state bit for whether the machine is active

int lastState = -1;
int state = -1;
int nextState = 4; //start 'off' until button

int lastBrewButtonState = HIGH;
int brewButtonState = HIGH;
int lastStopButtonState = HIGH;
int stopButtonState = HIGH;
boolean brewFlag = false;
boolean stopFlag = false;

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

  boilerIsFull = (analogRead(analogMaxPin) < BOILER_MAX_THRESH);
  boilerIsEmpty = (analogRead(analogMinPin) >= BOILER_MIN_THRESH);
  tankIsEmpty = (analogRead(analogTankMinPin) >= TANK_THRESH);

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

void toggleHeat(boolean heat) {
  digitalWrite(heatRelayPin, heat ? RELAY_HIGH : RELAY_LOW);
  heatIsOn = heat;
}

// high to low will toggle the "button pressed" state

void parseBrewButton() {
  lastBrewButtonState = brewButtonState;
  brewButtonState = digitalRead(singleBrewButtonPin);

  if (lastBrewButtonState == HIGH && brewButtonState == LOW) {
    brewButtonDownTime = millis();
  }

  else if (machineIsOn && brewButtonState == LOW && brewButtonDownTime + 1000l < millis()) {
    nextState = 2;
    brewFlag = true;
  }

  // this is a button toggle
  else if (lastBrewButtonState == LOW && brewButtonState == HIGH) {
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
  lastStopButtonState = stopButtonState;
  stopButtonState = digitalRead(stopBrewButtonPin);

  if (lastStopButtonState == HIGH && stopButtonState == LOW) {
    stopButtonDownTime = millis();
  } else if (stopButtonState == LOW && !buttonIsPressed && machineIsOn && (stopButtonDownTime + 1000l) < millis()) {
      machineIsOn = false;
      stopFlag = true;
  }

  // this is a button toggle
  else if (lastStopButtonState == LOW && stopButtonState == HIGH) {
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
    static boolean recvInProgress = false;
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
