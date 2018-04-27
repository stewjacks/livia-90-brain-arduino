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

boolean boilerIsFull = false;
boolean boilerIsEmpty = false;
boolean tankIsEmpty = false;
boolean buttonIsPressed = false;
boolean heatIsOn = false;

int lastState = -1;
int state = -1;
int nextState = 0;

int lastBrewButtonState = HIGH;
int brewButtonState = HIGH;
int lastStopButtonState = HIGH;
int stopButtonState = HIGH;

int previousFlowmeterVal = 0x0;
int flowmeterVal = 0x0;

int flowmeterCount = 0x0;

unsigned long brewButtonDownTime = 0l;

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

  pinMode(boilerSolenoidRelayPin, OUTPUT);
  pinMode(groupSolenoidRelayPin, OUTPUT);
  pinMode(pumpRelayPin, OUTPUT);
  pinMode(heatRelayPin, OUTPUT);

  digitalWrite(pumpRelayPin, RELAY_LOW);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  digitalWrite(heatRelayPin, RELAY_LOW);

  Serial.println("starting up...");
}

void loop() {
  // these are done first because they have the ability to effect state. I think buttons need their own state in the future
  parseBrewButton();
  parseStopButton();

  lastState = state;
  state = nextState;

  boilerIsFull = (analogRead(analogMaxPin) < BOILER_MAX_THRESH);
  boilerIsEmpty = (analogRead(analogMinPin) >= BOILER_MIN_THRESH);
  tankIsEmpty = (analogRead(analogTankMinPin) >= TANK_THRESH);

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
    default:
      baseState();
      break;
  }

  if (lastState == state) return;

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
  Serial.println("*************************");
}

// state 0
void baseState() {
  digitalWrite(pumpRelayPin, RELAY_LOW);
  digitalWrite(boilerSolenoidRelayPin, RELAY_LOW);
  digitalWrite(groupSolenoidRelayPin, RELAY_LOW);
  toggleHeat(true);

  // NOTE: tank min value should prevent a shot from being pulled, but shouldn't immediately kill a shot being poured

  // tank has water and boiler is empty
  if (!tankIsEmpty && boilerIsEmpty) {
    nextState = 2;
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
  // tank is done filling
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
//  Serial.println("state: tank is empty");
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

  // this is a button toggle
  else if (lastBrewButtonState == LOW && brewButtonState == HIGH) {
    buttonIsPressed = true;
  }

  // way to force a fill tank by pushing two buttons for 3 seconds
  else if (brewButtonState == LOW && stopButtonState == LOW && brewButtonDownTime + 3000l > millis()) {
    nextState = 2;
  }
}

void parseStopButton() {
  lastStopButtonState = stopButtonState;
  stopButtonState = digitalRead(stopBrewButtonPin);

  // this is a button toggle
  if (lastStopButtonState == LOW && stopButtonState == HIGH) {
    buttonIsPressed = false;
  }
}
