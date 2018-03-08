int analogPin= 0;
int raw= 0;

boolean boilerIsFull = false;
boolean boilerIsEmpty = false;
boolean tankIsEmpty = false;
boolean buttonIsPressed = false;
boolean heatIsOn = false;

int minPin = 3;
int maxPin = 2;
int tankMinPin = 7;

int boilerSolenoidRelayPin = 5;
int groupSolenoidRelayPin = 6;
int pumpRelayPin = 11;
int heatRelayPin = 12;

int buttonPin = 8;

int analogMinPin = 3;
int analogMaxPin = 2;
int analogTankMinPin = 4;

int state = 0;
int nextState = 0;

int ANALOG_THRESH = 800; // arbitrary number for "on" for ADC

boolean initializing = true;

void setup()
{
  Serial.begin(9600);
  
  pinMode(maxPin, INPUT);
  pinMode(minPin, INPUT);
  pinMode(tankMinPin, INPUT);
  pinMode(buttonPin, INPUT);
  
  pinMode(boilerSolenoidRelayPin, OUTPUT);
  pinMode(groupSolenoidRelayPin, OUTPUT);
  pinMode(pumpRelayPin, OUTPUT);
  pinMode(heatRelayPin, OUTPUT);
  
  Serial.print("starting up. Initial state: ");
  Serial.println(state);
}

void loop() {
  delay(100);  
  state = nextState;
  
//  boilerIsFull = !digitalRead(maxPin);
//  boilerIsEmpty = digitalRead(minPin);
//  tankIsEmpty = digitalRead(tankMinPin);
 
  buttonIsPressed = digitalRead(buttonPin);

  
  boilerIsFull = (analogRead(analogMaxPin) < ANALOG_THRESH); 
  boilerIsEmpty = (analogRead(analogMinPin) >= ANALOG_THRESH);
  tankIsEmpty = (analogRead(analogTankMinPin) >= ANALOG_THRESH);

//  raw= analogRead(analogPin);

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

  if (state == nextState && !initializing) return;

  initializing = false;
 
  Serial.println("*************************");
//  Serial.println(raw);
  Serial.print("state: ");
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

void baseState() {
//  Serial.println("state: base");
 
  digitalWrite(pumpRelayPin, LOW);
  digitalWrite(boilerSolenoidRelayPin, LOW);
  digitalWrite(groupSolenoidRelayPin, LOW);
  toggleHeat(true);
  
  // tank min value should prevent a shot from being pulled, but shouldn't immediately kill a shot being poured

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
//  Serial.println("state: pull a shot");

  // shot pulling is done. 
  if (!buttonIsPressed) {
    nextState = 0;
    return;
  }

  // optionally kill the shot here if the tank is empty.
//  if (tankIsEmpty) {
//    nextState = 3;
//    return;
//  }
 
  digitalWrite(pumpRelayPin, HIGH);
  digitalWrite(boilerSolenoidRelayPin, LOW);
  digitalWrite(groupSolenoidRelayPin, HIGH);
  toggleHeat(true);
  
  nextState = 1;
 
}

// state 2
void fillBoilerState() {
//  Serial.println("state: filling boiler");
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
  
  digitalWrite(pumpRelayPin, HIGH);
  digitalWrite(boilerSolenoidRelayPin, HIGH);
  digitalWrite(groupSolenoidRelayPin, LOW);
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

  digitalWrite(pumpRelayPin, LOW);
  digitalWrite(boilerSolenoidRelayPin, LOW);
  digitalWrite(groupSolenoidRelayPin, LOW);

  // added protection here
  if (boilerIsEmpty) {
    toggleHeat(false);
  }
  
  nextState = 3; 
  
}

void toggleHeat(boolean heat) {
  digitalWrite(heatRelayPin, heat ? HIGH : LOW);
  heatIsOn = heat;
}

