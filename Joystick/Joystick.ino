#define NUM_BUTTONS 6
#define NUM_AXES 8

int buttonPins[NUM_BUTTONS] = {1,2,3,4,5,6};
int axisPins[NUM_AXES] = {A0,A1,A2,A3,A4,A5,A6,A8};

void setup() {
  // This makes it so that the states are sent manually
  Joystick.useManualSend(true);
  
  // Declare button pins as input with the internal pullup resistor on
  for (int i = 0; i < NUM_BUTTONS; i++) 
    pinMode(buttonPins[i], INPUT_PULLUP);
}

void loop() {
  // Read analog pots
  for (int i = 0; i < NUM_AXES; i++) 
    Joystick.axis(i, analogRead(axisPins[i])); 
  
  // Read buttons
  for (int i = 0; i < NUM_BUTTONS; i++) 
    Joystick.button(i, digitalRead(buttonPins[i]) != HIGH);
  
  Joystick.send_now();  // Send control states
  delay(10);  // Update joystick values every 10 ms
}
