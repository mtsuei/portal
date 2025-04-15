/* This code mediates the threeway handshake between the
 * pc, arduino, and raspberry pi
 * 
 * A couple of important notes:
 * 1) the arduino can never exactly know the condition of the door
 * on startup. IT ASSUMES THE DOOR IS OPEN
 * 2) The arduino should never close the door if not prompted by the PI
 * AND if it's not in the right state to do so
 * 
 * TODO:
 * - Implement I2C -> Pi
 * - Implement state switching
 */

// Arduino code to read a binary value from serial and set pin D2 accordingly
const int outputPin = 5;        // Using digital pin D5; temp light output
const int resetPin = 6;         // Using digital pin D6 to reset the latch
const int ledPin = A0;

// Tower Solenoid Pins
const int LClose = 13;
const int RClose = 12;
const int LOpen = 11;
const int ROpen = 10;

// Pi comms pins (i.e. get info from comms)
const int piCommsPin_open = 7;
const int piCommsPin_close = 8;
const int piCommsPin_arm = 9;

/*  
 * `systemState` keeps track of where in the cycle we are
 *    0 - door closed, not armed
 *    1 - door closed, armed (i.e. sensing, possible open)
 *    2 - door open, not armed
 */
int systemState;

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Initialize pinModes
  pinMode(outputPin, OUTPUT);
  pinMode(LOpen, OUTPUT);
  pinMode(LClose, OUTPUT);
  pinMode(ROpen, OUTPUT);
  pinMode(RClose, OUTPUT);
  
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(piCommsPin_open, INPUT);
  pinMode(piCommsPin_close, INPUT);
  pinMode(piCommsPin_arm, INPUT);

  pinMode(ledPin, OUTPUT);
  
  // Initialize pins
  digitalWrite(outputPin, LOW);
  digitalWrite(LOpen, LOW);
  digitalWrite(LClose, LOW);
  digitalWrite(ROpen, LOW);
  digitalWrite(RClose, LOW);
  
  // Initialize mode
  // Arduino assumes door is open on first connection
  systemState = 2;
  
  // Wait for serial port to connect
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
}


char lastByte;              // Holds most recent serial state

void openDoor(){
  // Replace this with outputs to the solenoids
  digitalWrite(LOpen, HIGH);
  digitalWrite(ROpen, HIGH);
  delay(1000);
  digitalWrite(LOpen, LOW);
  digitalWrite(ROpen, LOW);
}

void closeDoor(){
  // Replace this with outputs to the solenoids
  digitalWrite(LClose, HIGH);
  digitalWrite(RClose, HIGH);
  delay(1000);
  digitalWrite(LClose, LOW);
  digitalWrite(RClose, LOW);  
}

void loop() {  
  char incomingByte;
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    incomingByte = Serial.read();
    // Right now should be '0' or '1'
    //lastByte = incomingByte;
  }

  // Act based on system state
  if (systemState == 0){ // Door closed, not armed --------------------
    analogWrite(ledPin, 0);
  } else if (systemState == 1){ // Door closed, armed -----------------
    // Door closed, armed
    analogWrite(ledPin, 255);
    if (incomingByte == '1') { 
      // Solenoid trigger signal received
      openDoor();
      systemState = 2;
    } 
  } else if (systemState == 2){ // Door open --------------------------
    // Do nothing for now 
    analogWrite(ledPin, 0);
  }
  
  if (digitalRead(piCommsPin_close) == HIGH or digitalRead(resetPin) == LOW){
    /*  I don't think we should check that we're in the open state
     *  to close the door; it's possible that the door is really open
     *  arduino thinks its closed. This will allow it to open anyways
     */
    closeDoor(); 
    systemState = 0;
  }

  if (digitalRead(piCommsPin_open) == HIGH ){
    openDoor();
    systemState = 2;
  }

  if (digitalRead(piCommsPin_arm) == HIGH){
      systemState = 1;  
  }
  
  // Small delay to avoid overwhelming the serial buffer
  delay(50);
}