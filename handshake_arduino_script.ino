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

// Tower Solenoid Pins
const int LClose = 13;
const int RClose = 12;
const int LOpen = 11;
const int ROpen = 10;

const int piCommsPin_close = 7; // Using digital pin D7 to listen for pi telling to close
const int piCommsPin_start = 8; // Using digital pin D8 to listen for pi tellingn to start

/*  
 * `systemState` keeps track of where in the cycle we are
 *    0 - door closed, not live
 *    1 - door closed, live (i.e. sensing, possible open)
 *    2 - door open, not live
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
  pinMode(piCommsPin_close, INPUT_PULLUP);
  pinMode(piCommsPin_start, INPUT_PULLUP);
  
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
  //digitalWrite(outputPin, HIGH); 

  delay(1000);
  digitalWrite(LOpen, LOW);
  digitalWrite(ROpen, LOW);
  //digitalWrite(outputPin, LOW); 
  systemState = 2;
}

void closeDoor(){
  // Replace this with outputs to the solenoids
  digitalWrite(LClose, HIGH);
  digitalWrite(RClose, HIGH);
  //digitalWrite(outputPin, HIGH); 

  delay(1000);
  digitalWrite(LClose, LOW);
  digitalWrite(RClose, LOW);  
  //digitalWrite(outputPin, LOW); 

  // TODO: CHANGE THIS TO 0 ONCE ARMING SIGNAL IMPLEMENTED
  systemState = 1;
}

void loop() {  
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    lastByte = incomingByte;
    
    // Process the value
    if (incomingByte == '1' && systemState == 1) {
      // Solenoid trigger signal received
      openDoor();
    } 
    else if (incomingByte == '0') {
      // Do nothing for now
    }
    // Ignore any other characters
  }

  // Close door
  
  if (digitalRead(resetPin) == LOW){
    /*  I don't think we should check that we're in the open state
     *  to close the door; it's possible that the door is really open
     *  arduino thinks its closed. This will allow it to open anyways
     */
    closeDoor(); 
  }
  
  
  // Small delay to avoid overwhelming the serial buffer
  delay(50);
}