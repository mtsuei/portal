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
 * - 
 */

// Arduino code to read a binary value from serial and set pin D2 accordingly
const int outputPin = 5;        // Using digital pin D5; temp light output
const int resetPin = 6;         // Using digital pin D6 to reset the latch

// Tower Solenoid Pins
const int leftTowerOpen = 13;
const int leftTowerClose = 12;
const int rightTowerOpen = 11;
const int rightTowerClose = 10;

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
  pinMode(leftTowerOpen, OUTPUT);
  pinMode(leftTowerClose, OUTPUT);
  pinMode(rightTowerOpen, OUTPUT);
  pinMode(rightTowerClose, OUTPUT);
  
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(piCommsPin_close, INPUT_PULLUP);
  pinMode(piCommsPin_start, INPUT_PULLUP);
  
  // Initialize pins
  digitalWrite(outputPin, LOW);
  digitalWrite(leftTowerOpen, LOW);
  digitalWrite(leftTowerClose, LOW);
  digitalWrite(rightTowerOpen, LOW);
  digitalWrite(rightTowerClose, LOW);
  
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
  digitalWrite(leftTowerOpen, HIGH);
  digitalWrite(rightTowerOpen, HIGH);
  digitalWrite(outputPin, HIGH); 

  delay(1000);
  digitalWrite(leftTowerOpen, LOW);
  digitalWrite(rightTowerOpen, LOW);
}

void closeDoor(){
  // Replace this with outputs to the solenoids
  digitalWrite(leftTowerClose, HIGH);
  digitalWrite(rightTowerClose, HIGH);
  digitalWrite(outputPin, LOW); 

  delay(1000);
  digitalWrite(leftTowerClose, LOW);
  digitalWrite(rightTowerClose, LOW);  
}

void loop() {  
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    lastByte = incomingByte;
    
    // Process the value
    if (incomingByte == '1') {
      // Solenoid trigger signal received
      // TODO: CHECK THAT WE'RE IN ARMED STATE
      // TODO: CHANGE STATE
      openDoor();
    } 
    else if (incomingByte == '0') {
      // Do nothing for now
    }
    // Ignore any other characters
  }

  // Close door
  
  if (digitalRead(resetPin) == LOW){
    // TODO: CHECK THAT WE'RE IN OPEN STATE
    // TODO: CHANGE STATE
    closeDoor(); 
  }
  
  
  // Small delay to avoid overwhelming the serial buffer
  delay(50);
}