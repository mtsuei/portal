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
const int outputPin = 2;        // Using digital pin D2
const int latchPin = 5;         // Using digital pin D5 as latch LED indicator
const int resetPin = 6;         // Using digital pin D6 to reset the latch

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
  pinMode(latchPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(piCommsPin_close, INPUT_PULLUP);
  pinMode(piCommsPin_start, INPUT_PULLUP);
  
  // Initialize pins
  digitalWrite(outputPin, LOW);
  digitalWrite(latchPin, LOW);

  // Initialize mode
  // Arduino assumes door is open on first connection
  mode = 
  
  // Wait for serial port to connect
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
}


char lastByte;              // Holds most recent serial state
bool latched = false;       // Has the door already been opened?

void openDoor(){
  // Replace this with outputs to the solenoids
  delay(100);  
}

void closeDoor(){
  // Replace this with outputs to the solenoids
  delay(100);    
}

void loop() {  
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    lastByte = incomingByte;
    
    // Process the value
    if (incomingByte == '1') {
      digitalWrite(outputPin, HIGH);
      Serial.println("Pin D2 set to HIGH");
      if(!latched){
        latched = true; 
        digitalWrite(latchPin, HIGH);
      }
    } 
    else if (incomingByte == '0') {
      digitalWrite(outputPin, LOW);
      Serial.println("Pin D2 set to LOW");
    }
    // Ignore any other characters
  }

  // Check if we need to reset the latch
  // I don't think a debounce is needed
  Serial.println(digitalRead(resetPin));
  if(digitalRead(resetPin) == 1){
    latched = false;
  }
  
  
  // Small delay to avoid overwhelming the serial buffer
  delay(50);
}