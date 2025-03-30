// Arduino code to read a binary value from serial and set pin D2 accordingly
const int outputPin = 2;        // Using digital pin D2
const int latchPin = 5;         // Using digital pin D5 as latch LED indicator
const int resetPin = 6;         // Using digital pin D6 to reset the latch

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Initialize pinModes
  pinMode(outputPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  
  
  // Initialize pins
  digitalWrite(outputPin, LOW);
  digitalWrite(latchPin, LOW);
  
  // Wait for serial port to connect
  /*while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }*/
}


char lastByte;              // Holds most recent serial state
bool latched = false;       // Has the door already been opened?

void loop() {
  Serial.println(lastByte);
  // Check if data is available to read
  if (Serial.available() > 0) {
    //blink();
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

void blink(){
    digitalWrite(outputPin, HIGH);
    delay(200);
    digitalWrite(outputPin, LOW);
    delay(200);
    digitalWrite(outputPin, HIGH);
    delay(200);
    digitalWrite(outputPin, LOW);
}