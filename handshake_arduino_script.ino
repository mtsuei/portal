// Arduino code to read a binary value from serial and set pin D2 accordingly
const int outputPin = 2;  // Using digital pin D2

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Set pin D2 as output
  pinMode(outputPin, OUTPUT);
  
  // Initialize pin to LOW
  digitalWrite(outputPin, LOW);
  
  // Wait for serial port to connect
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }
}

char lastByte;
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
    } 
    else if (incomingByte == '0') {
      digitalWrite(outputPin, LOW);
      Serial.println("Pin D2 set to LOW");
    }
    // Ignore any other characters
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
