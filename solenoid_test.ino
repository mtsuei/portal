const int pin = 6;
const int pin2 = 3;
bool waitingForInput = true;

void setup() {
  pinMode(pin, OUTPUT);
  pinMode(pin2, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin2, HIGH);  
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0 && waitingForInput) {
    while (Serial.available() > 0) {
      Serial.read();
      delay(10); 
    }

    // Toggle the pin
    if (digitalRead(pin) == LOW) {
      digitalWrite(pin, HIGH);
      digitalWrite(pin2, LOW);
      Serial.println("Pin 6 is now HIGH");
    } else {
      digitalWrite(pin, LOW);
      digitalWrite(pin2, HIGH);
      Serial.println("Pin 6 is now LOW");
    }

    waitingForInput = false; 
  }

  if (Serial.available() == 0) {
    waitingForInput = true;
  }
}
