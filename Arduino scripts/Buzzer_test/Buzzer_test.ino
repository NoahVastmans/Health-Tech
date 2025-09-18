int buzzerPin = 2; // Pin connected to the buzzer

void setup() {
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  tone(buzzerPin, 1000); // 1 kHz tone
  delay(500);             // play for 500 ms
  noTone(buzzerPin);      // stop the tone
  delay(500);             // wait 500 ms
}