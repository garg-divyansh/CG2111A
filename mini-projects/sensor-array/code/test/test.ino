#define OUT_PIN 3   // TCS3200 OUT -> Arduino pin 11

void setup() {
  Serial.begin(9600);

  pinMode(OUT_PIN, INPUT);

  // Configure scaling pins (S0 HIGH, S1 LOW)
  pinMode(22, OUTPUT);   // S0
  pinMode(23, OUTPUT);   // S1

  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);

  // Set color filter (Red first)
  pinMode(24, OUTPUT);   // S2
  pinMode(25, OUTPUT);   // S3

  digitalWrite(24, LOW);
  digitalWrite(25, LOW);
}

void loop() {

  unsigned long highTime = pulseIn(OUT_PIN, HIGH);
  unsigned long lowTime  = pulseIn(OUT_PIN, LOW);

  unsigned long period = highTime + lowTime;

  if (period > 0) {
    float frequency = 1000000.0 / period;
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
  } 
  else {
    Serial.println("No signal detected");
  }

  delay(500);
}