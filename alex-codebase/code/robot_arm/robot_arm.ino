#include "Arduino.h"
#include <avr/interrupt.h>

// Port B pin masks — Mega digital pins 50–53
#define BASE_PIN     (1 << 3)  // PB3 → Digital 50
#define SHOULDER_PIN (1 << 2)  // PB2 → Digital 51
#define ELBOW_PIN    (1 << 1)  // PB1 → Digital 52
#define GRIPPER_PIN  (1 << 0)  // PB0 → Digital 53

// Constrain limits for the 4 servos
#define BASE_MIN      5
#define BASE_MAX      180
#define SHOULDER_MIN  70
#define SHOULDER_MAX  170
#define ELBOW_MIN     0
#define ELBOW_MAX     90
#define GRIPPER_MIN   80
#define GRIPPER_MAX   95

char c = '\u0000';
int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;
volatile int msPerDeg = 10;
volatile uint8_t currPin = 0;

int angle_to_pulse(int angle) {
  return 1000 + ((long)angle * 4000 / 180);
}

int parse3(const String *s) {
  if (!s || s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return s->toInt();
}

ISR(TIMER5_COMPA_vect) {
  if (currPin != 0) PORTB |= currPin;
}

ISR(TIMER5_COMPB_vect) {
  if (currPin != 0) PORTB &= ~currPin;
}

void moveSmooth(uint8_t servoPin, int *cur, int target) {
  PORTB &= ~currPin;
  currPin = servoPin;

  int currMin = 0, currMax = 180;
  if      (servoPin == BASE_PIN)     { currMin = BASE_MIN;     currMax = BASE_MAX;     }
  else if (servoPin == SHOULDER_PIN) { currMin = SHOULDER_MIN; currMax = SHOULDER_MAX; }
  else if (servoPin == ELBOW_PIN)    { currMin = ELBOW_MIN;    currMax = ELBOW_MAX;    }
  else                               { currMin = GRIPPER_MIN;  currMax = GRIPPER_MAX;  }

  target = constrain(target, currMin, currMax);
  int current_ticks = angle_to_pulse(*cur);
  int target_ticks  = angle_to_pulse(target);

  if (target_ticks > current_ticks) {
    for (int ticks = current_ticks; ticks <= target_ticks; ticks += (4000 / 180)) {
      OCR5B = ticks;
      delay(msPerDeg);
    }
  } else {
    for (int ticks = current_ticks; ticks >= target_ticks; ticks -= (4000 / 180)) {
      OCR5B = ticks;
      delay(msPerDeg);
    }
  }

  OCR5B = target_ticks;
  *cur = target;
}

void homeAll() {
  moveSmooth(BASE_PIN,     &basePos,     90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 90);
  moveSmooth(ELBOW_PIN,    &elbowPos,    90);
  moveSmooth(GRIPPER_PIN,  &gripperPos,  90);
}

void setup() {
  Serial.begin(115200);

  // Set PORTB pins (50–53) as output
  DDRB |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

  cli();
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (1 << CS51);  // CTC mode, prescaler 8
  OCR5A  = 40000;                        // 20ms period
  OCR5B  = 3000;                         // 1.5ms initial pulse (midpoint)
  TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);
  sei();

  homeAll();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  c = cmd.charAt(0);
  int val = parse3(&cmd.substring(1));

  if (val < 0) {
    Serial.println("ERROR: Argument not valid");
    return;
  }

  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    msPerDeg = constrain(val, 5, 100);
    return;

  } else if (c == 'B') {
    Serial.print("Moving base to ");     
    Serial.println(val);
    moveSmooth(BASE_PIN, &basePos, val);

  } else if (c == 'S') {
    Serial.print("Moving shoulder to "); 
    Serial.println(val);
    moveSmooth(SHOULDER_PIN, &shoulderPos, val);

  } else if (c == 'E') {
    Serial.print("Moving elbow to ");   
    Serial.println(val);
    moveSmooth(ELBOW_PIN, &elbowPos, val);

  } else if (c == 'G') {
    Serial.print("Moving gripper to "); 
    Serial.println(val);
    moveSmooth(GRIPPER_PIN, &gripperPos, val);

  } else {
    Serial.println("ERROR: Unknown command");
  }
}