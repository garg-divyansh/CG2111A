#include "Arduino.h"
#include <avr/interrupt.h>

#define BASE_PIN (1 << 0)      // A0
#define SHOULDER_PIN (1 << 1)  // A1
#define ELBOW_PIN (1 << 2)     // A2
#define GRIPPER_PIN (1 << 3)   // A3

// Constrain limits for the 4 servos
#define BASE_MIN 0
#define BASE_MAX 180
#define SHOULDER_MIN 70
#define SHOULDER_MAX 170
#define ELBOW_MIN 0
#define ELBOW_MAX 95
#define GRIPPER_MIN 80
#define GRIPPER_MAX 95

char c = '\u0000';
int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;
volatile int msPerDeg = 10;
volatile uint8_t currPin = 0;

// Helper to convert angle to ticks (0.5us resolution)
// 0 deg = 1000 ticks (0.5ms), 180 deg = 5000 ticks (2.5ms)
int angle_to_pulse(int angle) {
  return 1000 + ((long)angle * 4000 / 180);
}

int parse3(const String *s) {
  if (!s || s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return s->toInt();
}

ISR(TIMER1_COMPA_vect) {
  if (currPin != 0) {
    PORTC |= currPin;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (currPin != 0) {
    PORTC &= ~currPin;
  }
}

void moveSmooth(uint8_t servoPin, int *cur, int target) {
  PORTC &= ~currPin;  // Force old pin low immediately
  currPin = servoPin;

  // Constrain target to prevent physical damage
  int currMin = 0;
  int currMax = 180;
  if (servoPin == BASE_PIN) {
    currMin = BASE_MIN;
    currMax = BASE_MAX;
  } else if (servoPin == SHOULDER_PIN) {
    currMin = SHOULDER_MIN;
    currMax = SHOULDER_MAX;
  } else if (servoPin == ELBOW_PIN) {
    currMin = ELBOW_MIN;
    currMax = ELBOW_MAX;
  } else {
    currMin = GRIPPER_MIN;
    currMax = GRIPPER_MAX;
  }
  target = constrain(target, currMin, currMax);

  int current_ticks = angle_to_pulse(*cur);
  int target_ticks = angle_to_pulse(target);

  // Move gradually by updating OCR1B
  if (target_ticks > current_ticks) {
    for (int ticks = current_ticks; ticks <= target_ticks; ticks += (4000 / 180)) {
      OCR1B = ticks;    // Update pulse width
      delay(msPerDeg);  
    }
  } else {
    for (int ticks = current_ticks; ticks >= target_ticks; ticks -= (4000 / 180)) {
      OCR1B = ticks;
      delay(msPerDeg);
    }
  }

  // Ensure we land exactly on target
  OCR1B = target_ticks;
  *cur = target;
}

void homeAll() {
  moveSmooth(BASE_PIN, &basePos, 90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 90);
  moveSmooth(ELBOW_PIN, &elbowPos, 90);
  moveSmooth(GRIPPER_PIN, &gripperPos, 90);
}

void setup() {
  Serial.begin(115200);

  DDRC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
  cli();

  // Mode: CTC with OCR1A as TOP
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler 8

  // Set Period to 20ms (40,000 ticks)
  OCR1A = 40000;

  // Set Initial Pulse to 1.5ms (Midpoint)
  OCR1B = 3000;

  // Enable Interrupts for Match A (Period) and Match B (Pulse)
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);

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
    msPerDeg = val;
    return;
  } else {
    if (c == 'B') {
      Serial.print("Moving base to ");
      Serial.println(val);
      moveSmooth(BASE_PIN, &basePos, val);
      return;
    } else if (c == 'S') {
      Serial.print("Moving shoulder to ");
      Serial.println(val);
      moveSmooth(SHOULDER_PIN, &shoulderPos, val);
      return;
    } else if (c == 'E') {
      Serial.print("Moving elbow to ");
      Serial.println(val);
      moveSmooth(ELBOW_PIN, &elbowPos, val);
      return;
    } else if (c == 'G') {
      Serial.print("Moving gripper to ");
      Serial.println(val);
      moveSmooth(GRIPPER_PIN, &gripperPos, val);
      return;
    } else {
      Serial.println("ERROR: Unknown command");
      return;
    }
  }
}