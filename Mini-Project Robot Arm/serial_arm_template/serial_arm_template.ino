#include <Servo.h>

const int BASE_PIN = 14;      // A0
const int SHOULDER_PIN = 15;  // A1
const int ELBOW_PIN = 16;     // A2
const int GRIPPER_PIN = 17;   // A3

Servo base, shoulder, elbow, gripper;

int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;
int msPerDeg = 10;

int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

void moveSmooth(Servo *sv, int *cur, int target) {
  if (!sv || !cur) return;

  target = constrain(target, 0, 180);
  int step = (target > *cur) ? 1 : -1;

  while (*cur != target) {
    *cur += step;
    sv->write(*cur);
    delay(msPerDeg);
  }
}

void homeAll() {
  moveSmooth(&base, &basePos, 90);
  moveSmooth(&shoulder, &shoulderPos, 90);
  moveSmooth(&elbow, &elbowPos, 90);
  moveSmooth(&gripper, &gripperPos, 90);
}

void setup() {
  Serial.begin(115200);

  base.attach(BASE_PIN);
  shoulder.attach(SHOULDER_PIN);
  elbow.attach(ELBOW_PIN);
  gripper.attach(GRIPPER_PIN);

  base.write(basePos);
  shoulder.write(shoulderPos);
  elbow.write(elbowPos);
  gripper.write(gripperPos);
}

void loop() {
  if (!Serial.available()) return;

  // Reads a string with the command until newline
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();                 // Remove any extra whitespace
  if (!cmd.length()) return;  // didn't read anything

  // Handle the home command
  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  // All subsequent commands need to have 4 characters
  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  // c is now the command character
  char c = cmd.charAt(0);
  // val is the numerical value of the argument
  int val = parse3(&cmd.substring(1));
  if (val < 0) {
    Serial.println("ERROR: Argument not valid");
    return;
  }

  // Vddd sets velocity as ms per degree
  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    msPerDeg = val;
    return;
  } else if (c == 'B') {
    Serial.print("Moving base to ");
    Serial.println(val);
    moveSmooth(&base, &basePos, val);
    return;
  } else if (c == 'S') {
    Serial.print("Moving shoulder to ");
    Serial.println(val);
    moveSmooth(&shoulder, &shoulderPos, val);
    return;
  } else if (c == 'E') {
    Serial.print("Moving elbow to ");
    Serial.println(val);
    moveSmooth(&elbow, &elbowPos, val);
    return;
  } else if (c == 'G') {
    Serial.print("Moving gripper to ");
    Serial.println(val);
    moveSmooth(&gripper, &gripperPos, val);
    return;
  } else {
    Serial.println("ERROR: Unknown command");
    return;
  }
}
