/*
 * Sensor Mini-Project Arduino Sketch
 *
 * Files:
 *   packets.h        - TPacket protocol definitions.
 *   serial_driver.h  - Serial transport layer.
 *   sensor_miniproject_template.ino - Main application logic.
 */

#include "packets.h"
#include "serial_driver.h"
static unsigned long lastTime = 0, currTime;

#define THRESHOLD 10
#define DEFAULT_SPEED 130

static int speed = DEFAULT_SPEED;
static char lastCmd = 'x';

// =============================================================
// Packet Helpers
// =============================================================

/*
 * Send a response packet with the given response type and parameter.
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a status response with the current state.
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop State Machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * E-Stop ISR: Handles button press/release with debouncing.
 * Transitions between RUNNING and STOPPED states.
 */
volatile static bool flag = false;
volatile static int count = 0;
ISR(INT3_vect) {
    currTime = millis();
    if(currTime - lastTime > THRESHOLD)
    {
        bool buttonpressed = (PIND & 0b00001000);
        if(buttonpressed && buttonState == STATE_RUNNING) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
            stop();
            lastCmd = 'x';
            speed = DEFAULT_SPEED;
        }
        else if(!buttonpressed && buttonState == STATE_STOPPED) {
            if(count == 1 || flag)
            {
                buttonState = STATE_RUNNING;
                stateChanged = true;
                count = 0;
                flag = false;
            }
            else
            {
                stateChanged = false;
                count++;
            }
        }
        lastTime = currTime;
    }
}


// =============================================================
// Color Sensor (TCS3200)
// =============================================================

/*
 * Color sensor functions using TCS3200.
 * Measures red, green, blue frequencies in Hz.
 */

// Pin connections:
// output: pin 19
// s0: pin 22
// s1: pin 23
// s2: pin 24
// s3: pin 25

static void redMode() {
    PORTA &= 0b11110011;
}

static void blueMode() {
    PORTA &= 0b11111011;
    PORTA |= 0b00001000;
}

static void greenMode() {
    PORTA &= 0b11110011;
    PORTA |= 0b00001100;
}
volatile static uint32_t frequencyCounter = 0;

static uint32_t getFrequency() {
    frequencyCounter = 0;
    EIMSK = 0b00001100;
    unsigned long startTime = millis();
    while(millis() - startTime < 100);  // 100 ms measurement window
    EIMSK = 0b00001000;
    return (uint32_t)(frequencyCounter) * 10; // Convert to Hz
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    PORTA &= 0b11111101;
    PORTA |= 0b00000001;
    redMode();
    *r = getFrequency();  // red,   in Hz
    greenMode();
    *g = getFrequency();  // green, in Hz
    blueMode();
    *b = getFrequency();  // blue,  in Hz
}

ISR(INT2_vect) {
    frequencyCounter += 1;
}

// =============================================================
// Robot Arm
// =============================================================

// Port B pin masks — Mega digital pins 50–53
#define BASE_PIN     (1 << 3)  // PB3 → Digital 50
#define SHOULDER_PIN (1 << 2)  // PB2 → Digital 51
#define ELBOW_PIN    (1 << 1)  // PB1 → Digital 52
#define GRIPPER_PIN  (1 << 0)  // PB0 → Digital 53

// Servo angle limits
#define BASE_MIN      5
#define BASE_MAX      180
#define SHOULDER_MIN  70
#define SHOULDER_MAX  175
#define ELBOW_MIN     0
#define ELBOW_MAX     90
#define GRIPPER_MIN   80
#define GRIPPER_MAX   95

char c = '\u0000';
int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;
volatile int msPerDeg = 10;
volatile uint8_t currPin = 0;

static int angle_to_pulse(int angle) {
  return 1000 + ((long)angle * 4000 / 180);
}

static int parse3(const String *s) {
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

static void moveSmooth(uint8_t servoPin, int *cur, int target) {
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

static void homeAll() {
  moveSmooth(BASE_PIN,     &basePos,     90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 90);
  moveSmooth(ELBOW_PIN,    &elbowPos,    90);
  moveSmooth(GRIPPER_PIN,  &gripperPos,  90);
}

static void robotArmHandler(char joint, int angle) {
    switch(joint) {
        case 'H': homeAll(); break;
        case 'V': msPerDeg = constrain(angle, 5, 100); break;
        case 'B': moveSmooth(BASE_PIN, &basePos, angle); break;
        case 'S': moveSmooth(SHOULDER_PIN, &shoulderPos, angle); break;
        case 'E': moveSmooth(ELBOW_PIN, &elbowPos, angle); break;
        case 'G': moveSmooth(GRIPPER_PIN, &gripperPos, angle); break;
    }

}


// =============================================================
// Command Handler
// =============================================================

/*
 * Processes incoming commands from the Pi.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            stop();
            lastCmd = 'x';
            speed = DEFAULT_SPEED;
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            flag = true;
            sei();
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        case COMMAND_COLOR:
        {
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            readColorChannels(&(pkt.params[0]), &(pkt.params[1]), &(pkt.params[2]));
            sendFrame(&pkt);
            break;
        }
        case COMMAND_MOVE:
            if(cmd->data[0] == 'w') {
                if(lastCmd != 'w') {
                    speed = DEFAULT_SPEED;
                }
                forward(speed);
                lastCmd = 'w';
            }
            else if(cmd->data[0] == 'a') {
                if(lastCmd != 'a') {
                    speed = 210;
                }
                cw(speed);
                lastCmd = 'a';
            }
            else if(cmd->data[0] == 's') {
                if(lastCmd != 's') {
                    speed = DEFAULT_SPEED;
                }
                backward(speed);
                lastCmd = 's';
            }
            else if(cmd->data[0] == 'd') {
                if(lastCmd != 'd') {
                    speed = 210;
                }
                ccw(speed);
                lastCmd = 'd';
            }
            else if(cmd->data[0] == 'x') {
                stop();
            }
            else if(cmd->data[0] == '+') {
                speed += 10;
                speed = constrain(speed, 80, 255);
            }
            else if(cmd->data[0] == '-') {
                speed -= 10;
                speed = constrain(speed, 80, 255);
            }
            if((cmd->data[0] == '+' || cmd->data[0] == '-') && lastCmd != 'x') {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command = RESP_OK;
                snprintf(pkt.data, sizeof(pkt.data), "Speed: %d", speed);
                sendFrame(&pkt);

            }
            break;
        case COMMAND_ARM:
            robotArmHandler(cmd->data[0], cmd->params[0]);
            break;

        case COMMAND_RELEASE:
            cli();
            buttonState  = STATE_RUNNING;
            stateChanged = false;
            flag = true;
            sei();
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;


    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialize serial at 9600 baud
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // Configure button pin and interrupt
    DDRD = 0b00000000;
    EICRA = 0b01110000;
    EIMSK = 0b00001000;

    // Configure color sensor pins
    DDRA = 0b00001111;
    // Configure robot arm pins
    DDRB |= 0b00001111;
    TCCR5A = 0;
    TCCR5B = (1 << WGM52) | (1 << CS51);
    OCR5A  = 40000;
    OCR5B  = 3000;
    TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);

    sei();
}

void loop() {
    // Report E-Stop state changes
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // Process incoming commands
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}