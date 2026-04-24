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
static unsigned long lastTime = 0, currTime;  // For debouncing E-Stop button

#define THRESHOLD 10  // Debounce threshold in ms
#define DEFAULT_SPEED 130  // Default motor speed

static int speed = DEFAULT_SPEED;  // Current motor speed
static char lastCmd = 'x';  // Last movement command

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
volatile static bool flag = false;  // Flag for release logic
volatile static int count = 0;  // Count for double-press detection
ISR(INT3_vect) {
    currTime = millis();
    if(currTime - lastTime > THRESHOLD)  // Debounce check
    {
        bool buttonpressed = (PIND & 0b00001000);  // Check if button is pressed
        if(buttonpressed && buttonState == STATE_RUNNING) {
            buttonState = STATE_STOPPED;  // Stop on press
            stateChanged = true;
            stop();
            lastCmd = 'x';
            speed = DEFAULT_SPEED;
        }
        else if(!buttonpressed && buttonState == STATE_STOPPED) {  // Release logic
            if(count == 1 || flag)  // Double press or flag set
            {
                buttonState = STATE_RUNNING;  // Resume
                stateChanged = true;
                count = 0;
                flag = false;
            }
            else
            {
                stateChanged = false;
                count++;  // Increment count for double press
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

// Set color sensor to red mode
static void redMode() {
    PORTA &= 0b11110011;
}

// Set color sensor to blue mode
static void blueMode() {
    PORTA &= 0b11111011;
    PORTA |= 0b00001000;
}

// Set color sensor to green mode
static void greenMode() {
    PORTA &= 0b11110011;
    PORTA |= 0b00001100;
}

volatile static uint32_t frequencyCounter = 0;  // Counter for frequency measurement

// Measure frequency over 100ms
static uint32_t getFrequency() {
    frequencyCounter = 0;
    EIMSK = 0b00001100;  // Enable INT2 for frequency counting
    unsigned long startTime = millis();
    while(millis() - startTime < 100);  // 100 ms measurement window
    EIMSK = 0b00001000;  // Disable INT2
    return (uint32_t)(frequencyCounter) * 10; // Convert to Hz
}

// Read RGB color channels
static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    PORTA &= 0b11111101;  // Enable color sensor
    PORTA |= 0b00000001;
    redMode();
    *r = getFrequency();  // red,   in Hz
    greenMode();
    *g = getFrequency();  // green, in Hz
    blueMode();
    *b = getFrequency();  // blue,  in Hz
}

// ISR for color sensor frequency counting
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

int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;  // Current servo positions
volatile int msPerDeg = 10;  // Milliseconds per degree for smooth movement
volatile uint8_t currPin = 0;  // Current servo pin being controlled

// Convert angle to pulse width for servo
static int angle_to_pulse(int angle) {
  return 1000 + ((long)angle * 4000 / 180);
}

// Parse a 3-digit string to int
static int parse3(const String *s) {
  if (!s || s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return s->toInt();
}

// Timer ISR for servo pulse start
ISR(TIMER5_COMPA_vect) {
  if (currPin != 0) PORTB |= currPin;
}

// Timer ISR for servo pulse end
ISR(TIMER5_COMPB_vect) {
  if (currPin != 0) PORTB &= ~currPin;
}

// Smoothly move servo to target angle
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

// Home all servos to 90 degrees
static void homeAll() {
  moveSmooth(BASE_PIN,     &basePos,     90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 90);
  moveSmooth(ELBOW_PIN,    &elbowPos,    90);
  moveSmooth(GRIPPER_PIN,  &gripperPos,  90);
}

// Handle robot arm commands
static void robotArmHandler(char joint, int angle) {
    switch(joint) {
        case 'H': homeAll(); break;  // Home all
        case 'V': msPerDeg = constrain(angle, 5, 100); break;  // Set speed
        case 'B': moveSmooth(BASE_PIN, &basePos, angle); break;  // Base
        case 'S': moveSmooth(SHOULDER_PIN, &shoulderPos, angle); break;  // Shoulder
        case 'E': moveSmooth(ELBOW_PIN, &elbowPos, angle); break;  // Elbow
        case 'G': moveSmooth(GRIPPER_PIN, &gripperPos, angle); break;  // Gripper
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
        case COMMAND_ESTOP:  // Emergency stop command
            cli();  // Disable interrupts
            stop();
            lastCmd = 'x';
            speed = DEFAULT_SPEED;
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            flag = true;
            sei();  // Enable interrupts
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

        case COMMAND_COLOR:  // Color sensor command
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
        case COMMAND_MOVE:  // Movement command
            if(cmd->data[0] == 'w') {  // Forward
                if(lastCmd != 'w') {
                    speed = DEFAULT_SPEED;
                }
                forward(speed);
                lastCmd = 'w';
            }
            else if(cmd->data[0] == 'a') {  // Turn left
                if(lastCmd != 'a') {
                    speed = 210;
                }
                cw(speed);
                lastCmd = 'a';
            }
            else if(cmd->data[0] == 's') {  // Backward
                if(lastCmd != 's') {
                    speed = DEFAULT_SPEED;
                }
                backward(speed);
                lastCmd = 's';
            }
            else if(cmd->data[0] == 'd') {  // Turn right
                if(lastCmd != 'd') {
                    speed = 210;
                }
                ccw(speed);
                lastCmd = 'd';
            }
            else if(cmd->data[0] == 'x') {  // Stop
                stop();
            }
            else if(cmd->data[0] == '+') {  // Increase speed
                speed += 10;
                speed = constrain(speed, 80, 255);
            }
            else if(cmd->data[0] == '-') {  // Decrease speed
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
        case COMMAND_ARM:  // Robot arm command
            robotArmHandler(cmd->data[0], cmd->params[0]);
            break;

        case COMMAND_RELEASE:  // Release E-Stop
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
    // Configure button pin and interrupt for E-Stop
    DDRD = 0b00000000;  // Set PORTD as input
    EICRA = 0b01110000;  // Rising/falling edge for INT3
    EIMSK = 0b00001000;  // Enable INT3

    // Configure color sensor pins (PORTA)
    DDRA = 0b00001111;  // Set lower 4 bits as output
    // Configure robot arm pins (PORTB)
    DDRB |= 0b00001111;  // Set lower 4 bits as output
    // Setup Timer5 for servo control
    TCCR5A = 0;
    TCCR5B = (1 << WGM52) | (1 << CS51);  // CTC mode, prescaler 8
    OCR5A  = 40000;  // 20ms period at 16MHz/8
    OCR5B  = 3000;   // Initial pulse width
    TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);  // Enable compare interrupts

    sei();  // Enable global interrupts
}

void loop() {
    // Report E-Stop state changes to Pi
    if (stateChanged) {
        cli();  // Disable interrupts temporarily
        TState state = buttonState;
        stateChanged = false;
        sei();  // Re-enable interrupts
        sendStatus(state);
    }

    // Process incoming commands from Pi
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}