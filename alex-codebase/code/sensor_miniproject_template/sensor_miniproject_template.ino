/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"
static unsigned long lastTime = 0, currTime;

#define THRESHOLD 10 
#define DEFAULT_SPEED 130

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
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
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
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
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */

// arduino pin connections
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
// volatile static bool colorWindowDone = false;

static uint32_t getFrequency() {
    frequencyCounter = 0;
    // colorWindowDone = false;
    // TCNT3 = 0;

    EIMSK = 0b00001100;  // Timer1 external clock on T1 rising edge
    // TCCR3B = 0b00001100;  // Timer3 CTC, prescaler 256
    unsigned long startTime = millis();
    while(millis() - startTime < 100);  // Wait for 100 ms
    EIMSK = 0b00001000;
    return (uint32_t)(frequencyCounter) * 10; // 100 ms window -> Hz
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

/* ISR(TIMER3_COMPA_vect) {
    EIMSK = 0b00001000;
    TCCR3B = 0;
    colorWindowDone = true;
}*/

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
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static int speed = DEFAULT_SPEED;
static char lastCmd = 'x';
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            flag = true;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
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

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        case COMMAND_COLOR:
        {
            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR;
            // strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
            pkt.data[sizeof(pkt.data) - 1] = '\0';
            readColorChannels(&(pkt.params[0]), &(pkt.params[1]), &(pkt.params[2]));
            // pkt.params[0] = 5; pkt.params[1] = 5; pkt.params[2] = 5;
            sendFrame(&pkt);
            break;
        }
        case COMMAND_MOVE:
            // TODO (Activity 3): add your own command and response types for the motor driver.
            if(cmd->data[0] == 'w') {
                speed = DEFAULT_SPEED;
                forward(speed);
                lastCmd = 'w';
            }
            else if(cmd->data[0] == 'a') {
                speed = 210;
                cw(speed);
                lastCmd = 'a';
            }
            else if(cmd->data[0] == 's') {
                speed = DEFAULT_SPEED;
                backward(speed);
                lastCmd = 's'; 
            }
            else if(cmd->data[0] == 'd') {
                speed = 210;
                ccw(speed);
                lastCmd = 'd';
            }
            else if(cmd->data[0] == 'x') {
                stop();
                lastCmd = 'x';
                speed = DEFAULT_SPEED;
            }
            else if(cmd->data[0] == '+') {
                if(lastCmd != 'x') {
                    speed += 10;
                    speed = constrain(speed, 80, 255);
                }
            }
            else if(cmd->data[0] == '-') {
                if(lastCmd != 'x') {
                    speed -= 10;
                    speed = constrain(speed, 80, 255);
                }
            }
            if((cmd->data[0] == '+' || cmd->data[0] == '-') && lastCmd != 'x') {
                switch(lastCmd) {
                    case 'w': forward(speed); break;
                    case 'a': cw(speed); break;
                    case 's': backward(speed); break;
                    case 'd': ccw(speed); break;
                    case 'x': stop(); break;
                }
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

    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    DDRD = 0b00000000;
    EICRA = 0b01110000;
    EIMSK = 0b00001000;
    
    // DDRB = 0b00000000;

    // TCCR3A = 0b0;
    // TCCR3B = 0b0;
    // TIMSK3 = 0b00000010;
    // TCNT3 = 0;
    // OCR3A = 6250;

    DDRA = 0b00001111;
    DDRB = 0b00001111;
    TCCR5A = 0;
    TCCR5B = (1 << WGM52) | (1 << CS51);  // CTC mode, prescaler 8
    OCR5A  = 40000;                        // 20ms period
    OCR5B  = 3000;                         // 1.5ms initial pulse (midpoint)
    TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);

    sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}