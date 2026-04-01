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
volatile static int count = 0;
ISR(INT4_vect) {
    currTime = millis();
    if(currTime - lastTime > THRESHOLD)
    {
        bool buttonpressed = (PINE & 0b00010000);
        if(buttonpressed && buttonState == STATE_RUNNING) {
            buttonState = STATE_STOPPED;
            stateChanged = true;
        }
        else if(!buttonpressed && buttonState == STATE_STOPPED) {
            if(count == 1)
            {
                buttonState = STATE_RUNNING;
                stateChanged = true;
                count = 0;
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
volatile static bool colorWindowDone = false;
static uint32_t getFrequency() {
    frequencyCounter = 0;
    colorWindowDone = false;
    TCNT3 = 0;

    EIMSK = 0b00110000;  // Timer1 external clock on T1 rising edge
    TCCR3B = 0b00001100;  // Timer3 CTC, prescaler 256
    while(!colorWindowDone);
    return (uint32_t)(frequencyCounter) * 10; // 100 ms window -> Hz
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    PORTA &= 0b11111101;
    PORTA |= 0b00000001;
    redMode();
    *r = getFrequency();  // red,   in Hz
    // delay(10);
    greenMode();
    *g = getFrequency();  // green, in Hz
    // delay(10);
    blueMode();
    *b = getFrequency();  // blue,  in Hz
    // delay(10);
}

ISR(TIMER3_COMPA_vect) {
    EIMSK = 0b00010000;
    TCCR3B = 0;
    colorWindowDone = true;
}

ISR(INT5_vect) {
    frequencyCounter += 1;
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
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
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
    DDRE = 0b00000000;
    EICRB = 0b00001101;
    EIMSK = 0b00010000;

    DDRB = 0b00000000;
    TCCR1A = 0b0;
    TCCR1B = 0b0;
    TCNT1 = 0;

    TCCR3A = 0b0;
    TCCR3B = 0b0;
    TIMSK3 = 0b00000010;
    TCNT3 = 0;
    OCR3A = 6250;

    DDRA = 0b00001111;

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
