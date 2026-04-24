/*
 * Serial transport layer and packet framing.
 *
 * USE_BAREMETAL_SERIAL controls the transport:
 *   0 - Uses Arduino Serial library.
 *   1 - Uses custom circular buffers and ISRs.
 */

#pragma once

#include <avr/interrupt.h>
#include <string.h>
#include "packets.h"

#define USE_BAREMETAL_SERIAL 1

#if USE_BAREMETAL_SERIAL

#define TX_BUFFER_SIZE  128
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE - 1)
#define RX_BUFFER_SIZE  256
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE - 1)

volatile uint8_t tx_buf[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0, tx_tail = 0;

volatile uint8_t rx_buf[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

// =============================================================
// USART0 initialisation (used when USE_BAREMETAL_SERIAL == 1)
// =============================================================

// Configure USART0 for 8N1 at the given baud rate with TX, RX, and
// RX Complete interrupt enabled.  ubrr = (F_CPU / (16 * baud)) - 1.
// For 9600 baud at 16 MHz: ubrr = 103.
void usartInit(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
}

/*
 * Enqueue data into TX buffer. Returns true if successful, false if no space.
 * Enables UDRE interrupt on success.
 */
bool txEnqueue(const uint8_t *data, uint8_t len) {
    // TODO
    uint8_t freeSpace = (tx_tail - tx_head - 1) & TX_BUFFER_MASK;
    if(freeSpace < len) {
        return false;
    }

    for(int i = 0; i<len; i++) {
        tx_buf[tx_head] = data[i];
        tx_head = (tx_head + 1) & TX_BUFFER_MASK;
    }

    UCSR0B |= 0b00100000;
    return true;
}

/*
 * TX Data Register Empty ISR: Send next byte from TX buffer.
 */
ISR(USART0_UDRE_vect) {
  UDR0 = tx_buf[tx_tail];
  tx_tail = (tx_tail + 1) & TX_BUFFER_MASK;
  if(tx_tail == tx_head) {
    UCSR0B &= ~0b00100000;
  }
}

/*
 * Dequeue data from RX buffer. Returns true if successful, false if insufficient data.
 */
bool rxDequeue(uint8_t *data, uint8_t len) {
    uint8_t availableBits = (rx_head - rx_tail - 1) & RX_BUFFER_MASK;
    if(availableBits < len) {
        return false;
    }
    for(int i = 0; i < len; i++) {
        data[i] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
    }
    return true;
}

/*
 * RX Complete ISR: Store received byte in RX buffer if space available.
 */
ISR(USART0_RX_vect) {
  uint8_t received = UDR0;
  uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;
  if(next != rx_tail) {
    rx_buf[rx_head] = received;
    rx_head = next;
  }
}

#endif


// =============================================================
// Framing: magic number + XOR checksum
// =============================================================

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/*
 * Send a framed packet.
 * Uses Serial.write() or TX buffer depending on USE_BAREMETAL_SERIAL.
 */
static void sendFrame(const TPacket *pkt) {
    uint8_t frame[FRAME_SIZE];
    frame[0] = MAGIC_HI;
    frame[1] = MAGIC_LO;
    memcpy(&frame[2], pkt, TPACKET_SIZE);
    frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);
#if USE_BAREMETAL_SERIAL
    while (!txEnqueue(frame, FRAME_SIZE))
        ;
#else
    Serial.write(frame, FRAME_SIZE);
#endif
}

/*
 * Receive a framed packet if available.
 * Returns true if a valid packet is extracted into *pkt.
 * Uses Serial or RX buffer depending on USE_BAREMETAL_SERIAL.
 */
static bool receiveFrame(TPacket *pkt) {
#if USE_BAREMETAL_SERIAL
    while (((rx_head - rx_tail) & RX_BUFFER_MASK) >= FRAME_SIZE) {
        uint8_t hi = rx_buf[rx_tail];
        uint8_t lo = rx_buf[(rx_tail + 1) & RX_BUFFER_MASK];

        if (hi == MAGIC_HI && lo == MAGIC_LO) {
            uint8_t frame[FRAME_SIZE];
            for (uint8_t i = 0; i < FRAME_SIZE; i++)
                frame[i] = rx_buf[(rx_tail + i) & RX_BUFFER_MASK];

            uint8_t expected = computeChecksum(&frame[2], TPACKET_SIZE);
            if (frame[FRAME_SIZE - 1] == expected) {
                memcpy(pkt, &frame[2], TPACKET_SIZE);
                rx_tail = (rx_tail + FRAME_SIZE) & RX_BUFFER_MASK;
                return true;
            }
        }

        rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
    }
    return false;
#else
    static uint8_t state = 0;
    static uint8_t raw[TPACKET_SIZE];
    static uint8_t index = 0;

    while (Serial.available() > 0) {
        uint8_t byte = (uint8_t)Serial.read();

        switch (state) {
            case 0:
                if (byte == MAGIC_HI) {
                    state = 1;
                }
                break;

            case 1:
                if (byte == MAGIC_LO) {
                    state = 2;
                    index = 0;
                } else if (byte != MAGIC_HI) {
                    state = 0;
                }
                break;

            case 2:
                raw[index++] = byte;
                if (index >= TPACKET_SIZE) {
                    state = 3;
                }
                break;

            case 3: {
                uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
                if (byte == expected) {
                    memcpy(pkt, raw, TPACKET_SIZE);
                    state = 0;
                    return true;
                }
                state = (byte == MAGIC_HI) ? 1 : 0;
                break;
            }
        }
    }
    return false;
#endif
}
