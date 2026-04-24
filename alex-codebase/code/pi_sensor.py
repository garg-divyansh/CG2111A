#!/usr/bin/env python3
"""
Raspberry Pi Sensor Interface.

Provides reliable packet communication with Arduino and a CLI for user commands.
Packet format: MAGIC (2B) | TPacket (100B) | CHECKSUM (1B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select
from second_terminal.packets import *
from second_terminal import relay
import tty
import termios
import atexit

_fd = sys.stdin.fileno()
_old_settings = termios.tcgetattr(_fd)

def enableRawMode():
    """Enable raw mode for stdin to read single characters without buffering."""
    tty.setcbreak(_fd)

def disableRawMode():
    """Restore original terminal settings."""
    termios.tcsetattr(_fd, termios.TCSADRAIN, _old_settings)

atexit.register(disableRawMode)

# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result

def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Synchronize to the start of a frame by finding the magic bytes
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING  # Current E-Stop state: STATE_RUNNING or STATE_STOPPED


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form.

    The 'data' field carries an optional debug string from the Arduino.
    When non-empty, it is printed automatically so you can embed debug
    messages in any outgoing TPacket on the Arduino side (set pkt.data to
    a null-terminated string up to 31 characters before calling sendFrame).
    This works like Serial.print(), but sends output to the Pi terminal.
    """
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
        elif cmd == RESP_COLOR:
            print(f"R: {pkt['params'][0]} Hz, G: {pkt['params'][1]} Hz, B: {pkt['params'][2]} Hz")
            red = pkt['params'][0]
            green = pkt['params'][1]
            blue = pkt['params'][2]
            # Determine the dominant color based on frequency values
            if red > green and red > blue:
                print("Detected color: RED")
            elif green > red and green > blue:
                print("Detected color: GREEN")
            elif blue > red and blue > green:
                print("Detected color: BLUE")
        else:
            print(f"Response: unknown command {cmd}")
        # Print the optional debug string from the data field.
        # On the Arduino side, fill pkt.data before calling sendFrame() to
        # send debug messages to this terminal (similar to Serial.print()).
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        # Handle message packets from Arduino
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
    print("Sending color command...")
    sendCommand(COMMAND_COLOR, data=b'')


# ----------------------------------------------------------------
# CAMERA
# ----------------------------------------------------------------

from alex_camera import cameraOpen, cameraClose, captureGreyscaleFrame, renderGreyscaleFrame
_camera = cameraOpen()
_frames_remaining = 15   # frames remaining before further captures are refused


def handleCameraCommand():
    """
    Capture and display a greyscale frame.
    Gates on E-Stop and frame count.
    """
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return

    global _frames_remaining
    if _frames_remaining <= 0:
        print("Refused: no frames remaining.")
        return
    
    frame = captureGreyscaleFrame(_camera)
    renderGreyscaleFrame(frame)
    _frames_remaining -= 1
    print(f"Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# Key mappings:
#   e  send E-Stop command
#   c  request color reading
#   p  capture camera frame
#   w/a/s/d  move (continuous)
#   x  stop movement
#   + -  adjust something

def handleMoveCommand(line):
    if isEstopActive():
        print("Refused: E-Stop is active.")
        return
    sendCommand(COMMAND_MOVE, data=line.encode('ascii'))

def getKey():
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    if rlist:
        return sys.stdin.read(1)
    return None


def runCommandInterface():
    """
    Real-time command loop with continuous keypress handling.
    Movement keys (w/a/s/d) are continuous.
    Other commands are one-shot.
    """

    print("Sensor interface ready.")
    print("Controls: w/a/s/d = move (hold), x = stop, c = color, p = camera, e = E-stop")
    print("Press Ctrl+C to exit.\n")

    last_move_key = None
    last_key_time = time.time()
    KEY_TIMEOUT = 0.2  # seconds before auto-stop

    while True:
        # 1. Handle incoming Arduino packets
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(
                    packFrame(pkt['packetType'], pkt['command'], pkt['data'], pkt['params'])
                )

        # 2. Read key (non-blocking, single char)
        key = getKey()

        if key:
            key = key.lower()
            if key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt
            print(f"\rKey pressed: '{key}'   \n", end='', flush=True)
            # Handle continuous movement keys
            if key in ['w', 'a', 's', 'd']:
                if key != last_move_key:
                    handleMoveCommand(key)
                    last_move_key = key
                last_key_time = time.time()

            # Immediate stop
            elif key == 'x':
                handleMoveCommand('x')
                last_move_key = None

            # One-shot commands
            elif key == 'e':
                print("Sending E-Stop command...")
                sendCommand(COMMAND_ESTOP)
            
            elif key == 'r':
                print("Sending E-Stop release command...")
                sendCommand(COMMAND_RELEASE)

            elif key == 'c':
                handleColorCommand()

            elif key == 'p':
                handleCameraCommand()

            elif key in ['+', '-']:
                handleMoveCommand(key)

            else:
                print(f"Unknown input: '{key}'")

        # 3. Detect key release (auto-stop)
        if last_move_key and (time.time() - last_key_time > KEY_TIMEOUT):
            handleMoveCommand('x')
            last_move_key = None

        # 4. Handle second terminal relay
        relay.checkSecondTerminal(_ser)

        time.sleep(0.02)

# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    # Initialize serial connection, relay, and raw mode, then run the command interface
    openSerial()
    relay.start()
    enableRawMode()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        disableRawMode()
        closeSerial()
        cameraClose(_camera)
        relay.shutdown()
