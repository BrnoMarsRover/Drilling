"""
comm_worker.py

Runs a background daemon thread that talks to the drill controller using
either of two transports, chosen at connect time:

  - USB serial  : direct connection to the ESP32 (used during standalone
                   bench testing, exactly as before).
  - Localhost UDP: used once the drill is mounted on the rover. This app
                   sends framed messages to a UDP port that the main rover
                   communication app is listening on, and listens on its
                   own UDP port for messages the main app forwards back
                   from the drill. The main app's job of actually getting
                   those bytes to/from the drill over the rover link is a
                   black box from here.

Only one transport is active at a time. Once connected, the rest of the
app (gui.py) doesn't need to know or care which one is in use — it only
ever touches tx_queue / rx_queue / connect_serial() / connect_udp() /
disconnect() / is_connected() / send().

  - Reads commands from tx_queue and sends them over the active transport
  - Continuously reads incoming bytes, assembles complete framed messages,
    parses them, and puts results into rx_queue

The GUI thread never touches the serial port or socket directly.
"""

import threading
import queue
import socket
import serial
import time
import protocol

BAUD_RATE = 38400

MODE_SERIAL = "serial"
MODE_UDP    = "udp"

UDP_HOST         = "127.0.0.1"   # both endpoints run on this machine
UDP_READ_TIMEOUT = 0.1           # seconds — keeps the loop responsive to disconnect()
UDP_MAX_DATAGRAM = 4096          # generous upper bound for one incoming datagram

# How long to wait for a response after sending a command (seconds)
RESPONSE_TIMEOUT = 1.0


class CommWorker:
    def __init__(self):
        self.tx_queue = queue.Queue()  # GUI puts commands here
        self.rx_queue = queue.Queue()  # Worker puts parsed responses here

        self._thread = None
        self._stop_event = threading.Event()
        self._connected = False
        self._debug_buffer = ""

        self._mode = None          # MODE_SERIAL or MODE_UDP once connected
        self._serial = None        # serial.Serial instance (serial mode)
        self._socket = None        # socket.socket instance (UDP mode)
        self._udp_remote_addr = None

    # ------------------------------------------------------------------ #
    #  Public interface                                                    #
    # ------------------------------------------------------------------ #

    def connect_serial(self, port: str) -> bool:
        """
        Open a direct USB serial connection to the ESP32.
        Returns True on success, False on failure.
        """
        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            self.rx_queue.put({"error": str(e)})
            return False

        self._mode = MODE_SERIAL
        self._start_thread()
        return True

    def connect_udp(self, local_port: int, remote_port: int) -> bool:
        """
        Open a localhost UDP socket in place of the serial connection.

        local_port  — port this app listens on. The main rover comms app
                      should send the drill's responses here.
        remote_port — port this app sends to. The main rover comms app
                      should be listening here.

        Returns True on success, False on failure.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((UDP_HOST, local_port))
            sock.settimeout(UDP_READ_TIMEOUT)
        except OSError as e:
            self.rx_queue.put({"error": str(e)})
            return False

        self._socket = sock
        self._udp_remote_addr = (UDP_HOST, remote_port)
        self._mode = MODE_UDP
        self._start_thread()
        return True

    def disconnect(self):
        """Stop the background thread and close whichever transport is open."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._connected = False

        if self._serial is not None:
            if self._serial.is_open:
                self._serial.close()
            self._serial = None

        if self._socket is not None:
            self._socket.close()
            self._socket = None

        self._udp_remote_addr = None
        self._mode = None

    def is_connected(self) -> bool:
        return self._connected

    def send(self, raw_bytes: bytes):
        """Queue a raw framed message for sending. Call from the GUI thread."""
        if self._connected:
            self.tx_queue.put(raw_bytes)

    # ------------------------------------------------------------------ #
    #  Internal — thread lifecycle                                         #
    # ------------------------------------------------------------------ #

    def _start_thread(self):
        self._debug_buffer = ""
        self._stop_event.clear()
        self._connected = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ #
    #  Internal — transport-specific I/O                                   #
    # ------------------------------------------------------------------ #

    def _write(self, data: bytes):
        if self._mode == MODE_SERIAL:
            self._serial.write(data)
        else:
            self._socket.sendto(data, self._udp_remote_addr)

    def _read(self) -> bytes:
        """Read whatever is available right now. Returns b'' on timeout."""
        if self._mode == MODE_SERIAL:
            return self._serial.read(64)  # non-blocking due to timeout=0.1
        try:
            data, _addr = self._socket.recvfrom(UDP_MAX_DATAGRAM)
            return data
        except socket.timeout:
            return b""

    # ------------------------------------------------------------------ #
    #  Internal — main loop (identical for both transports)                #
    # ------------------------------------------------------------------ #

    def _run(self):
        """
        Main loop of the background thread.
        Alternates between draining the TX queue and reading incoming bytes.
        """
        buffer = bytearray()

        while not self._stop_event.is_set():
            # --- Send any queued commands ---
            while not self.tx_queue.empty():
                try:
                    data = self.tx_queue.get_nowait()
                    self._write(data)
                except queue.Empty:
                    break
                except (serial.SerialException, OSError) as e:
                    self.rx_queue.put({"error": str(e)})
                    self._connected = False
                    return

            # --- Read incoming bytes ---
            try:
                incoming = self._read()
            except (serial.SerialException, OSError) as e:
                self.rx_queue.put({"error": str(e)})
                self._connected = False
                return

            if incoming:
                buffer.extend(incoming)
                buffer = self._process_buffer(buffer)

            time.sleep(0.005)  # small sleep to avoid busy-looping

    def _process_buffer(self, buffer: bytearray) -> bytearray:
        """
        Scan the buffer for complete messages (STX...ETX), parse them,
        and put results in rx_queue. Return the remaining unprocessed bytes.
        Bytes outside of a framed message are treated as debug text and
        forwarded to the log.

        Note: with UDP, each call to _read() returns one whole datagram
        rather than an arbitrary slice of a byte stream, but a single
        datagram can still contain more than one framed message (or, if
        the sender on the other end ever splits a message across two
        datagrams, a partial one) — so the same incremental assembly logic
        used for serial is reused here rather than assuming one datagram
        equals one message.
        """
        while len(buffer) > 0:
            b = buffer[0]

            if b != protocol.STX:
                # Outside a frame — collect as debug text
                char = chr(b)
                if char == '\n':
                    # Newline flushes the debug buffer
                    if self._debug_buffer:
                        self.rx_queue.put({"debug": self._debug_buffer})
                        self._debug_buffer = ""
                else:
                    self._debug_buffer += char
                buffer = buffer[1:]
                continue

            # b == STX — flush any pending debug text before parsing the frame
            if self._debug_buffer:
                self.rx_queue.put({"debug": self._debug_buffer})
                self._debug_buffer = ""

            # We need at least 5 bytes for a minimal message
            if len(buffer) < 5:
                break

            # The second byte is the payload length
            payload_length = buffer[1]
            # Full message length: STX + length_byte + payload + checksum + ETX
            full_length = 2 + payload_length + 2

            if len(buffer) < full_length:
                break  # Not enough bytes yet — wait for more

            message = bytes(buffer[:full_length])
            buffer = buffer[full_length:]

            parsed = protocol.parse_response(message)
            if parsed is not None:
                self.rx_queue.put(parsed)
            else:
                self.rx_queue.put({"error": "Malformed or corrupt message received"})

        return buffer
