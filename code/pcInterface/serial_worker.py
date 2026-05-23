"""
serial_worker.py

Runs a background daemon thread that:
  - Reads commands from tx_queue and sends them over UART
  - Continuously reads from UART, assembles complete messages, parses them,
    and puts results into rx_queue

The GUI thread never touches the serial port directly.
It only puts items into tx_queue and reads items from rx_queue.
"""

import threading
import queue
import serial
import time
import protocol

BAUD_RATE = 38400

# How long to wait for a response after sending a command (seconds)
RESPONSE_TIMEOUT = 1.0


class SerialWorker:
    def __init__(self):
        self.tx_queue = queue.Queue()  # GUI puts commands here
        self.rx_queue = queue.Queue()  # Worker puts parsed responses here

        self._serial = None
        self._thread = None
        self._stop_event = threading.Event()
        self._connected = False
        self._debug_buffer = ""

    def connect(self, port: str) -> bool:
        """
        Open the serial port and start the background thread.
        Returns True on success, False on failure.
        """
        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            self._connected = True
            return True
        except serial.SerialException as e:
            self.rx_queue.put({"error": str(e)})
            return False

    def disconnect(self):
        """Stop the background thread and close the serial port."""
        self._stop_event.set()
        self._connected = False
        if self._serial and self._serial.is_open:
            self._serial.close()

    def is_connected(self) -> bool:
        return self._connected

    def send(self, raw_bytes: bytes):
        """Queue a raw framed message for sending. Call from the GUI thread."""
        if self._connected:
            self.tx_queue.put(raw_bytes)

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
                    self._serial.write(data)
                except queue.Empty:
                    break
                except serial.SerialException as e:
                    self.rx_queue.put({"error": str(e)})
                    self._connected = False
                    return

            # --- Read incoming bytes ---
            try:
                incoming = self._serial.read(64)  # non-blocking due to timeout=0.1
            except serial.SerialException as e:
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
