# Drilling HMI

A desktop application for controlling and monitoring the drilling device.

It supports two connection modes, switchable in the app:

- **USB Serial** — direct connection to the ESP32 over UART. Used during
  standalone bench testing.
- **UDP (localhost)** — used once the drill is mounted on the rover. This
  app runs alongside the main rover communication app on the same laptop.
  Framed drill messages are sent over a localhost UDP socket to the main
  app, which is responsible for actually getting them to/from the drill
  over the rover link. Responses come back the same way.

## Requirements

- Python 3.8 or newer — download from https://www.python.org/downloads/
  - During installation, tick **"Add Python to PATH"** on the first screen
- For USB Serial mode: the ESP32 connected via USB (USB-to-serial adapter or direct USB)
- For UDP mode: the main rover communication app running on the same machine

## Installation

1. Open CMD and navigate to this folder:
   ```
   cd ...\Drilling\code\pcInterface
   ```

2. Install dependencies:
   ```
   pip install -r requirements.txt
   ```

## Running the application

```
python main.py
```

A window will open with a **Connection** bar at the top.

### USB Serial mode

Select **USB Serial**, choose the correct COM port from the dropdown, and
click **Connect**.

To find the correct COM port: open Device Manager → Ports (COM & LPT) →
look for your USB serial device.

### UDP (localhost) mode

Select **UDP (localhost)** and set two ports:

- **Listen port** — the port this app listens on. The main rover comms
  app must send the drill's responses to `127.0.0.1` on this port.
- **Send port** — the port this app sends to. The main rover comms app
  must be listening on `127.0.0.1` on this port.

These two ports must be different from each other, and must match
whatever the main app is configured to use on its side. Click **Connect**
to open the socket.

Note: UDP mode expects the main rover communication app to already be
running and listening — this app doesn't start or manage it.

The mode switch and its fields lock while connected; disconnect first to
change modes.

## Project structure

```
drilling-hmi/
├── main.py            # Entry point — run this
├── protocol.py        # Message building and parsing (checksum, framing)
├── comm_worker.py      # Background thread handling USB serial and UDP communication
├── gui.py              # All GUI windows and widgets
├── requirements.txt    # Python dependencies
└── README.md            # This file
```

## Notes

- Baud rate is fixed at 38400 as per the protocol spec (USB Serial mode only)
- The application will not send commands if not connected
- The STATE command is sent automatically every 500 ms when connected, regardless of mode
- `serial_worker.py` from earlier versions has been replaced by `comm_worker.py` — delete the old file if it's still present in this folder
