# Drilling HMI

A desktop application for controlling and monitoring the drilling device over UART.

## Requirements

- Python 3.8 or newer — download from https://www.python.org/downloads/
  - During installation, tick **"Add Python to PATH"** on the first screen
- The ESP32 connected via USB (USB-to-serial adapter or direct USB)

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

A window will open. Select the correct COM port from the dropdown and click **Connect**.

To find the correct COM port: open Device Manager → Ports (COM & LPT) → look for your USB serial device.

## Project structure

```
drilling-hmi/
├── main.py            # Entry point — run this
├── protocol.py        # Message building and parsing (checksum, framing)
├── serial_worker.py   # Background thread handling UART communication
├── gui.py             # All GUI windows and widgets
├── requirements.txt   # Python dependencies
└── README.md          # This file
```

## Notes

- Baud rate is fixed at 38400 as per the protocol spec
- The application will not send commands if not connected
- The STATE command is sent automatically every 500 ms when connected
