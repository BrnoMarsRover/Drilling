"""
main.py

Entry point for the Drilling HMI application.
Run this file with: python main.py
"""

from serial_worker import SerialWorker
from gui import App


def main():
    worker = SerialWorker()
    app = App(worker)
    app.mainloop()
    worker.disconnect()


if __name__ == "__main__":
    main()
