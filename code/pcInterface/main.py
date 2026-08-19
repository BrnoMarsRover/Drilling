"""
main.py

Entry point for the Drilling HMI application.
Run this file with: python main.py
"""

from comm_worker import CommWorker
from gui import App


def main():
    worker = CommWorker()
    app = App(worker)
    app.mainloop()
    worker.disconnect()


if __name__ == "__main__":
    main()
