"""Drilling HMI (PyQt6) launcher: python main.py"""
import sys
from PyQt6.QtWidgets import QApplication
from comm_worker import CommWorker
from gui import App


def main():
    qt_app = QApplication(sys.argv)
    qt_app.setStyle('Fusion')
    worker = CommWorker()
    window = App(worker)
    window.show()
    try:
        return qt_app.exec()
    finally:
        worker.disconnect()


if __name__ == '__main__':
    sys.exit(main())
