#!/usr/bin/env python


import sys
from PyQt5.QtWidgets import QApplication
from mission_control_ui import MissionControl
import signal 

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MissionControl()
    window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
