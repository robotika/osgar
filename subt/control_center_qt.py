"""
    Control Center for SubT Urban
"""

import threading
import math
import sys
import platform
from contextlib import suppress

from PyQt5.QtCore import pyqtSignal, QSize, Qt, QPointF
from PyQt5.QtGui import QPalette, QKeySequence, QPainter, QColor, QFont, QTransform, QIcon, QPolygonF
from PyQt5.QtWidgets import ( QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
                              QMessageBox, QMainWindow, QAction, QToolBar, QMenuBar, QCheckBox,
                              QDockWidget)

import osgar.record
import osgar.logger
import osgar.bus
import osgar.node


####################################################################################
CFG_LORA = {
  "version": 2,
  "robot": {
    "modules": {
      "cc": {
          "driver": "subt.control_center_qt:OsgarControlCenter",
          "init": {}
      },
      "lora": {
          "driver": "lora",
          "init": {}
      },
      "lora_serial": {
          "driver": "serial",
          "init": {"port": "/dev/lora" if platform.system() == 'Linux' else 'COM35',
                   "speed": 115200}
      },
    },
    "links": [["lora_serial.raw", "lora.raw"],
              ["lora.raw", "lora_serial.raw"],
              ["lora.robot_status", "cc.robot_status"],
              ["cc.cmd", "lora.cmd"]]
  }
}


####################################################################################
class DummyRobotReporter(osgar.node.Node):
    def __init__(self, cfg, bus):
        super().__init__(cfg, bus)
        bus.register('status')
        self.sleep_time = cfg['sleep']

    def run(self):
        step = 500
        x, y = 0, 0
        heading = 30
        try:
            while self.is_alive():
                x += step * math.cos(math.radians(heading))
                y += step * math.sin(math.radians(heading))
                self.publish('status', [1, [int(x), int(y), heading*100], 'running'])
                self.sleep(self.sleep_time)

        except osgar.bus.BusShutdownException:
            pass


CFG_DEMO = {
  "version": 2,
  "robot": {
    "modules": {
      "cc": {
          "driver": "subt.control_center_qt:OsgarControlCenter",
          "init": {}
      },
      "robot1": {
          "driver": "subt.control_center_qt:DummyRobotReporter",
          "init": {
              "sleep": 0.5
          }
      }
    },
    "links": [["robot1.status", "cc.robot_status"]]
  }
}
####################################################################################


class OsgarControlCenter:

    def __init__(self, init, bus):
        self.bus = bus
        bus.register('cmd')
        self.view = None # will be set from outside by record
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        self.thread.join()

    def run(self):
        while True:
            try:
                dt, channel, data = self.bus.listen()
            except osgar.bus.BusShutdownException:
                return
            if channel == "robot_status":
                robot, pose2d, status = data
                self.view.robot_status.emit(robot, pose2d, status)

    def pause_mission(self):
        self.bus.publish('cmd', [0, b'Pause'])

    def continue_mission(self):
        self.bus.publish('cmd', [0, b'Continue'])

    def stop_mission(self):
        self.bus.publish('cmd', [0, b'Stop'])


class MainWindow(QMainWindow):

    def __init__(self, arguments):
        super().__init__()
        self.setWindowTitle("Control Center")
        #self.setWindowIcon(QIcon('something.pnp'))
        self._createMenu()
        self._createStatusBar()
        self._createDock()
        self.setCentralWidget(View())
        self.recorder = None
        self.cc = None
        self.centralWidget().show_message.connect(self.status.showMessage)
        if "--demo" not in arguments:
            self.cfg = CFG_LORA
            self.status.showMessage("Using LoRa cfg")
        else:
            self.cfg = CFG_DEMO
            self.status.showMessage("Using demo cfg")

    def _createMenu(self):
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("&File")
        exit_action = QAction("Exit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.setStatusTip("Exit application")
        exit_action.triggered.connect(self.close)
        exit_action.setIcon(QIcon.fromTheme("application-exit"));
        self.file_menu.addAction(exit_action)

        self.start_stop = QCheckBox("&Recording", self)
        self.start_stop.stateChanged.connect(self.on_start_stop)
        self.menu.setCornerWidget(self.start_stop, Qt.TopRightCorner)

    def _createStatusBar(self):
        self.status = self.statusBar()

    def _createDock(self):
        d = QDockWidget("All Robots", self)
        w = QWidget()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)
        layout.addWidget(QPushButton("Pause Mission", clicked=self.on_pause_mission))
        layout.addWidget(QPushButton("Continue Mission", clicked=self.on_continue_mission))
        layout.addWidget(QPushButton("Exit Mission", clicked=self.on_stop_mission))
        w.setLayout(layout)
        d.setWidget(w)
        d.setDisabled(True)
        self.addDockWidget(Qt.RightDockWidgetArea, d)

    def on_start_stop(self, state):
        if self.recorder is None:
            assert state == Qt.Checked
            self.recorder = record(self.centralWidget(), self.cfg)
            self.cc = next(self.recorder)
            print("recording started")
        else:
            assert state == Qt.Unchecked
            with suppress(StopIteration):
                next(self.recorder)
            self.recorder = None
            print("recording stopped")
        for d in self.findChildren(QDockWidget):
            d.setDisabled(self.recorder is None)

    def closeEvent(self, e):
        self.centralWidget().close()

    def on_pause_mission(self):
        print("pause mission")
        self.cc.pause_mission()

    def on_continue_mission(self):
        print('continue mission')
        self.cc.continue_mission()

    def on_stop_mission(self):
        print("stop mission")
        self.cc.stop_mission()


def record(view, cfg):
    with osgar.logger.LogWriter(prefix='control-center-') as log:
        log.write(0, bytes(str(cfg), 'ascii'))
        recorder = osgar.record.Recorder(config=cfg['robot'], logger=log)
        cc = recorder.modules['cc']
        cc.view = view
        view.robot_statuses = {}
        with recorder:
            yield cc


class View(QWidget):

    robot_status = pyqtSignal(int, list, str)
    show_message = pyqtSignal(str, int)
    colors = [QColor(Qt.white), QColor(Qt.green), QColor(Qt.blue), QColor(Qt.red),
              QColor(Qt.cyan), QColor(Qt.magenta), QColor(Qt.yellow)]

    def __init__(self):
        super().__init__()
        self.robot_status.connect(self.on_robot_status)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)
        self.robot_statuses = {}

    def on_robot_status(self, robot, pose2d, status):
        self.show_message.emit(f"received position {pose2d} and status {status} from robot {robot}", 3000)
        self.robot_statuses.setdefault(robot, []).append((pose2d, status))
        self.update()

    def paintEvent(self, e):
        t = QTransform() # world transform
        t.translate(self.width() / 2, self.height() / 2) # in pixels
        t.scale(0.1, -0.1) # 1px == 1cm (position is in mm)

        t2 = QTransform() # just orientation
        t2.scale(1, -1)

        with QPainter(self) as qp:
            for robot, statuses in self.robot_statuses.items():
                base_color = self.colors[robot]
                qp.setPen(base_color)
                path = QPolygonF()
                for (x, y, heading), status in statuses:
                    robot_position = t.map(QPointF(x, y))
                    path.append(robot_position)
                    qp.drawEllipse(robot_position, 3, 3)
                    #heading = math.radians(heading/100.0)
                    #p2 = QTransform(t2).rotateRadians(heading).map(QPointF(10,0))
                    #qp.drawLine(p1, p1+p2)
                qp.setPen(base_color.darker())
                qp.drawPolyline(path)
            qp.setPen(Qt.white)

    def sizeHint(self):
        return QSize(800, 600)


def main():
    app = QApplication(sys.argv)

    window = MainWindow(app.arguments())
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

# vim: expandtab sw=4 ts=4
