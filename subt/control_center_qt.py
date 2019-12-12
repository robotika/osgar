"""
    Control Center for SubT Urban
"""

import threading
import math
import random
import sys
import platform
from datetime import timedelta
from contextlib import suppress

from PyQt5.QtCore import pyqtSignal, QSize, Qt, QPointF, QLineF
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
class DummyRobot:
    def __init__(self, cfg, bus):
        self.bus = bus
        bus.register('status')
        self.speed = cfg.get('speed', 0.5) # m/s
        self.random = random.Random(cfg.get('seed', 1))
        self.x, self.y = cfg.get('x', 0), cfg.get('y', 0) # m
        self.heading = math.radians(cfg.get('heading', 30)) # degrees
        self.id = cfg.get('id', 1)
        self.thread = threading.Thread(target=self.run)

    def start(self):
        self.thread.start()

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        self.thread.join()

    def run(self):
        x, y = self.x, self.y
        heading = self.heading
        last_dt = timedelta()
        current_speed = self.speed
        state = b'Continue'
        try:
            while True:
                dt, channel, data = self.bus.listen()
                #print(dt, channel, data, self.id)
                if channel == 'cmd':
                    robot_id, cmd = data
                    if robot_id == 0 or robot_id == self.id:
                        if cmd == b'Stop':
                            return
                        if cmd == b'GoHome':
                            print(dt, self.id, "GoHome not implemented")
                        state = cmd
                elif channel == 'move':
                    if state == b'Continue':
                        step = (dt - last_dt).total_seconds() * current_speed
                        x += step * math.cos(heading)
                        y += step * math.sin(heading)
                        if self.random.random() > 0.3:
                            heading += math.radians(10)
                    last_dt = dt
                    pose2d = [round(x * 1000), round(y * 1000), round(math.degrees(heading) * 100)]
                    self.bus.publish('status', [self.id, pose2d, state])
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
      "timer": {
          "driver": "timer",
          "init": {
              "sleep": 0.5,
          }
      },
      "robot1": {
          "driver": "subt.control_center_qt:DummyRobot",
          "init": {
              "id": 1,
              "heading": 0*30,
              "x": 1,
              "y": 0,
          }
      },
      "robot2": {
          "driver": "subt.control_center_qt:DummyRobot",
          "init": {
              "id": 2,
              "heading": 1*30,
              "x": 1,
              "y": 1,
          }
      },
      "robot3": {
          "driver": "subt.control_center_qt:DummyRobot",
          "init": {
              "id": 3,
              "heading": 2*30,
              "x": 0,
              "y": 1,
          }
      },
      "robot4": {
          "driver": "subt.control_center_qt:DummyRobot",
          "init": {
              "id": 4,
              "heading": 3*30,
              "x": -1,
              "y": 1,
          }
      }
    },
    "links": [
        ["timer.tick", "robot1.move"],
        ["timer.tick", "robot2.move"],
        ["timer.tick", "robot3.move"],
        ["timer.tick", "robot4.move"],
        ["robot1.status", "cc.robot_status"],
        ["robot2.status", "cc.robot_status"],
        ["robot3.status", "cc.robot_status"],
        ["robot4.status", "cc.robot_status"],
        ["cc.cmd", "robot1.cmd"],
        ["cc.cmd", "robot2.cmd"],
        ["cc.cmd", "robot3.cmd"],
        ["cc.cmd", "robot4.cmd"],
    ]
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
                robot_id, (x, y, heading), status = data
                pose2d = [x/1000, y/1000, math.radians(heading/100)]
                self.view.robot_status.emit(robot_id, pose2d, status)

    def pause_mission(self):
        self.bus.publish('cmd', [0, b'Pause'])

    def continue_mission(self):
        self.bus.publish('cmd', [0, b'Continue'])

    def stop_mission(self):
        self.bus.publish('cmd', [0, b'Stop'])

    def go_home(self):
        self.bus.publish('cmd', [0, b'GoHome'])


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
        layout.addWidget(QPushButton("&Pause Mission", clicked=self.on_pause_mission))
        layout.addWidget(QPushButton("&Continue Mission", clicked=self.on_continue_mission))
        layout.addWidget(QPushButton("&Stop Mission", clicked=self.on_stop_mission))
        layout.addWidget(QPushButton("&Go Home", clicked=self.on_go_home))
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

    def on_go_home(self):
        print("go home")
        self.cc.go_home()


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

    robot_status = pyqtSignal(int, list, bytes)
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
        self.setMouseTracking(True)
        self.robot_statuses = {}

    def on_robot_status(self, robot, pose2d, status):
        #self.show_message.emit(f"robot {robot}: pose2d {pose2d}, status {status}", 3000)
        self.robot_statuses.setdefault(robot, []).append((pose2d, status))
        self.update()

    def paintEvent(self, e):
        t = QTransform() # world transform
        t.translate(self.width() / 2, self.height() / 2) # in pixels
        t.scale(100, -100) # 1px == 1cm (position is in meters)

        t2 = QTransform() # just orientation
        t2.scale(1, -1)

        with QPainter(self) as qp:
            # draw center cross
            qp.setPen(Qt.red)
            center = t.map(QPointF(0,0))
            qp.drawLine(center-QPointF(10,0), center+QPointF(10,0))
            qp.drawLine(center-QPointF(0,10), center+QPointF(0,10))
            # draw scale
            meter = QLineF(t.map(QPointF(1,0)), t.map(QPointF(0,0))).length()
            qp.drawLine(20, self.height()-20, 20 + meter, self.height() - 20)
            qp.drawLine(20, self.height()-25, 20, self.height()-15)
            qp.drawLine(20 + meter, self.height()-25, 20 + meter, self.height()-15)
            # draw robot paths
            for robot, statuses in self.robot_statuses.items():
                base_color = self.colors[robot%len(self.colors)]
                qp.setPen(base_color)
                path = QPolygonF()
                for (x, y, heading), status in statuses:
                    robot_position = t.map(QPointF(x, y))
                    path.append(robot_position)
                    qp.drawEllipse(robot_position, 3, 3)
                # draw direction vector of last position
                facing = QTransform(t2).rotateRadians(heading).map(QPointF(10,0))
                qp.drawLine(robot_position, robot_position+facing)
                # highlight last position
                qp.setBrush(base_color)
                qp.drawEllipse(robot_position, 4, 4)
                qp.setBrush(Qt.NoBrush)
                # draw path
                qp.setPen(base_color.darker())
                qp.drawPolyline(path)
            qp.setPen(Qt.white)

    def sizeHint(self):
        return QSize(800, 600)

    def mouseMoveEvent(self, e):
        t = QTransform()
        t.translate(self.width() / 2, self.height() / 2)
        t.scale(100, -100)
        t, ok = t.inverted()
        p = t.map(e.localPos())

        self.show_message.emit(f"[{p.x(): .2f}, {p.y(): .2f}]", 0)
        #print('-'*20)
        # for dragging
        #print(e.globalPos())
        #print(e.screenPos())


def main():
    app = QApplication(sys.argv)

    window = MainWindow(app.arguments())
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

# vim: expandtab sw=4 ts=4
