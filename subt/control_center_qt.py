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
                              QDockWidget, QComboBox, QMenu)

import osgar.record
import osgar.replay
import osgar.logger
import osgar.bus
import osgar.node
from subt.artifacts import BACKPACK, RESCUE_RANDY, PHONE, VENT, GAS


DEFAULT_ARTF_Z_COORD = 1.0  # when reporting artifacts manually from View

g_filename = None  # filename for replay log

####################################################################################
g_artf_colors = {
        BACKPACK : Qt.red,
        RESCUE_RANDY : Qt.yellow,
        PHONE : Qt.blue,
        VENT : Qt.white,
        GAS : Qt.darkYellow
    }
ARTF_TYPES = g_artf_colors.keys()

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
              ["lora.artf", "cc.artf"],
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

    def join(self, timeout=None):
        self.thread.join(timeout)

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
        self.robot_id = 0  # ALL by default

    def start(self):
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def request_stop(self):
        self.bus.shutdown()

    def join(self, timeout=None):
        self.thread.join(timeout)

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
            elif channel == "artf":
                robot_id, (artf, x, y, z) = data
                print(dt, robot_id, artf, (x/1000, y/1000, z/1000))
                self.view.artf_xyz.emit(artf, (x/1000, y/1000, z/1000))

    def pause_mission(self):
        self.bus.publish('cmd', [self.robot_id, b'Pause'])

    def continue_mission(self):
        self.bus.publish('cmd', [self.robot_id, b'Continue'])

    def stop_mission(self):
        self.bus.publish('cmd', [self.robot_id, b'Stop'])

    def go_home(self):
        self.bus.publish('cmd', [self.robot_id, b'GoHome'])

    def set_robot_id(self, i):
        self.robot_id = i


class MainWindow(QMainWindow):

    zoom_reset = pyqtSignal()
    zoom_in = pyqtSignal()
    zoom_out = pyqtSignal()

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
        self.zoom_reset.connect(self.centralWidget().on_zoom_reset)
        self.zoom_in.connect(self.centralWidget().on_zoom_in)
        self.zoom_out.connect(self.centralWidget().on_zoom_out)
        if "--replay" in arguments:
            self.cfg = None
            print(arguments)
            global g_filename
            g_filename = arguments[-1]
        elif "--demo" not in arguments:
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

        self.view_menu = self.menu.addMenu("&View")

        zoomin_action = QAction("Zoom in", self)
        # allow multiple shortcuts
        # see https://www.qtcentre.org/threads/1616-Multiple-shortuts-for-the-action
        zoomin_action2 = QAction(zoomin_action)
        zoomin_action.setShortcut(Qt.Key_Plus)
        zoomin_action2.setShortcut(Qt.Key_Equal)
        zoomin_action.triggered.connect(self.zoom_in)
        zoomin_action2.triggered.connect(self.zoom_in)
        self.view_menu.addAction(zoomin_action)
        self.view_menu.addAction(zoomin_action2)

        zoomreset_action = QAction("Zoom reset", self)
        zoomreset_action.setShortcut(Qt.Key_0)
        zoomreset_action.triggered.connect(self.zoom_reset)
        self.view_menu.addAction(zoomreset_action)

        zoomout_action = QAction("Zoom out", self)
        zoomout_action.setShortcut(Qt.Key_Minus)
        zoomout_action.triggered.connect(self.zoom_out)
        self.view_menu.addAction(zoomout_action)

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

        self.cb = QComboBox()
        self.cb.addItem("#0 - ALL")
        self.cb.addItem("-----------------")  # 1 is reserved for the Control Center LoRa
        self.cb.addItem("#2 - Eduro")
        self.cb.addItem("#3 - Kloubak K3")
        self.cb.addItem("#4 - MOBoS")
        self.cb.addItem("#5 - Kloubak K2")
        self.cb.addItem("#6 - Maria")
        self.cb.currentIndexChanged.connect(self.selectionchange)
        layout.addWidget(self.cb)

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
            if g_filename is None:
                self.recorder = record(self.centralWidget(), self.cfg)
            else:
                self.recorder = replay(self.centralWidget(), g_filename)
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
        print("pause mission", self.cb.currentText())
        self.cc.pause_mission()

    def on_continue_mission(self):
        print('continue mission', self.cb.currentText())
        self.cc.continue_mission()

    def on_stop_mission(self):
        print("stop mission", self.cb.currentText())
        self.cc.stop_mission()

    def on_go_home(self):
        print("go home", self.cb.currentText())
        self.cc.go_home()

    def selectionchange(self, i):
        print("Current index", i, "selection changed ", self.cb.currentText())
        self.cc.set_robot_id(i)

def record(view, cfg):
    with osgar.logger.LogWriter(prefix='control-center-', note=str(sys.argv)) as log:
        log.write(0, bytes(str(cfg), 'ascii'))
        recorder = osgar.record.Recorder(config=cfg['robot'], logger=log)
        cc = recorder.modules['cc']
        cc.view = view
        view.robot_statuses = {}
        with recorder:
            yield cc


def replay(view, filename):
    args = type('MyClass', (object,), 
                {'logfile': filename, 'module': 'cc', 'config': None,
                 'debug': None, 'force': True})
    cc = osgar.replay.replay(args)
    cc.view = view
    view.robot_statuses = {}
    cc.start()
    yield cc
    cc.join()


class View(QWidget):

    robot_status = pyqtSignal(int, list, bytes)
    artf_xyz = pyqtSignal(str, tuple)
    show_message = pyqtSignal(str, int)
    colors = [QColor(Qt.white), QColor(Qt.green), QColor(Qt.blue), QColor(Qt.red),
              QColor(Qt.cyan), QColor(Qt.magenta), QColor(Qt.yellow)]

    def __init__(self):
        super().__init__()
        self.robot_status.connect(self.on_robot_status)
        self.artf_xyz.connect(self.on_artf_xyz)
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)
        self.setMouseTracking(True)
        self.setCursor(Qt.CrossCursor)
        self.translation = QPointF(0, 0) # how many px is the viewport shifted
        self.scale = 100  # 1px == 1cm (position is in meters)
        self.grab_start = None
        self.grab_diff = QPointF(0, 0)
        self.robot_statuses = {}
        self.artifacts = []
        self.reported_artifacts = []

    def on_robot_status(self, robot, pose2d, status):
        #self.show_message.emit(f"robot {robot}: pose2d {pose2d}, status {status}", 3000)
        self.robot_statuses.setdefault(robot, []).append((pose2d, status))
        self.update()

    def on_artf_xyz(self, artf_type, xyz):
        print(artf_type, xyz)
        self.artifacts.append((artf_type, xyz))

    def _transform(self):
        t = QTransform() # world transform
        t.translate(self.width() / 2, self.height() / 2) # in pixels
        t.translate(self.translation.x(), self.translation.y())
        t.translate(self.grab_diff.x(), self.grab_diff.y())
        t.scale(self.scale, -self.scale)
        return t

    def paintEvent(self, e):
        transform = self._transform()
        with QPainter(self) as painter:
            self._drawGrid(painter, transform)
            self._drawCenterCross(painter, transform)
            self._drawScale(painter, transform)
            self._drawArtf(painter, transform)
            self._drawReportedArtf(painter, transform)
            for robot, statuses in self.robot_statuses.items():
                base_color = self.colors[robot%len(self.colors)]
                self._drawRobotPath(painter, transform, base_color, statuses)

    def _drawGrid(self, painter, transform):
        painter.setPen(QColor(Qt.white).darker().darker())
        meter = QLineF(transform.map(QPointF(1, 0)), transform.map(QPointF(0, 0))).length()
        limits = [(1.5, 50), (4, 20), (10, 10), (35, 5), (100000, 1)]
        unit = [unit for px,unit in limits if meter < px][0]
        t, ok = transform.inverted()
        r = self.rect()
        x, y = [], []
        for point in [(r.left(),r.top()), (r.left(), r.bottom()), (r.right(), r.top()), (r.right(), r.bottom())]:
            p = t.map(QPointF(*point))
            x.append(p.x())
            y.append(p.y())
        minx, maxx = min(x), max(x)
        miny, maxy = min(y), max(y)
        for i in range(math.ceil(minx/unit), math.ceil(maxx/unit)):
            painter.drawLine(transform.map(QPointF(i * unit, maxy)), transform.map(QPointF(i * unit, miny)))
        for j in range(math.ceil(miny/unit), math.ceil(maxy/unit)):
            painter.drawLine(transform.map(QPointF(maxx, j * unit)), transform.map(QPointF(minx, j * unit)))

    def _drawRobotPath(self, painter, transform, base_color, statuses):
        # just orientation
        t2 = QTransform(math.copysign(1, transform.m11()), transform.m12(),
                        transform.m21(), math.copysign(1, transform.m22()),
                        0, 0)

        painter.setPen(base_color)
        path = QPolygonF()
        for (x, y, heading), status in statuses:
            robot_position = transform.map(QPointF(x, y))
            path.append(robot_position)
            painter.drawEllipse(robot_position, 3, 3)
        # draw direction vector of last position
        facing = QTransform(t2).rotateRadians(heading).map(QPointF(10, 0))
        painter.drawLine(robot_position, robot_position + facing)
        # highlight last position
        painter.setBrush(base_color)
        painter.drawEllipse(robot_position, 4, 4)
        painter.setBrush(Qt.NoBrush)
        # draw path
        painter.setPen(base_color.darker())
        painter.drawPolyline(path)

    def _drawCenterCross(self, painter, transform):
        painter.setPen(Qt.red)
        center = transform.map(QPointF(0, 0))
        painter.drawLine(center - QPointF(10, 0), center + QPointF(10, 0))
        painter.drawLine(center - QPointF(0, 10), center + QPointF(0, 10))

    def _drawScale(self, painter, transform):
        painter.setPen(Qt.red)
        meter = QLineF(transform.map(QPointF(1, 0)), transform.map(QPointF(0, 0))).length()
        painter.drawLine(20, self.height() - 20, 20 + meter, self.height() - 20)
        painter.drawLine(20, self.height() - 25, 20, self.height() - 15)
        painter.drawLine(20 + meter, self.height() - 25, 20 + meter, self.height() - 15)

    def _drawArtf(self, painter, transform):
        for name, xyz in self.artifacts:
            if name in g_artf_colors:
                painter.setPen(g_artf_colors[name])
            else:
                painter.setPen(Qt.gray)
            x, y, z = xyz
            pt = transform.map(QPointF(x, y))
            painter.drawEllipse(pt, 10, 10)

    def _drawReportedArtf(self, painter, transform):
        painter.setPen(Qt.red)
        for name, xyz in self.reported_artifacts:
            x, y, z = xyz
            pt = transform.map(QPointF(x, y))
            radius = transform.map(QPointF(5, 0)).x() - transform.map(QPointF(0, 0)).x()
            painter.setPen(g_artf_colors[name])
            painter.drawEllipse(pt, radius, radius)

    def sizeHint(self):
        return QSize(800, 600)

    def mousePressEvent(self, e):
        if e.buttons() == Qt.LeftButton:
            self.setCursor(Qt.ClosedHandCursor)
            self.grab_start = e.globalPos()
        elif e.buttons() == Qt.RightButton:
            contextMenu = QMenu(self)
            listAct = [contextMenu.addAction(a) for a in ARTF_TYPES]
            action = contextMenu.exec_(self.mapToGlobal(e.pos()))
            if action in listAct:
                t = self._transform()
                t, ok = t.inverted()
                pt = t.map(QPointF(e.pos()))  # we need float resolution
                self.reported_artifacts.append((action.text(), (pt.x(), pt.y(), DEFAULT_ARTF_Z_COORD)))
                self.update()

    def mouseReleaseEvent(self, e):
        cursor = self.cursor().shape()
        if cursor == Qt.ClosedHandCursor:
            self.grab_start = None
            self.setCursor(Qt.CrossCursor)
            self.translation += self.grab_diff
            self.grab_diff = QPointF(0, 0)

    def mouseMoveEvent(self, e):
        t = self._transform()
        t, ok = t.inverted()
        p = t.map(e.localPos())

        # TODO: show in tooltip instead?
        self.show_message.emit(f"[{p.x(): .2f}, {p.y(): .2f}]", 0)

        if self.grab_start is not None:
            self.grab_diff = e.globalPos() - self.grab_start
            self.update()

    def on_zoom_reset(self):
        self.scale = 100
        self.update()

    def on_zoom_in(self):
        self.scale *= 1.1
        self.update()

    def on_zoom_out(self):
        self.scale /= 1.1
        self.update()


def main():
    app = QApplication(sys.argv)

    window = MainWindow(app.arguments())
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

# vim: expandtab sw=4 ts=4
