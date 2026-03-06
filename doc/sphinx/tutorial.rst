MyRobot Tutorial
================

This tutorial explains how to create a simple simulated robot and a control
application using OSGAR. We will use the ``myrobot`` example as a reference.

The ``myrobot`` example consists of three main files:

- ``myrobot.py``: The robot driver, which simulates the robot's behavior.
- ``myapp.py``: The application, which controls the robot.
- ``myrobot.json``: The configuration file, which connects the robot and the
  application.

The Robot Driver - ``myrobot.py``
---------------------------------

The ``myrobot.py`` file defines a simple simulated robot with differential
drive. It receives speed commands and simulates the robot's movement,
publishing its new pose (position and orientation).

Here is the code for ``myrobot.py``:

.. code-block:: python

  """
  Example of robot diver outside OSGAR package
    - simulation of differential robot
  """
  import math

  from osgar.node import Node
  from osgar.bus import BusShutdownException


  class MyTimer(Node):
      def __init__(self, config, bus):
          super().__init__(config, bus)
          bus.register('tick')
          self.sleep_time = config['sleep']

      def run(self):
          try:
              while self.is_bus_alive():
                  self.publish('tick', None)
                  self.sleep(self.sleep_time)

          except BusShutdownException:
              pass


  class MyRobot(Node):
      def __init__(self, config, bus):
          super().__init__(config, bus)
          bus.register('pose2d')
          self.pose = (0, 0, 0)
          self.speed, self.angular_speed = 0, 0
          self.desired_speed, self.desired_angular_speed = 0, 0
          self.last_update = None

      def send_pose(self):
          x, y, heading = self.pose
          self.publish('pose2d', [round(x*1000), round(y*1000),
                                  round(math.degrees(heading)*100)])

      def update_pose(self, diff_time):
          self.speed = self.desired_speed  # TODO motion model
          self.angular_speed = self.desired_angular_speed
          t = diff_time

          x, y, heading = self.pose
          x += math.cos(heading) * self.speed * t
          y += math.sin(heading) * self.speed * t
          heading += self.angular_speed * t
          self.pose = (x, y, heading)

      def slot_desired_speed(self, data):
          self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)

      def update(self):
          dt, channel, data = self.listen()
          if self.last_update is not None:
              t = (dt - self.last_update).total_seconds()
              self.update_pose(t)
          self.last_update = dt

          if channel == 'desired_speed':
              self.slot_desired_speed(data)

          self.send_pose()

  # vim: expandtab sw=4 ts=4


The Application - ``myapp.py``
------------------------------

The ``myapp.py`` file defines an application that controls the robot. It sends
speed commands to make the robot move in a square.

Here is the code for ``myapp.py``:

.. code-block:: python

  """
  Example of a simple application controling robot to move in figure 8
  """
  import math
  from datetime import timedelta

  from osgar.node import Node


  def distance(pose1, pose2):
      return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


  class MyApp(Node):
      def __init__(self, config, bus):
          super().__init__(config, bus)
          bus.register('desired_speed')
          self.max_speed = config.get('max_speed', 0.1)
          self.max_angular_speed = math.radians(50)  # TODO config
          self.verbose = False
          self.last_position = (0, 0, 0)
          self.is_moving = False
          self.pose2d = None  # TODO should be defined by super().__init__()

      # TODO refactor to common "serializer"
      def send_speed_cmd(self, speed, angular_speed):
          return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

      # TODO refactor to common driver (used from sick2018 example)
      def go_straight(self, how_far):
          print(self.time, "go_straight %.1f" % how_far, self.last_position)
          start_pose = self.last_position
          if how_far >= 0:
              self.send_speed_cmd(self.max_speed, 0.0)
          else:
              self.send_speed_cmd(-self.max_speed, 0.0)
          while distance(start_pose, self.last_position) < abs(how_far):
              self.update()
          self.send_speed_cmd(0.0, 0.0)

      def turn(self, angle, with_stop=True):
          print(self.time, "turn %.1f" % math.degrees(angle))
          start_pose = self.last_position
          if angle >= 0:
              self.send_speed_cmd(0.0, self.max_angular_speed)
          else:
              self.send_speed_cmd(0.0, -self.max_angular_speed)
          while abs(start_pose[2] - self.last_position[2]) < abs(angle):
              self.update()
          if with_stop:
              self.send_speed_cmd(0.0, 0.0)
              start_time = self.time
              while self.time - start_time < timedelta(seconds=2):
                  self.update()
                  if not self.is_moving:
                      break
              print(self.time, 'stop at', self.time - start_time)

      def on_pose2d(self, data):
          prev = self.pose2d
          self.pose2d = data
          x_mm, y_mm, heading_mdeg = self.pose2d
          self.last_position = (x_mm/1000.0, y_mm/1000.0,
                                math.radians(heading_mdeg/100.0))
          self.is_moving = (prev != self.pose2d)

      def run(self):
          print("MyApp example - figure 8!")
          step_size = 0.5  # meters
          deg90 = math.radians(90)

          for i in range(4):
              self.go_straight(step_size)
              self.turn(deg90)

          for i in range(4):
              self.go_straight(step_size)
              self.turn(-deg90)

          print("END")


  if __name__ == "__main__":
      from osgar.launcher import launch

      launch(app=MyApp, description='Navigate figure eight', prefix='myapp-')

  # vim: expandtab sw=4 ts=4

The Configuration - ``myrobot.json``
------------------------------------

The ``myrobot.json`` file defines the modules (the robot and the application)
and the connections between them.

Here is the content of ``myrobot.json``:

.. code-block:: json

  {
    "version": 2,
    "robot": {
      "modules": {
        "app": {
            "driver": "application",
            "in": ["emergency_stop", "pose2d"],
            "out": ["desired_speed"],
            "init": {
              "max_speed": 0.5
            }
        },
        "myrobot": {
            "driver": "myrobot:MyRobot",
            "in": ["desired_speed"],
            "out": ["emergency_stop", "pose2d"],
            "init": {}
        },
        "timer": {
            "driver": "myrobot:MyTimer",
            "in": [],
            "out": ["tick"],
            "init": {
              "sleep": 0.1
            }
        }
      },
      "links": [["app.desired_speed", "myrobot.desired_speed"],
                ["myrobot.emergency_stop", "app.emergency_stop"],
                ["myrobot.pose2d", "app.pose2d"],
                ["timer.tick", "myrobot.tick"]]
    }
  }

Running the Example
-------------------

To run the example, you need to execute the ``osgar.record`` module with the
``myrobot.json`` configuration file. You also need to add the ``examples``
directory to your ``PYTHONPATH`` so that OSGAR can find the ``myrobot`` module.

.. code-block:: bash

  PYTHONPATH=examples python -m osgar.record examples/myrobot/myrobot.json

This will create a log file with the recorded data from the simulation.
