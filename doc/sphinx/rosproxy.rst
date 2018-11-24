ROS Proxy
=========

The node `rosproxy` is an interface to `ROS <http://wiki.ros.org/>`_ (Robot
Operating System). The implementation is based on `experiments with robot Husky
<https://robotika.cz/robots/husky/en#140820>`_ back in 2014. The technical info
is taken from `ROS Technical Overview
<http://wiki.ros.org/ROS/Technical%20Overview>`_. `Hello World
<https://robohub.org/ros-101-intro-to-the-robot-operating-system/>`_ and `ROS
Practical Example <https://robohub.org/ros-101-a-practical-example/>`_ are good
starting points to get familiar with the basic ROS concept.

Note, that OSGAR ROS Proxy implementation does not require compilation and
specific operating system (typically Ubuntu linux). The price is then limited
set of available messages (compatible with OSGAR), but they can be extended in
the future.

OSGAR ROS Proxy is a node both in OSGAR as well as ROS sense. It connects to
ROS master, provides XMLRPC server for topic registration and finally it
communicates over TCP in order to send and receive messages. In some sense it
is actually bridge between these two worlds.

Every OSGAR ROS Proxy node configuration needs:
  - ROS Master URI
  - ROS Client URI, i.e. configuration for XMLRPC server
  - topic and if it should be published or subscribed

