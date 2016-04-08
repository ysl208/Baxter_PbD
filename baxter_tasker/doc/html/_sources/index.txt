.. baxter_tasker documentation master file, created by
   sphinx-quickstart on Wed Apr 16 10:25:00 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to baxter_tasker's documentation!
=========================================

This ROS package called baxter tasker is intended for the Baxter Research
Robot from Rethink Robotics. It offers an interface on the controlling 
computer and on the robot. In particular this interface is designed to help
users to setup a simple pick and place behavior, and automatically generate the corresponding Python code.
Simple behaviors can
be directly learned on the robot, whereas more complex behaviors have to be
programmed in Python. 
See the install instructions in INSTALL.txt or the setup_ section in the
documentation to get 3D environment running, in order to use all capabilities 
of baxter tasker.



What is it for?
---------------
This software it suited for Python developers who are new to baxter,
even if you already have experience in Python. There are a lot of tools
to show how the stuff and access to the robot works. 
In this term the interface allows a quick start into the subject of the
Baxter Research Robot.

What does it do?
----------------
This software gives a general overview of baxter's capabilities. There a lot
of useful classes in Python that show the usage of the sensor data and 
how to control the robot. The basic interface uses 

* the display,
* the buttons,
* the cameras,
* the arms,
* the infrared distance sensors,
* the leds and
* the sonar sensors.  

There are also procedures
to save trajectories and information about the environment. For advanced programmers,
there are two classes where new functions could be added, which are automatically added
to the graphical user interface. The classes offer an easy access to all of the 
robot's hardware.

What was used?
--------------
The Baxter Tasker is made for Python 2.7  with ROS Groovy that uses the catkin builder.
The current baxter-sdk is 0.7.0. There is also an optional but recommended package
called MoveIt, which was used to create a virtual scene around baxter. In the virtual 
scene MoveIt! allows collision free planning. 

Dependencies - Compatibility - Tested Versions
----------------------------------------------
The Baxter Tasker was tested and works with
* Python 2.7
* Groovy, Hydro (with MoveIt fix)
* Baxter RSDK Version 0.7.0 and 1.0.0
* MoveIt
* Ubuntu 12.04, 12.10, 13.04
There is a good chance that it is also working with different ROS and Ubuntu versions.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

.. _setup:
.. include:: ../INSTALL.txt

.. include:: quickstart.txt
   
Interface Description
=====================

.. include:: interface_robot.txt

.. include:: interface_workstation.txt
   
API Reference
=============

Contents:

.. toctree::
   :maxdepth: 2
   
   modules

.. include:: environment_config.txt

.. include:: ../CHANGELOG.txt

.. include:: ../TODO.txt

 

