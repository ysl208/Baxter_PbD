.. 	__________                   .___              
	\______   \ ____ _____     __| _/_____   ____  
	|       _// __ \\__  \   / __ |/     \_/ __ \ 
	|    |   \  ___/ / __ \_/ /_/ |  Y Y  \  ___/ 
	|____|_  /\___  >____  /\____ |__|_|  /\___  >
	       \/     \/     \/      \/     \/     \/ 
	2014-04-11
	
TO BUILD the Documentation run in this directory: ::

    $rosdoc_lite .

This ROS package called baxter tasker is intended for the Baxter Research Robot from Rethink Robotics.It offers an interface on the controlling computer and on the robot. In particular this interface is designed to help the user, to setup a simple pick and place behavior. Simple behaviors can directly learned on the robot, whereas more complex behaviors have to be programmed in Python. 

See the install instruction in INSTALL.txt to get 3D environment running, in order to use all capabilities of baxter tasker

Optional:
=========
	Either this:
	------------
	If you have installed the 3d environment, called MoveIt, then start this one first, after you executed your baxter script:
		$ roslaunch baxter_tasker moveit_baxter.launch
	This command starts MoveIt, RViz and the trajectory action server for the robot. The action server handles all trajectory requests to execute
	a move, evading all obstacles in your 3D scene.

	Or this:
	--------	
	In case you have limited resources you could instead start everything without RViz by:
		$ roslaunch baxter_tasker moveit_baxter_norviz.launch
	
Required:
=========
Now is the time to start baxter tasker:
	$ roslaunch baxter_tasker baxter_tasker_main.launch
This starts the interface for baxter tasker. If you are starting this interface regularly, you should add an alias to your .bashrc.
To do this simply copy this into your terminal:	
	$ echo 'alias bt="roslaunch baxter_tasker baxter_tasker_main.launch"' >> ~/.bashrc
After a restart of your terminal you can also run the ros node by:
	$ bt
	
The Interface(remote computer):
=====================================
The interface on the computer consists of two columns of buttons on the left side, a list of tasks in the middle and an editor on the right. The buttons offer some functionality to trigger specific functions, which are described in detail below. 
Not all of them are available on the robot. In fact you are working with a collaborative robot. 
The idea behind is that you can create a new task, learn with the robot, which generates Python code in the editor field.
In this interface you have the possibility to modify parameters of this code and also access nearly the complete interface in the background.
You can simply save your task by clicking on File (in the top row) and click on "Save all Tasks". Once you generated or wrote some code, you select your task from the list and click on the execute button. This gives you the power to test stuff very quickly. 

	Functionality of the buttons:
	-----------------------------
	Add Task: Creates a new task in the list. Use "File->Save Tasks" to keep your task
	Execute Task: Executes the selected tasks of the list.
	Enable/Disable Robot: Enables or disables the motors
	
	Back Button: Goes one level higher in menu
	Confirm Right Button: Confirms a selection with side right
	Confirm Left Button: Confirms a selection with side left
	Turn Wheel: Turns the left wheel to the right (negative mathematic sense)
	Round Cuff Right Button: Does nothing
	Round Cuff Left Button: Does nothing
	Oval Cuff Right Button: Opens/Closes the right gripper
	Oval Cuff Left Button: Opens/Closes the left gripper
	Set HR Logo: Sets the Humarobotics logo to the screen of the robot
	Create MoveIt URDF: Calibrates all joints and then updates the URDF model of the robot
	
	STOP: Set a movement flag to stop new movements.
	Continue: Unsets the movement flag
	Go to Init: Moves the arms to a start position
	Generate Plans: Generate plans for the current scenario and scene
	
	Test Switch 1: Picks a switch item with the right arm
	Test Switch 2: Drops a switch item with the left arm
	Test Cover Small: Picks and drops a small cover with the right arm
	Test Cover Door: Picks and drops a door cover with the left arm
	Test Tray: Picks and drops a tray with the left arm
	Test Goblet Blue: Picks and drops a blue goblet with the left arm
	Test Goblet Red: Picks and drops a red goblet with the left arm

The Interface (on the robot):
=============================
The interface on the robot offers some basic functions. You can control the interface by the buttons on the robot. Although most of the functionality is identical for both sides of the robot, the button side is important when it comes to a specific arm motion.

Robot Buttons:
--------------
	Hand Cuff:
		Cuff button: Frees the arm to be able to move it around
		Oval button: Opens/Closes gripper
	Navigator Elbow:
		Rethink button: Goes to home menu
		Arrow button: Goes one level higher in menu
		Wheel button: Confirms a selection
		Wheel: Selects previous or next option on screen
	Navigator Torso:
		Wheel button: Unsets the stop flag (same as the continue button on computer)
		Arrow button: Sets the stop flag: Robot stops after its trajectory has been finished
	Shoulder buttons:
		Enables or disables the motors of the robot
		
Menu:
-----
	Go to Init: Moves both arms to a starting/neutral position in front of the robot:
	Run Scenarios: Lists all scenarios that are implemented in the script "baxter_scenarios.py". 
		One scenario "Pick and Place" is interactive. This mean that you can directly teach the robot what to do. 
		In this scenario you can save multiple trajectory for each arm. In the interface you see a counter which is the number of 
		trajectory to be saved. With the "next" and "previous" option you can choose the number where the trajectory should be saved.
		With "learn" you can save a trajectory to a step. By hitting "Run Step" you execute your current trajectory. "Run All" runs all
		saved trajectory from step 0 upwards until there is no trajectory any more. The trajectories are saved in the data folder of the 
		package. 
	Select arms: Some scenarios e.g. consignment2 have the possibility to only use one arm. In case you want to test something or
		you dont have enough space around the robot
	Show camera: Depending on the side you press the wheel confirmation button, the according camera in the hand is being displayed on the
		screen. By any other action afterwards on the Navigator Elbow the camera stream is stopped.
	Teach Menu: Here you can show the robot one trajectory on each arm that can be executed afterwards. In addition you have the 
		possibility to stop the execution at all times from both arms and the computer interface.
	Manage Boxes (requires MoveIt):
		This one sets the origin and inclination of a box relative to gripper endpoint. "Point0" is a top border corner of a box, while
		"Point1" lies on the x-axis of Point0. "Select Box" selects the predefined boxes. Boxes are defined by their length,
		width, height, thickness, number of objects, dimension of the object, pose and offset of the objects or possible drop off points 
		for objects. All predefined boxes are defined in scripts/baxter_scene.py (Going to be moved into a separate config file in the
		next version). 
	Select Gripper (requires MoveIt):  Depending if you are using the electric or suction gripper you can adjust the length of the gripper.
		This is important for the collision detection in your 3D model.
		If you have a custom gripper, you need to change the code in scripts/baxter_scene.py -> attachGripper






	

