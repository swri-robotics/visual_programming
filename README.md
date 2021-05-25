# 10.R8803
Visual Robot Programming Application

### Issues
- Tracking is currently disabled since the aruco library requires building OpenCV3.4 from source to build in noetic. See [issue](https://github.com/pal-robotics/aruco_ros/issues/89)

### Setup
The following software is needed to build and run the application:
- wstool
- rosdep
- pip3

#### Real Hardware Only
##### Install the Intel Realsense SDK 2.0
- Follow the instructions for [installing the Realsense SDK only](https://github.com/intel-ros/realsense)
- Install OpenCV3

#### Catkin Workspace Setup
- Use `wstool` to download all the required repositories
	```
	wstool init src src/visual_programming/.rosinstall
	```
- Use `rosdep` from the workspace top level directory to install all required debians
	```
	rosdep install --from-path src --ignore-src
	```
- Install python dependencies, from the workspace top level directory run the following
	```
	pip3 install -r src/visual_programming/python_dependencies.txt
	```
- (Real Hardware Only)
	```
	wstool merge -t src src/visual_programming/.rosinstall_hardware
	```
	> Installs realsense ROS driver
	
___
####	Joystick Device Details
- This demo uses a Fortune Tech Wireless VR30 gamepad in combination with a bluetooth receiver. However any other USB joystick should work
- Fortune Tech Wireless VR30 gamepad only:
	- To connect go into the bluetooth menu and pair it to the **Fortune Tech Wireless** device

---
#### Joystick Device Test

- Install the following package 
    ```
    sudo apt-get install jstest-gtk
    ```
- Run the `jstest-gtk` application and make sure that you can see your joystick.  In addition to that take note of the joystick's device name (usually */dev/input/js0*)
- Run the `rosrun joy joy_node _dev:=/dev/input/js0` command and echo the `/joy` topic to make sure that the ros driver publishes the joystick messages on button presses
- (Optional) More info on configuring a joystick to work with ROS can be found at [here](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

---
### Run Application
This application allows adding/removing waypoints to a robot trajectory. Once a trajectory has been created,  
the software will generate a correspoding robot joint path, preview and execute the path on the real  
(or simulated) robot. 

  *Currently the Planning and execution capabilities are a work in progress.
- If using Real Hardware, make sure that the joystick, robot and the camera are connected.
- Run the setup launch file:
  - Simulation Mode:
	  ```
	  roslaunch vpr_tracking application_setup.launch
	  ```
    > In tracking simulation mode, Use the slider widgets in the **TF Publisher** window to set the pose of the wand
    
  - Hardware Mode:
  	  ```
	  roslaunch vpr_tracking application_setup.launch sim_tracking:=False sim_robot:=False
	  ```	  

- Start the application:
    Run the application launch file as follows:
  ```
  roslaunch vpr_tracking application_run.launch
  ```
    - A text marker will show in rviz with the available actions:  
      - ADD WAYPOINT
      - CLEAR_WAYPOINTS
      - PREVIEW PATH 
      - DELETE_WAYPOINT 
      - ADD_SEGMENT    
      - MOVE HOME
    - Use the joystick's D-pad up and down directions to select an action.  Then press the trigger button  
    to execute the action.

    - Move the tracking tool while making sure it is in the view of the camera. Add a waypoint by using the `ADD WAYPOINT` action.
      > As you add waypoints to the trajectory a green line Rviz Marker will show the connection between the active waypoints. The yellow line shows the waypoints that were recorded continuously.
      
    - After adding waypoints preview the robot path with the `PREVIEW PATH` action.
    - Then execute the trajection with the `EXECUTE PATH` action.

  *Note: All these command are context dependent so they may not be applicable all the time.
