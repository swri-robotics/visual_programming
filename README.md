# 10.R8803
Visual Robot Programming IR&amp;D

### Requirements
##### Install the Intel Realsense SDK 2.0
- Follow the instructions for [installing the Realsense SDK only](https://github.com/intel-ros/realsense)

##### Catkin Workspace Setup
- Use `wstool` to download all the required repositories
- Use `rosdep` from the workspace top level directory to install all required debians
	```
	rosdep install --from-path src --ignore-src
	```
- Install python dependencies, from the workspace top level directory run the following
	```
	pip2.7 install -r src/10.R8803/requirements.txt 
	```
___
#####	Joystick Device Setup
- This demo uses a Fortune Tech Wireless VR30 gamepad in combination with a bluetooth receiver.
- The joystick shows up as a USB device
- To connect go into the bluetooth menu and pair it to the **Fortune Tech Wireless** device

---
##### Joystick Driver Setup


- Configure the USB Joystick as described [here](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
- Install the following package 
    ```
    sudo apt-get install jstest-gtk
    ```
- Run the `jstest-gtk` application and make sure that you can see your joystick.  In addition to that take note of the joystick's device name (usually */dev/input/js0*)
- Run the `rosrun joy joy_node _dev:=/dev/input/js0` command and echo the `/joy` topic to make sure that the ros driver publishes the joystick messages on button presses

---
### Run Application
This application allows adding/removing waypoints to a robot trajectory. Once a trajectory has been created,  
the software will generate a correspoding robot joint path, preview and execute the path on the real  
(or simulated) robot. 

  *Currently the Planning and execution capabilities are a work in progress.
- Make sure that the joystick, robot and the camera are connected.
- Run the setup launch file:
  ```
  roslaunch vpr_tracking application_setup.launch sim_tracking:=False sim_robot:=False
  ```
  
  > Omit the `sim_tracking` and `sim_robot` arguments when running in simulation mode

  > In tracking simulation mode, you'll see the robot in Rviz as well as a gui window with sliders to set the location of the tracked frame

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
