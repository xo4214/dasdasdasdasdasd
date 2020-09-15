# Running SLAM using TurtleBot3 Manipulator

 SLAM of TurtleBot3 with OpenMANIPULATOR-X is different from SLAM we learned earlier. Since the robot arm blocks certain parts of the LDS sensor, we can only map smoothly by limiting the range of the LDS sensor used in SLAM.  
 You can map without using invalid angle values by filtering the set angle range data of the LDS sensor from turtlebot3_manipulation_slam/config/scan_data_filter.yaml file as shown below.    
 scan_filter_chain:  
- name: Remove 120 to 240 degree  
- type: laser_filters/LaserScanAngularBoundsFilterInPlace  
- params:  
    lower_angle: 2.0944  
    upper_angle: 4.18879  
### Run roscore  

 [Remote PC] Run roscore with the command below.  

 $ roscore  

 [TurtleBot3 SBC] Run the node that activates rosserial and LDS sensor with the command below.  

 $ roslaunch turtlebot3_bringup turtlebot3_robot.launch  

**roslaunch turtlebot3_bringup turtlebot3_robot.launch**  

**1. turtlebot3_core.launch**  
- subscribe : cmd_vel, joint_trajectory_point, gripper_position
- publish : joint_states, odom

**2. turtlebot3_lidar.launch**
- publish : scan

 Since the OpenCR firmware has changed, running turtlebot3_robot.launch file subscribes joint_trajectory_point and gripper_position in addition to the topics created when you run turtlebot3_robot.launch described above. joint_trajectory_point is the position value of each joint of the robot arm, and it is transferred to each motor of the robot arm through OpenCR.   gripper_position is the joint position value of the robot gripper, and it is transferred to the robot gripper motor through OpenCR to move the robot.   
 By using the Message Publisher of rqt, you can control simply by giving a position value.  

 Manipulation is not used when creating a map using SLAM, so there is no need to run OpenMANIPULATOR controller and move_group interface shown below.  

 $ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch  
 $ roslaunch turtlebot3_manipulation_moveit_config move_group.launch  

### Running SLAM Node

 [Remote PC] In this part, we run SLAM using Gmapping

 $ roslaunch turtlebot3_manipulation_slam slam.launch

**roslaunch turtlebot3_manipulation_slam slam.launch**

**1. urdf**
- Unified Robot Description Format. An XML format file that expresses the configuration and connection type of the robot.  

**2. robot_state_publisher**
- receives information about each joint of the robot and publishes the obtained joint information in tf (transform) format by referring to urdf.  
- subscribe : joint_states
- publish : tf

**3. laser_filter Node**
-  Runs a node that filters invalid value range of the LDS sensor. This part ignores the angle of the rear part where the OpenMANIPULATOR is installed.  

**4. turtlebot3_gmapping.launch**
- Parameter information required to run SLAM using Gmapping is loaded onto the parameter server. Use this setting to set gmapping.  

**5. turtlebot3_gmapping.rviz**
- Run RViz by applying the default settings of RViz required to display SLAM with Gmapping on the RViz screen.  
 
 The robot's urdf is loaded from the defined space when you run urtlebot3_manipulation_slam.launch. It also uses joint_states and urdf to create robot_state_publisher node that publishes tf.  

### Running turtlebot3_teleop_key Node

 [Remote PC] Complete map by moving the robot to the uncharted part of the map.  

 $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  

 [Remote PC] When the map is completed, run map_saver node to save the map file.  

 $ rosrun map_server map_saver -f ~/${map_name}  

 <-f> option specifies the map file save location and file name. The command above uses ~/${map_name} option, so it will save as `${map_name}.pgm` and `${map_name}.yaml`… in the user’s home folder (~/ or /home/). Enter the file name in ${map_name}.  

# Navigation

 Navigation of TurtleBot3 with OpenMANIPULATOR is not too different from Navigation run in default TurtleBot’s platform. However, as in SLAM, it is preferred to set the LDS sensor range. Relevant nodes that can control the robot arm and gripper can be run to run OpenMANIPULATOR during Navigation if necessary.  

### Running roscore

 [Remote PC] Run roscore
 $ roscore

### Running Bringup

 [TurtleBot3 SBC] Run the node that activates rosserial and LDS sensor with the command below.  

 $ roslaunch turtlebot3_bringup turtlebot3_robot.launch  

**roslaunch turtlebot3_bringup turtlebot3_robot.launch**

**1. turtlebot3_core.launch**
- subscribe : cmd_vel, joint_trajectory_point, gripper_position  
- publish : joint_states, odom  

**2. turtlebot3_lidar.launch**
- publish : scan

 Since the OpenCR firmware has changed, running turtlebot3_robot.launch file subscribes joint_trajectory_point and gripper_position in addition to the topics created when you run turtlebot3_robot.launch described above. joint_trajectory_point is the position value of each joint of the robot arm, and it is transferred to each motor of the robot arm through OpenCR.   gripper_position is the joint position value of the robot gripper, and it is transferred to the robot gripper motor through OpenCR to move the robot. By using the Message Publisher of rqt, you can control simply by giving a position value.  

### Running Navigation

 [Remote PC] Running the command below loads various parameters and maps to run Navigation. It also loads URDF and RViz configuration to create GUI environment.
Many nodes will run simultaneously, so check the files and nodes first.

**$ roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml**

**1. urdf**
- Imports turtlebot3_manipulation_robot.urdf.xacro file in the form of a combination of TurtleBot3 and OpenMANIPULATOR. In this file, the files describing the shapes of TurtleBot3 and OpenMANIPULATOR are combined to create the overall robot shape.  

**2. robot_state_publisher**
- The robot_state_publisher receives information on each joint of the robot and publishes the obtained data in tf format by referring to the urdf.

**3. laser_filter**
- Filters the set angle range data of the LDS sensor.

**4. map_server**
- Imports the map and configuration file completed with SLAM.

**5. amcl.launch**
- Imports various parameters to use AMCL particle filter. Reads initialpose and tf using the map and sensor scan values and predicts the robot location using a particle filter.

**6. move_base.launch**
- move_base node provides a ROS interface to access the robot’s Navigation stack. move_base connects the global planner and the local planner to move the robot to the destination, and it stores the costmap for each planner at the same time. When the goal is received in the form of an Action, it transfers in the form of an Action to update the current position (feedback), status, and movement result. In addition, the cmd_vel topic for moving the robot according to the current state is continuously published.  

**7. rviz**
- Creates GUI window that visualizes various data and parameters.  

 roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml

### Controlling OpenMANIPULATOR

 If you create a node that controls OpenMANIPULATOR when running Navigation, you can control the robot arm with Navigation.  

 If you move OpenMANIPULATOR during robot motion, vibration or shift in the center of gravity can cause unstable robot arm motion. It is recommended to move robot arms when they are inactive.

**Running turtlebot3_manipulation_bringup Node**

 [Remote PC] Run arm_controller and gripper_controller just like controlling OpenMANIPULATOR.

 $ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch

**roslaunch turtlebot3_manipulation_bringup**
**turtlebot3_manipulation_bringup.launch**

**turtlebot3_manipulation_bringup Node**

 Running turtlebot3_manipulation_bringup.launch runs both arm_controller and gripper_controller. As an action server controller that communicates with move_group, it imports target trajectories of the arm and the gripper joints through each move_group and publishes them in order. Published topics are delivered to DYNAMIXEL assembled on the robot's joint through OpenCR to move the OpenMANIPULATOR.

**Running move_group Node**

 After running the move_group node, you can use MoveIt or ROBOTIS GUI to control OpenMANIPULATOR. This manual describes two ways to run ROBOTIS GUI. Use the appropriate interface between the two methods.

 $ roslaunch turtlebot3_manipulation_moveit_config move_group.launch

**roslaunch turtlebot3_manipulation_moveit_config move_group.launch**

 Running move_group.launch runs move_group node. The move_group node receives commands through the user interface and sends them to the robot controller in the form of an action.  

**Running ROBOTIS GUI Controller**

 [Remote PC] ROBOTIS GUI supports Task Space Control that refers to the effective gripping position (red hexahedron between grippers) of the gripper based on the first DYNAMIXEL of OpenMANIPULATOR and Joint Space Control that refers to the angle of each joint. Either methods can be used as needed.

$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch

**roslaunch turtlebot3_manipulation_gui **
**turtlebot3_manipulation_gui.launch**

 qt gui using c++ move_group_interface as user interface is run. The current joint position and end-effector position received through the interface are displayed on the gui. If you click the Send button, the set position value is transmitted to the move_group through the interface and transferred to the controller, and moves the robot.


 © 2020 ROBOTIS. Powered by Jekyll & Minimal Mistakes.