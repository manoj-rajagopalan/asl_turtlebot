#  Demo Instructions

##### Step 1: Launch the Bringup

`roslaunch asl_turtlebot turtlebot3_bringup_jetson_pi.launch`

##### Step 2: Launch the Supervisor

`./scripts/supervisor.py`

##### Step 3: Launch Teleop

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

##### Step 4: Manual Mapping

Manually explore the environment. Be sure all relevant landmarks are localized.

##### Step 5: Stop Teleop

Kill the teleop process.

##### Step 6: Switch to Auto Mode

`rostopic pub man_control std_msgs/String "Auto"`

##### Step 7: Send Orders

`./scripts/request_publisher.py`

