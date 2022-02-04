# ROS2 package: vehicle_controller

### This package is mainly responsible for controlling PX4 in the OFFBOARD mode by sending offboard control commands and velocity setpoints to PX4.<br><br>

This package has a node called the **controller** node which acts as a layer between the **avoidance_node** node in the **collision_avoidance** package and PX4.<br>

It basically does the following:<br>
1. Subscribes to the **GoalTrajectory_PubSubTopic** to get the velocity setpoint data from the **collision_avoidance** package via the **TrajectorySetpoint** PX4 message.<br>
2. Syncs the timestamp with PX4 and adds a timestamp to the velocity setpoint message.<br>
3. Publishes the **OffboardControlMode** with the correct configuration along with the velocity setpoint to PX4 frequently.<br>
4. Publishes a VehicleCommand message to PX4 to land the UAV when landing is signaled from the **avoidance_client** node. <br>

This node gets the MAVLink System ID of the vehicle as a parameter. This ID should match the MAVLink ID that is configured on PX4 side otherwise, PX4 will ignore the the commands that are sent from the companion computer.

This node is launched using the **collision_avoidance_bringup** package.
