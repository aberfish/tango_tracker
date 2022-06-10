# tango_tracker
ROS package using opencv to track a wheeled robot using fiducial markers.

## Nodes
### tango_tracker.py
Detects ARUCO-5x5-50 markers in each message from the color image input and publishes their location.

#### Subscribes:
 - */camera/color/raw_image* (sensor_msgs.Image): Color image input

#### Publishes:
 - */position* (geometry_msgs.Point): Image X and Y coordinates of the robot. Z is always zero

#### Parameters:
 - *~show_ui* (boolean, Optional): If True, show the opencv tracking UI for debugging. Defaults to false
 - *~robot_arucoID* (int, Optional): ID of the Aruco marker on the robot. Defaults to selecting the lowest Aruco marker ID in the frame

#### Usage:
To **track the robot**:
```rosrun tango_tracker tango_tracker.py```

To track the robot and **show the debug UI**:
```rosrun tango_tracker tango_tracker.py _show_ui:=True```

To track the robot with a **specific marker ID**:
```rosrun tango_tracker tango_tracker.py _robot_arucoID:=<marker_id>```

## Setup
Place contents of repository in the directory ```<ros_workspace_dir>/src/tango_tracker```. Build and source your ROS workspace.