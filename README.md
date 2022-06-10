# tango_tracker
ROS package using opencv to track a wheeled robot using fiducial markers.

## Nodes
### tango_tracker.py
Detects ARUCO-5x5-50 markers in each message from the color image input and calculates their location.

Subscribes:
 - */camera/color/raw_image*: Color image input

Publishes:

*NONE*

Parameters:
 - *~show_ui* (boolean, Optional): If True, show the opencv tracking UI for debugging. Defaults to false
 - *~robot_arucoID* (int, Optional): ID of the Aruco marker on the robot. Defaults to selecting the lowest Aruco marker ID in the frame


## Setup
Place contents of repository in the directory ```<ros_workspace_dir>/src/tango_tracker```. Build and source your ROS workspace.