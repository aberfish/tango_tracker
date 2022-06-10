# tango_tracker
ROS package using opencv to track a wheeled robot using fiducial markers.

## Nodes
### tango_tracker.py
Detects ARUCO-5x5-50 markers in each message from the color image input and calculates their location.

Subscribes:
 - /camera/color/raw_image: Color image input

Publishes:
    NONE


## Setup
Place contents of repository in the directory ```<ros_workspace_dir>/src/tango_tracker```. Build and source your ROS workspace.