# tango_tracker
ROS package using opencv to track a wheeled robot using fiducial markers.

- [tango_tracker](#tango_tracker)
  - [Nodes](#nodes)
    - [tango_tracker.py](#tango_trackerpy)
      - [Subscribes:](#subscribes)
      - [Publishes:](#publishes)
      - [Parameters:](#parameters)
      - [Usage:](#usage)
  - [Setup](#setup)
  - [Launch Files](#launch-files)
    - [tracker_debug](#tracker_debug)
      - [Arguments](#arguments)

## Nodes
### tango_tracker.py
Detects ARUCO-5x5-50 markers in each message from the color image input and publishes their location.

All input images are scaled to height = 500px, keeping constant aspect ratio. Current camera images will end up with h=500px w=888px.

#### Subscribes:
 - */camera/color/image_raw* (sensor_msgs.Image): Color image input

#### Publishes:
 - */orientation_2d* (stad_msgs.Int8): Angle of marker in degrees, where 0 is right on the x axis, increasing counter-clockwise
 - */position*_2d (geometry_msgs.Point): Image X and Y coordinates of the robot. Z is always zero
 - */tracker_debug/final_img* (sensor_msgs.Image): Resultant debug image showing detected markers

#### Parameters:
 - *~show_ui* (boolean, Optional): If True, show the opencv tracking UI for debugging. Defaults to false
 - *~robot_arucoID* (int, Optional): ID of the Aruco marker on the robot. Defaults to selecting the lowest Aruco marker ID in the frame
 - *~marker_size* (float): Measuerment in **centimeters** of aruco marker width/height

#### Usage:
To **track the robot**:
```rosrun tango_tracker tango_tracker.py```

To track the robot and **show the debug UI**:
```rosrun tango_tracker tango_tracker.py _show_ui:=True```

To track the robot with a **specific marker ID**:
```rosrun tango_tracker tango_tracker.py _robot_arucoID:=<marker_id>```

## Setup
Place contents of repository in the directory ```<ros_workspace_dir>/src/tango_tracker```. Build and source your ROS workspace.

## Launch Files
### tracker_debug
Example usage: 

```roslaunch tango_tracker tracker_debug.launch marker_size:=5 rosbag_path:="/home/rose/Documents/TangoProject/2022-06-06-14-05-29.bag"```

#### Arguments
 - *marker_size* (float): Measuerment in **centimeters** of aruco marker width/height
 - *rosbag_path* (str): Absolute path to a rosbag containing the */camera/color/raw_image* topic