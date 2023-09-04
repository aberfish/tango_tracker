#!/usr/bin/env python3
from __future__ import print_function

import roslib
import os, sys
import rospy, rospkg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from time import sleep





class image_converter:



  def __init__(self):
    rospy.init_node('aruco_realsense', anonymous=False)
    ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
  "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
  "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
  "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
  "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
    aruco_type = "DICT_5X5_250"
    self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    self.aruco_params = cv2.aruco.DetectorParameters_create()

    self.bridge = CvBridge()
    self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    self.info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
    self.pub_annotated_image = rospy.Publisher("/tango/aruco_annotated",Image, queue_size=10) #publish annotated image

    ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
    ts.registerCallback(self.callback)


  def callback(self, data, ros_cinfo):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    c = cv2.waitKey(1)
    if c == 27: #this is the escape key
      cv2.destroyAllWindows()

    corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

    detected_markers = aruco_display(corners, ids, rejected, cv_image)

    self.rosimgannotated = Image()
    self.rosimgannotated.header.stamp = rospy.Time.now()
    self.rosimgannotated = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
    self.rosimgannotated.header.frame_id = "zed_camera_center"
    self.pub_annotated_image.publish(self.rosimgannotated) #publish annotated image

def aruco_display(corners, ids, rejected, img):
    if len(corners) > 0:
        print(len(corners))
        ids = ids.flatten()

        for (marker_corner, marker_id) in zip(corners, ids):

            corners = marker_corner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners
            
            top_right = (int(top_right[0]), int(top_right[1]))
            top_left = (int(top_left[0]), int(top_left[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

            cv2.line(img, top_left, top_right, (0, 255, 0), 2)
            cv2.line(img, top_right, bottom_right, (0, 255, 0), 2)
            cv2.line(img, bottom_right, bottom_left, (0, 255, 0), 2)
            cv2.line(img, bottom_left, top_left, (0, 255, 0), 2)

            c_x = int((top_left[0] + bottom_right[0]) / 2.0)
            c_y = int((top_left[1] + bottom_right[1]) / 2.0)
            cv2.circle(img, (c_x, c_y), 4, (0, 0, 255), -1)

            cv2.putText(img, str(marker_id), (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(marker_id))

    return img

    try:
        cv2.imshow('image', cv_image)
        sleep(0.1)
        cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)



def main(args):
  
  #rospy.init_node('aruco_ros1', anonymous=False)
  
  ic = image_converter()
  try:
    rospy.spin()
    #rospy.sleep(rate)
    print("Shutting down")
    #foundObjectsFileWrite()


  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
