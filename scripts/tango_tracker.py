import rospy
from sensor_msgs import msg as sens_msg
from geometry_msgs import msg as geom_msg
from cv_bridge import CvBridge, CvBridgeError

import cv2
import imutils

cam_bridge = CvBridge()
position_pub = None

# aruco marker parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters_create()
MARKER_SIZE = 0 # marker width/height in centimeters

# TODO use MARKER_SIZE to scale coordinates

# UI options
SHOW_UI = False # controlled by param server, defaults to False
DRAW_MARKER_CROSS = True
DRAW_MARKER_RECT = True
DRAW_MARKER_ID = True

fid_id = -1 # robot's aruco marker ID, if -1 then assume only one marker and always output location of lowest id present

def detect_fiducial(img, fid_id=-1):
    """Locates ARUCO fiducial marker in given image
    
    Args:
        img (BGR Image): Image to analyse
        fid_id (int, Optional): Marker ID to search for. If -1 then assume only one marker and always output location of lowest id present. Deafults to -1
        
    Returns:
        [int, int]: Location of center of fiducial marker
        BGR Image: Image with identified markers drawn on
    """
    corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

    ret_img = img.copy()

    if ids is not None:
        ids = ids.flatten()

        for marker, id in zip(corners, ids):
            if (id == fid_id or fid_id == -1):
                # convert corner locations to ints
                top_left = marker[0, 0]
                top_right = marker[0, 1]
                bot_right = marker[0, 2]
                bot_left = marker[0, 3]

                top_right = (int(top_right[0]), int(top_right[1]))
                bot_right = (int(bot_right[0]), int(bot_right[1]))
                bot_left = (int(bot_left[0]), int(bot_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # calculate centre of marker
                center = (int((top_right[0]+bot_left[0])/2), int((top_left[1]+bot_right[1])/2))

                if SHOW_UI and DRAW_MARKER_CROSS:
                    # draw cross at centre
                    cv2.line(ret_img, (center[0]-20, center[1]), (center[0]+20, center[1]), (0, 0, 255), 1)
                    cv2.line(ret_img, (center[0], center[1]-20), (center[0], center[1]+20), (0, 0, 255), 1)
                
                if SHOW_UI and DRAW_MARKER_RECT:
                    # draw box around marker
                    cv2.line(ret_img, top_left, top_right, (0, 255, 0), 2)
                    cv2.line(ret_img, top_right, bot_right, (0, 255, 0), 2)
                    cv2.line(ret_img, bot_right, bot_left, (0, 255, 0), 2)
                    cv2.line(ret_img, bot_left, top_left, (0, 255, 0), 2)

                if SHOW_UI and DRAW_MARKER_ID:
                    # draw marker id text
                    cv2.putText(ret_img, f"FID{id}", bot_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

                return center, ret_img # if successfuly found marker, return location
    
    # if reaches here, didnt find marker, or didnt find marker with id indicated

    if fid_id == -1:
        rospy.loginfo("Didnt find marker in frame")
    else:
        rospy.loginfo("Didnt find marker with ID f{fid_id} in frame")

    return None, ret_img

def image_callback(img_msg):
    """rgb image message callback
    """

    # Try to convert the ROS Image message to a CV2 Image
    try:
        image = cam_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.logerr("Failed to convert ROS message to image")

    # analyse image
    image = imutils.resize(image, height=500)
    rospy.loginfo(f"Image size after scaling: w={image.shape[1]} h={image.shape[0]}")
    marker_center, image = detect_fiducial(image)

    if marker_center is not None: # None if no marker detected

        # publish center position and debug image
        if position_pub is None:
            rospy.logerr("Position publisher not initialised")
        if final_img_pub is None:
            rospy.logerr("Image publisher not initialised")
        position_pub.publish(x=marker_center[0], y=marker_center[1])
        final_img_pub.publish(cam_bridge.cv2_to_imgmsg(image, encoding="bgr8"))

    if SHOW_UI:
        # display image
        cv2.imshow("Image Window", image)

        # handle user input
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User requested shutdown")


if __name__ == "__main__":

    rospy.init_node('tango_tracker')
    rate = rospy.Rate(7) # ROS Rate at 5Hz
    rospy.loginfo("Tango Tracker started")

    SHOW_UI = rospy.get_param('~show_ui', default=False)
    if type(SHOW_UI) is not bool:
        rospy.logerr("param ~show_ui must be a boolean")

    fid_id = rospy.get_param('~robot_arucoID', default=-1)
    if type(fid_id) is not int:
        rospy.logerr("param ~robot_arucoID must be an integer")

    MARKER_SIZE = rospy.get_param('~marker_size')

    image_sub = rospy.Subscriber("/camera/color/image_raw", sens_msg.Image, image_callback)
    position_pub = rospy.Publisher("/position_2d", geom_msg.Point, queue_size=10)
    final_img_pub = rospy.Publisher("/tracker_debug/final_img", sens_msg.Image, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()