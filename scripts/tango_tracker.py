import rospy
from sensor_msgs import msg
from cv_bridge import CvBridge, CvBridgeError

import cv2
import imutils

cam_bridge = CvBridge()

# aruco marker parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
aruco_params = cv2.aruco.DetectorParameters_create()

DRAW_MARKER_CROSS = True
DRAW_MARKER_RECT = False
DRAW_MARKER_ID = False

def detect_fiducial(img):
    """Identifies ARUCO fiducial markers in given image
    
    Args:
        img (BGR Image): Image to analyse
    
    Returns:
        array: Array of corners identified
        array: Array of marker ids identified
        BGR Image: Image with identified markers drawn on
    """
    corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

    out = {}

    ret_img = img.copy()

    if ids is not None:
        ids = ids.flatten()

        for marker, id in zip(corners, ids):
            top_left = marker[0, 0]
            top_right = marker[0, 1]
            bot_right = marker[0, 2]
            bot_left = marker[0, 3]

            top_right = (int(top_right[0]), int(top_right[1]))
            bot_right = (int(bot_right[0]), int(bot_right[1]))
            bot_left = (int(bot_left[0]), int(bot_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))

            center = (int((top_right[0]+bot_left[0])/2), int((top_left[1]+bot_right[1])/2))
            out[f'{id}'] = center

            if DRAW_MARKER_CROSS:
                # draw cross at centre
                cv2.line(ret_img, (center[0]-20, center[1]), (center[0]+20, center[1]), (0, 0, 255), 1)
                cv2.line(ret_img, (center[0], center[1]-20), (center[0], center[1]+20), (0, 0, 255), 1)
            
            if DRAW_MARKER_RECT:
                # draw box around marker
                cv2.line(ret_img, top_left, top_right, (0, 255, 0), 2)
                cv2.line(ret_img, top_right, bot_right, (0, 255, 0), 2)
                cv2.line(ret_img, bot_right, bot_left, (0, 255, 0), 2)
                cv2.line(ret_img, bot_left, top_left, (0, 255, 0), 2)

            if DRAW_MARKER_ID:
                # draw marker id text
                cv2.putText(ret_img, f"FID{id}", bot_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
    else:
        rospy.loginfo("Didnt find marker in frame")

    return out, ret_img

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
    marker_centers, image = detect_fiducial(image)

    # display iamge
    cv2.imshow("Image Window", image)

    # handle user input
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown")


if __name__ == "__main__":

    rospy.init_node('tango_tracker')
    rospy.loginfo("Tango Tracker started") 

    image_sub = rospy.Subscriber("/camera/color/image_raw", msg.Image, image_callback)

    while not rospy.is_shutdown():
        rospy.spin()