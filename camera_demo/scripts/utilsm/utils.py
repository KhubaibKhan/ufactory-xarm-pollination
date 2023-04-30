import rospy
import tf2_ros
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from yolov5.detect2 import *

# Initialize cv_bridge
cv_bridge = CvBridge()

# Realsense class to get aligned image and depth image
class Realsense:
    def __init__(self):
        # Subscribe to aligned image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Subscribe to aligned depth image topic
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

        # Subscribe to camera info topic
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # camera info
        self.camera_info = None

        # Initialize cv_bridge
        self.cv_bridge = CvBridge()

        # Global image
        self.image = None
        self.depth = None

    # Function that gets depth from depth image
    def get_depth(self, depth, xc, yc):
        # Convert ROS image to OpenCV image
        cv_depth = cv_bridge.imgmsg_to_cv2(depth, 'passthrough')

        # Get the average depth in the 5x5 region around the centroid
        depth = np.mean(cv_depth[yc - 10:yc + 10, xc - 10:xc + 10])


        # Return depth
        return depth
    
    # Function that gets image from image topic
    def image_callback(self, image):
        self.image = image

    # Function that gets depth from depth topic
    def depth_callback(self, depth):
        self.depth = depth

    # Function that gets camera info from camera info topic
    def camera_info_callback(self, camera_info):
        self.camera_info = camera_info

# Function that detects color block in image
def detect_color_block(image, color):
    # Convert ROS image to OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')
    original_size = cv_image.shape

    # resize the image
    cv_image = cv2.resize(cv_image, (640, 480))
    # cv_image = cv2.flip(cv_image, 0)
    # flip the image to match the camera

    xyxy = run(weights="/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/yolov5/runs/train/exp7/weights/best.pt", image=cv_image)
    
    # Draw the xyxy box on the image
    for box in xyxy:
        cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        cv2.putText(cv_image, box[4], (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show image
    cv2.imshow("Image", cv_image)
    cv2.waitKey(1)

    if len(xyxy) == 0:
        return []
    # Find the center of each xyxy box
    centroids   = []
    for box in xyxy:
        xc = int((box[0] + box[2]) / 2)
        yc = int((box[1] + box[3]) / 2)
        # Resize the centroids to the original image size
        xc = int(xc * (original_size[1] / 640))
        yc = int(yc * (original_size[0] / 480))

        centroids.append([xc, yc, box[4]])

    return centroids

    # # Convert image to HSV format
    # cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # # Define color range
    # color_lower = (color[0] - 10, 100, 100)
    # color_upper = (color[0] + 10, 255, 255)

    # # Create mask for color range
    # mask = cv2.inRange(cv_image_hsv, color_lower, color_upper)

    # # Find contours in mask
    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # if len(contours) == 0:
    #     return -1, -1
    # # Find largest contour in mask
    # contour = max(contours, key=cv2.contourArea)

    # # find the xmin, ymin, xmax, ymax of the contour
    # x, y, w, h = cv2.boundingRect(contour)

    # # Find centroid of largest contour
    # M = cv2.moments(contour)
    # xc = int(M['m10'] / M['m00'])
    # yc = int(M['m01'] / M['m00'])

    # # Draw contour and centroid on image
    # cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)
    # cv2.circle(cv_image, (xc, yc), 7, (255, 255, 255), -1)
    # cv2.putText(cv_image, "centroid", (xc - 20, yc - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # # Show image
    # cv2.imshow("Image", cv_image)
    # cv2.waitKey(1)

    # # Return centroid
    # return xc, yc

# Using image and depth find the 3D position of the block
def get_3d_position(depth, xc, yc, camera_info=None):
    # Get camera intrinsics
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    # print("Camera info", camera_info.K)

    # Get depth
    z = depth

    # Get 3D position
    x = (xc - cx) * z / fx
    y = (yc - cy) * z / fy

    # # Normalize the 3D position
    x = x / 1000.0
    y = y / 1000.0
    z = z / 1000.0


    # Return 3D position
    return x, y, z

# Transform 3d position from camera frame to base frame
def transform_3d_position(x, y, z, camera_frame, base_frame, tf_buffer):
    # Initialize transform
    transform = None

    # Wait for transform
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform(base_frame, camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # Create point
    point = PointStamped()
    point.header.frame_id = camera_frame
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = z

    # Transform point
    transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

    # Return transformed point
    return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z