import rospy
import tf2_ros
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
# from yolov5.detect2 import *
from ultralytics import YOLO
from boxmot import DeepOCSORT
from pathlib import Path

# Initialize cv_bridge
cv_bridge = CvBridge()

# Load the yolo model
yolo_model = YOLO('/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/weights/best_khu.pt')

# start a cv2 videowriter
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter(f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/{time.time()}.avi', fourcc, 20.0, (640, 480))

def create_videoWriter(name):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    global out
    out = cv2.VideoWriter(f'/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/exp_results/{name}.avi', fourcc, 20.0, (640, 480))


# tracker
# tracker = DeepOCSORT(
#     model_weights=Path('/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/weights/osnet_x1_0.pt'),
#     device='cuda:0',
#     fp16=False,
# )

# plotting parameters
color = (255, 255, 255)
thickness = 2
fontscale = 0.5

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
    def get_depth(self, depth, image, xc, yc, hw):
        # Convert ROS image to OpenCV image
        cv_depth = cv_bridge.imgmsg_to_cv2(depth, 'passthrough')
        
        # Convert ROS image to OpenCV image
        cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')

        # Get the average depth of all the yellow pixels in the block
        box_region_depth = cv_depth[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]
        box_region_color = cv_image[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]

        # Detect the yellow pixels in the box region
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(box_region_color, lower_yellow, upper_yellow)
        yellow_pixels = np.where(yellow_mask == 255)

        # Get the average depth of the yellow pixels
        depth = np.mean(box_region_depth[yellow_pixels[0], yellow_pixels[1]])


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

    # Function that gets depth from depth image
    def get_depth(self, depth_msg, rgb_msg, xc, yc, hw):
        cv_depth = cv_bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        cv_depth = cv2.resize(cv_depth, (640, 480))
        # Convert ROS image to OpenCV image
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        image_size = cv_image.shape
        # Now you have aligned depth image with respect to the RGB image
        # You can use the aligned_depth_image along with rgb_image for further processing or visualization

        # # Get the average depth of all the yellow pixels in the block
        # box_region_depth = cv_depth[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]
        # box_region_color = cv_image[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]

        # # # Detect the yellow pixels in the box region
        # # lower_yellow = np.array([20, 100, 100])
        # # upper_yellow = np.array([30, 255, 255])
        # # yellow_mask = cv2.inRange(box_region_color, lower_yellow, upper_yellow)
        # # yellow_pixels = np.where(yellow_mask == 255)

        # # Get the average depth of the yellow pixels
        # depth = np.mean(box_region_depth)

        # resize the centroids from image size to 640x480
        xc = int(xc * 640 / image_size[1])
        yc = int(yc * 480 / image_size[0])


        # get the depth of the center pixel and its 8 neighbors
        depth = cv_depth[yc, xc]
        depth += cv_depth[yc - 1, xc]
        depth += cv_depth[yc + 1, xc]
        depth += cv_depth[yc, xc - 1]
        depth += cv_depth[yc, xc + 1]
        depth += cv_depth[yc - 1, xc - 1]
        depth += cv_depth[yc - 1, xc + 1]
        depth += cv_depth[yc + 1, xc - 1]
        depth += cv_depth[yc + 1, xc + 1]
        depth /= 9

        xc += 5

        # plot the points where we get the depth on the depth frame
        cv2.circle(cv_depth, (xc, yc), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc, yc - 1), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc, yc + 1), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc - 1), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc + 1), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc - 1), 20, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc + 1), 20, (0, 0, 255), 2)

        # show using imshow
        cv2.imshow("Depth circles", cv_depth)
        cv2.waitKey(1)


        # Return depth
        return depth


# Realsense class to get aligned image and depth image
class AzureKinect:
    def __init__(self):
        # Subscribe to aligned image topic
        self.image_sub = rospy.Subscriber('/rgb/image_rect_color', Image, self.image_callback)

        # Subscribe to aligned depth image topic
        self.depth_sub = rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.depth_callback)

        # Subscribe to camera info topic
        self.camera_info_sub = rospy.Subscriber('/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # subscribe to depth camera info topic
        self.depth_info_sub = rospy.Subscriber('/depth/camera_info', CameraInfo, self.depth_camera_info_callback)

        # camera info
        self.camera_info = None

        # Initialize cv_bridge
        self.cv_bridge = CvBridge()

        # Global image
        self.image = None
        self.depth = None
    

    # Function that gets depth from depth image
    def get_depth(self, depth_msg, rgb_msg, xc, yc, hw):
        cv_depth = cv_bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        cv_depth = cv2.resize(cv_depth, (640, 480))
        # Convert ROS image to OpenCV image
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        image_size = cv_image.shape
        # Now you have aligned depth image with respect to the RGB image
        # You can use the aligned_depth_image along with rgb_image for further processing or visualization

        # # Get the average depth of all the yellow pixels in the block
        # box_region_depth = cv_depth[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]
        # box_region_color = cv_image[yc - int(hw[1] / 2):yc + int(hw[1] / 2), xc - int(hw[0] / 2):xc + int(hw[0] / 2)]

        # # # Detect the yellow pixels in the box region
        # # lower_yellow = np.array([20, 100, 100])
        # # upper_yellow = np.array([30, 255, 255])
        # # yellow_mask = cv2.inRange(box_region_color, lower_yellow, upper_yellow)
        # # yellow_pixels = np.where(yellow_mask == 255)

        # # Get the average depth of the yellow pixels
        # depth = np.mean(box_region_depth)

        # resize the centroids from image size to 640x480
        xc = int(xc * 640 / image_size[1])
        yc = int(yc * 480 / image_size[0])


        # get the depth of the center pixel and its 8 neighbors
        depth = cv_depth[yc, xc]
        depth += cv_depth[yc - 1, xc]
        depth += cv_depth[yc + 1, xc]
        depth += cv_depth[yc, xc - 1]
        depth += cv_depth[yc, xc + 1]
        depth += cv_depth[yc - 1, xc - 1]
        depth += cv_depth[yc - 1, xc + 1]
        depth += cv_depth[yc + 1, xc - 1]
        depth += cv_depth[yc + 1, xc + 1]
        depth /= 9

        xc += 5

        # plot the points where we get the depth on the depth frame
        cv2.circle(cv_depth, (xc, yc), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc, yc - 1), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc, yc + 1), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc - 1), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc - 1, yc + 1), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc - 1), 2, (0, 0, 255), 2)
        cv2.circle(cv_depth, (xc + 1, yc + 1), 2, (0, 0, 255), 2)

        # show using imshow
        cv2.imshow("Depth circles", cv_depth)
        cv2.waitKey(1)


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

    def depth_camera_info_callback(self, depth_camera_info):
        self.depth_camera_info = depth_camera_info

# Function that detects color block in image
def detect_color_block_python(cv_imageimage, cv_depth, color):
    original_size = cv_image.shape
    # conver it into colored image
    # print(np.unique(cv_depth, return_counts=True))
    cv_depth = cv2.applyColorMap(cv2.convertScaleAbs(cv_depth, alpha=0.03), cv2.COLORMAP_JET)

    depth_resize = cv2.resize(cv_depth, (640, 480))

    # resize the image
    cv_image = cv2.resize(cv_image, (640, 480))
    # cv_image = cv2.flip(cv_image, 0)
    # flip the image to match the camera

    # xyxy = run(weights="/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/yolov5/best.pt", image=cv_image, imgsz=(640, 480))
    results = yolo_model(cv_image, imgsz=640, agnostic_nms=True, iou=0.3, conf=0.2, classes=[0, 1, 2, 3, 4, 5, 6, 7, 8])[0]
    boxes = results.boxes.cpu().numpy()
    data = boxes.data
    
    # ts = tracker.update(data, cv_image)                     # update tracker
    # print("Tracker", ts)
    # if ts.shape[0] != 0:
    #     xyxys = ts[:, 0:4].astype('int')
    #     ids = ts[:, 4].astype('int')
    #     confs = ts[:, 5]
    #     clss = ts[:, 6]
    #     for xyxy, id, conf, cls in zip(xyxys, ids, confs, clss):
    #         cv_image = cv2.rectangle(
    #             cv_image,
    #             (xyxy[0], xyxy[1]),
    #             (xyxy[2], xyxy[3]),
    #             color,
    #             thickness
    #         )
    #         cv2.putText(
    #             cv_image,
    #             f'{id} {conf:.2f} {cls}',
    #             (xyxy[0], xyxy[1] - 5),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             fontscale,
    #             color,
    #             thickness
    #         )

    # Draw the xyxy box on the image
    for box in data:
        cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        cv2.putText(cv_image, yolo_model.names[box[5]], (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # put the class score on the image
        cv2.putText(cv_image, str(box[4]), (int(box[0]), int(box[1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


        # shift the bounding box by 21 pixels to the right and draw it on depth image
        cv2.rectangle(depth_resize, (int(box[0]) + 21, int(box[1])), (int(box[2]) + 21, int(box[3])), (255, 0, 0), 2)
        cv2.putText(depth_resize, yolo_model.names[box[5]], (int(box[0]) + 21, int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # put the class score on the image
        # cv2.putText(depth_resize, str(box[4]), (int(box[0]) + 21, int(box[1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # write a frame to the video
    out.write(cv_image)
    
    # Show image
    cv2.imshow("Image", cv_image)
    cv2.imshow("Depth", depth_resize)
    cv2.waitKey(1)

    # if len(ts) == 0:
    #     return []
    # Find the center of each xyxy box and also the height and width
    centroids   = []
    hw_list     = []

    for box in data:
        xc = int((box[0] + box[2]) / 2)
        yc = int((box[1] + box[3]) / 2)
        # Resize the centroids to the original image size
        xc = int(xc * (original_size[1] / 640))
        yc = int(yc * (original_size[0]/480))

        hw_list.append([box[2] - box[0], box[3] - box[1]])
        print("hw_list: ", box[2] - box[0], box[3] - box[1])

        centroids.append([xc, yc, yolo_model.names[box[5]], [box[2] - box[0], box[3] - box[1]], box[4]])

    return centroids

def get_depth(cv_depth, rgb, xc, yc, hw):
    image_size = rgb.shape


    # resize the centroids from image size to 640x480
    xc = int(xc * 640 / image_size[1])
    yc = int(yc * 480 / image_size[0])


    # get the depth of the center pixel and its 8 neighbors
    depth = cv_depth[yc, xc]
    depth += cv_depth[yc - 1, xc]
    depth += cv_depth[yc + 1, xc]
    depth += cv_depth[yc, xc - 1]
    depth += cv_depth[yc, xc + 1]
    depth += cv_depth[yc - 1, xc - 1]
    depth += cv_depth[yc - 1, xc + 1]
    depth += cv_depth[yc + 1, xc - 1]
    depth += cv_depth[yc + 1, xc + 1]
    depth /= 9

# Function that detects color block in image
def detect_color_block(image, depth, color, color_info):
    # Convert ROS image to OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')

    # undistort the image using color camera info
    # camera_info_K = np.array(color_info.K).reshape([3, 3])
    # camera_info_D = np.array(color_info.D)
    # cv_image = cv2.undistort(cv_image, camera_info_K, camera_info_D)
    original_size = cv_image.shape

    # print(depth)
    # convert depth image to OpenCV image
    cv_depth = cv_bridge.imgmsg_to_cv2(depth, 'passthrough')
    # undistort the image using depth camera info
    # depth_info_K = np.array(depth_info.K).reshape([3, 3])
    # depth_info_D = np.array(depth_info.D)
    # cv_depth = cv2.undistort(cv_depth, depth_info_K, depth_info_D)
    # cv_depth = cv_depth * 1000
    # conver it into colored image
    # print(np.unique(cv_depth, return_counts=True))
    cv_depth = cv2.applyColorMap(cv2.convertScaleAbs(cv_depth, alpha=0.3), cv2.COLORMAP_JET)

    depth_resize = cv2.resize(cv_depth, (640, 480))

    # resize the image
    cv_image = cv2.resize(cv_image, (640, 480))
    # cv_image = cv2.flip(cv_image, 0)
    # flip the image to match the camera

    # xyxy = run(weights="/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/yolov5/best.pt", image=cv_image, imgsz=(640, 480))
    results = yolo_model(cv_image, imgsz=640, agnostic_nms=True, iou=0.3, conf=0.2, classes=[0, 1, 2, 3, 4, 5, 6, 7, 8])[0]
    boxes = results.boxes.cpu().numpy()
    data = boxes.data
    
    # ts = tracker.update(data, cv_image)                     # update tracker
    # print("Tracker", ts)
    # if ts.shape[0] != 0:
    #     xyxys = ts[:, 0:4].astype('int')
    #     ids = ts[:, 4].astype('int')
    #     confs = ts[:, 5]
    #     clss = ts[:, 6]
    #     for xyxy, id, conf, cls in zip(xyxys, ids, confs, clss):
    #         cv_image = cv2.rectangle(
    #             cv_image,
    #             (xyxy[0], xyxy[1]),
    #             (xyxy[2], xyxy[3]),
    #             color,
    #             thickness
    #         )
    #         cv2.putText(
    #             cv_image,
    #             f'{id} {conf:.2f} {cls}',
    #             (xyxy[0], xyxy[1] - 5),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             fontscale,
    #             color,
    #             thickness
    #         )

    # Draw the xyxy box on the image
    for box in data:
        cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        cv2.putText(cv_image, yolo_model.names[box[5]], (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # put the class score on the image
        # cv2.putText(cv_image, str(box[4]), (int(box[0]), int(box[1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


        # shift the bounding box by 21 pixels to the right and draw it on depth image
        cv2.rectangle(depth_resize, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
        cv2.putText(depth_resize, yolo_model.names[box[5]], (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # put the class score on the image
        cv2.putText(depth_resize, str(box[4]), (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    # write a frame to the video
    out.write(cv_image)

    # if len(ts) == 0:
    #     return []
    # Find the center of each xyxy box and also the height and width
    centroids   = []
    hw_list     = []

    for box in data:
        xc = int((box[0] + box[2]) / 2)
        yc = int((box[1] + box[3]) / 2)
        # Resize the centroids to the original image size
        xc = int(xc * (original_size[1] / 640))
        yc = int(yc * (original_size[0]/480))

        # plot the points where we get the depth on the depth frame
        cv2.circle(depth_resize, (int(xc * 640 / original_size[1]), int(yc * 480 / original_size[0])), 20, (0, 0, 255), 2)
        # cv2.circle(depth_resize, (xc - 1, yc), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc + 1, yc), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc, yc - 1), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc, yc + 1), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc - 1, yc - 1), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc - 1, yc + 1), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc + 1, yc - 1), 2, (255, 0, 0), 2)
        # cv2.circle(depth_resize, (xc + 1, yc + 1), 2, (255, 0, 0), 2)

        hw_list.append([box[2] - box[0], box[3] - box[1]])
        print("hw_list: ", box[2] - box[0], box[3] - box[1])

        centroids.append([xc+5, yc, yolo_model.names[box[5]], [box[2] - box[0], box[3] - box[1]], box[4]])

        # Show image
        cv2.imshow("Image", cv_image)
        cv2.imshow("Depth", depth_resize)
        cv2.waitKey(1)

    return centroids

# Function that converts azure image to cv2 image
def azure_image_to_cv2(image):
    cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')

    # resize the image
    cv_image = cv2.resize(cv_image, (640, 480))

    return cv_image

# Function that finds the ratio between the flower bounding box and the image
def find_ratio(image_size, hw_bounding_box, method='area'):
    if method == 'area':
        # Find the area of the image
        image_area = image_size[0] * image_size[1]
        image_area_delta = image_area + 1600
        # Find the area of the bounding box
        bounding_box_area = hw_bounding_box[0] * hw_bounding_box[1]

        # Find the ratio between the image and the bounding box
        ratio = image_area / bounding_box_area
        ratio_delta = image_area_delta / bounding_box_area

        # size of bounding box
        size_bbox = bounding_box_area

    elif method == 'chessboard':
        # find if the width is long or height
        ind = np.argmax(hw_bounding_box)
        if hw_bounding_box[0] > hw_bounding_box[1]:
            side_bbox = hw_bounding_box[0]
            side_image = image_size[1]
        else:
            side_bbox = hw_bounding_box[1]
            side_image = image_size[0]
        side_bbox_delta = side_bbox + 40

        # find the ratio between the image and the bounding box
        ratio = side_image / side_bbox
        ratio_delta = side_image / side_bbox_delta

        # size of bounding box
        size_bbox = hw_bounding_box[ind]

    elif method == 'diagonal':
        print("diagonal..................................................................................")
        image_diagonal = np.sqrt(image_size[0]**2 + image_size[1]**2)
        image_diagonal_delta = image_diagonal + 40
        # Find the area of the bounding box
        bounding_box_diagonal = np.sqrt(hw_bounding_box[0]**2 + hw_bounding_box[1]**2)

        # Find the ratio between the image and the bounding box
        ratio = image_diagonal / bounding_box_diagonal
        ratio_delta = image_diagonal_delta / bounding_box_diagonal

        # size of bounding box
        size_bbox = bounding_box_diagonal
        # 


    return ratio, size_bbox, ratio_delta

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
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # print the exception
            print(e)
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


# Function that gets the 3d position of the camera with base frame
def get_camera_position(camera_frame, base_frame, tf_buffer):
    # Initialize transform
    transform = None

    # Wait for transform
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform(base_frame, camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # Return transform
    return transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z

# Function that gets the 3d pose of the camera with base frame
def get_camera_pose(camera_frame, base_frame, tf_buffer):
    # Initialize transform
    transform = None

    # Wait for transform
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform(base_frame, camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    
    print("Transform", transform)

    # Return transform
    return transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z



# Function that gets the transform between two frames
def get_transform(frame1, frame2, tf_buffer):
    # Initialize transform
    transform = None

    # Wait for transform
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform(frame1, frame2, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # Return transform
    return transform


def save_success_image(image, path):
    # Convert ROS image to OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(image, 'bgr8')
    original_size = cv_image.shape

    # resize the image
    cv_image = cv2.resize(cv_image, (640, 480))
    # cv_image = cv2.flip(cv_image, 0)
    # flip the image to match the camera

    # xyxy = run(weights="/home/vision/catkin_ws/src/xarm_ros/xarm_vision/camera_demo/scripts/yolov5/best.pt", image=cv_image, imgsz=(640, 480))
    results = yolo_model(cv_image, imgsz=640, agnostic_nms=True, iou=0.3, conf=0.2, classes=[0, 1, 2, 3, 4, 5, 6, 7, 8])[0]
    boxes = results.boxes.cpu().numpy()
    data = boxes.data
    
    
    # ts = tracker.update(data, cv_image)                     # update tracker
    # print("Tracker", ts)
    # if ts.shape[0] != 0:
    #     xyxys = ts[:, 0:4].astype('int')
    #     ids = ts[:, 4].astype('int')
    #     confs = ts[:, 5]
    #     clss = ts[:, 6]
    #     for xyxy, id, conf, cls in zip(xyxys, ids, confs, clss):
    #         cv_image = cv2.rectangle(
    #             cv_image,
    #             (xyxy[0], xyxy[1]),
    #             (xyxy[2], xyxy[3]),
    #             color,
    #             thickness
    #         )
    #         cv2.putText(
    #             cv_image,
    #             f'{id} {conf:.2f} {cls}',
    #             (xyxy[0], xyxy[1] - 5),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             fontscale,
    #             color,
    #             thickness
    #         )

    # Draw the xyxy box on the image
    for box in data:
        cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        cv2.putText(cv_image, yolo_model.names[box[5]], (int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # put the class score on the image
        cv2.putText(cv_image, str(box[4]), (int(box[0]), int(box[1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # save the cv2 image to 
    cv2.imwrite(path, cv_image)