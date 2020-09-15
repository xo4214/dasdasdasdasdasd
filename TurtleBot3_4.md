# Machine Learning

## Machine Learning I : object_detector_3d

**Goal**  
 Recognize an object using a chainer, one of machine learning frameworks, and calculate distance to the object by using Depth camera. (Link : chainer)

**Operating Environment**

- Ubuntu 16.04
- ROS Kinetic
- Python 2.7.16
- Intel RealSense D435

**Setup**

1. ROS Kinetic Installation : refer to wiki.ros.org

2. RealSense D435 ROS Package Installation

3. \$ sudo apt install ros-kinetic-realsense2-camera

4. Dependent Package Installation

- Install if missing pip
- \$ sudo apt install python-pip
- chainer, chainercv
- \$ pip install chainer chainercv
- ros_numpy
- \$ sudo apt install ros-kinetic-ros-numpy

5. object_detector_3d Installation

- go to catkin workspace
- \$ cd ~/catkin_ws/src
- github lfs Installation
- \$ curl -s
  https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
- \$ sudo apt install git-lfs
- object_detector_3d source, model download (use github lfs )
- \$ git clone  
  https://github.com/NobutakaShimada/object_detector_3d.git
- Compile
- \$ cd ~/catkin_ws
- \$ catkin_make

**Running (It takes more than a few seconds depending on the PC)**

1. realsense & object_detector_3d node

2. \$ roslaunch object_detector_3d run.launch

3. rviz

4. \$ roslaunch object_detector_3d rviz.launch

**Run Screen**

Left to the diagram : The output by rostopic echo /object_detection_3d is displayed

Top-right of the diagram : /object_detection_3d/result_image is displayed through rviz

If you look at the top right screen, you can see that a keyboard, a mug, a bottle, and two monitors are detected from the front. Each detection has a caption, displaying the following information.

- Object Name
- Detection Score. The value ranges in [0, 1], and reliability of the detection increases as the value approaches 1.
- 3D coordinates of the object’s center point(described later). The origin of the coordinate system is the center of the camera. The x, y, and z-axis directions are given in meters and point to right, down, and inward, respectively.

If you look at the z value, which is the distance in the depth direction among the five detection results, you can see that the value increases as the position of the object is further inside.

**ROS Node**

1. Topic

- Subscribed Topics

  § /camera/color/image_raw [sensor_msgs/Image]

  color image, used for 2D object detection

  § /camera/depth/color/points [sensor_msgs/PointCloud2]

  3D PointCloud, used for calculating 3D coordinates Requires time synchronization with the camera image above to use

- Published Topics

  § /object_detection_3d [object_detector_3d/Detection3DResult]

  § int32 num_detections  
  § Detection3D[] detections

  This topic consists of the number of the detected objects (num_detections) and detection information (detections). Detection information, Detection3D, consists of the following information.

  int32 class_id  
   string class_name  
   float32 score  
   float32 y_min  
   float32 x_min  
   float32 y_max  
   float32 x_max  
   geometry_msgs/Point position

- class_id and class_name are the classification number and name of the detected object.  
  Score means the reliability of the detection. y_min, x_min, y_max, and x_max are the coordinates of the upper left and lower right of the bounding box of the detected object. Lastly, position is the 3D coordinate location of the object.

  § /object_detection_3d/result_image  
  [sensor_msgs/Image]

  This is a result image that includes information on the detected object in the image used for detection. It is an image like the one on the above right.

- Other

  § Internal parameters of sensor_msgs.CameraInfo: /camera/color/image_raw
  § realsense2_camera.Extrinsics: External parameter for converting from coordinate system of point group to color camera coordinate system.

**Implementation details**

**1. Input Data**

- Input

  § 2D Camera Image

  § 3D PointCloud

  § Camera’s internal parameters

  § External parameter indicating the position of the camera in the PointCloud coordinate system.

- Output

  § Object’s center point coordinate in PointCloud coordinate system.

  § Type of the object

  § Detection reliability

**2. Algorithm Overview**

Extract 3D coordinates of an object following the steps below.

- (1) Detects multiple objects in the image using the Object Detector with input from the image obtained from the camera.

- (2) Find frustum from the PointCloud coordinate system for the bbox(bounding box) corresponding to each detection.

- (3) Extract subset of point contained in view frustum corresponding to each detection.

- (4) Find the coordinates of the center point for each point subset group.

- (5) Integrate 2D detection result and 3D center point coordinates into a 3D detection result.

**3. Description of each algorithm**

- (1) 2D Object detection

2D object detection refers to the detection of predefined objects contained in an image. When you input a 2D image for search, the following information is displayed.

Below are predicted values corresponding to multiple objects.

- Box surrounding the object (axis aligned bounding box, bbox)

- Type of the object

- Detection reliability

Specifically, the object detector uses SSD300 learned by MS COCO and can detect 80 kinds of objects using CNN. For a detailed detection list, refer to the list shown in the link.

chainer, python’s deep learning framework, is used for implementation and chainercv is used specifically for image processing.

- (2, 3) When extracting the point subset of the detected object, use the bbox information in the previous step. For each bbox, only points that enter the bbox from the camera point of view in PointCloud are extracted as partial PointCloud.

- (4) Center point calculation in partial PointCloud

Partial PointCloud consists of points obtained from the target, background, and shielded objects. The concentrated point representing these points is called the center point.

Although there are various definitions of the center point, the center point is defined as the center of the PointCloud from the software perspective.

However, this method uses point data without distinguishing between the extracted object and the non-object portion. This can cause that the position away from the center to be calculated as the center point depending on the shape of the object or the difference bbox. It seems better to define center point as calculating the center of PointCloud after removing the non-extracted part from the partial PointCloud.

- (5) Integration of 2D detection and 3D center point coordinates

Omitted because it is simple.

## Machine Learning II: YOLO

**Goal**

Practice recognition of objects in an ROS environment using YOLO. YOLO (You Only Look Once) is a real-time object detection system that has faster speed than other detection systems. YOLO is powered by darknet, a neural network framework that educates and runs DNN (deep neural network).

**Operating Environment**

- Ubuntu 16.04
- ROS Kinetic
- Intel RealSense D435

**Setup**

- ROS Kinetic Installation : refer to wiki.ros.org.
- RealSense D435 ROS Package Installation  
   \$ sudo apt install ros-kinetic-realsense2-camera

- darknet_ros(YOLO for ROS) Installation

  - Go to catkin workspace
  - \$ cd ~/catkin_ws/src

- Source download

  - \$ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git

- Compile

  - \$ cd ~/catkin_ws
  - \$ catkin_make -DCMAKE_BUILD_TYPE=Release

- yolo v3 weight download

  - \$ cd ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
  - \$ wget http://pjreddie.com/media/files/yolov3.weights

- Change darknet_ros settings
  - open darknet_ros/config/ros.yaml file with editor

§ \$ nano  
 ~/catkin_ws/src/darknet_ros/darknet_ros/config/ros.yaml

- Change the topic in camera_reading to /camera/color/image_raw and save  
  § subscribers:  
  §  
  § camera_reading:  
  § topic: /camera/color/image_raw  
  § queue_size: 1

**Running (Use pre-trained models)**

1. realsense

2. \$ roslaunch realsense2_camera rs_camera.launch

3. YOLO(darket_ros)

4. \$ roslaunch darknet_ros darknet_ros.launch

**Run Screen**

As in the image above, several objects are recognized simultaneously. A box is created at the boundary of the object and the name of the object is shown at the top left of the box.

**ROS Node**

**1. Topic**

- Subscribed Topics  
  § /camera/color/image_raw [sensor_msgs/Image]

Color image, used for object detection

- Published Topics

§ /darknet_ros/bounding_boxes [darknet_ros_msgs/BoundingBoxes]

This topic contains the information of the recognized object. As shown below, it consists of the header of the message, the header of the image for detection, and BoundingBox which is the detected object information..

§ Header header  
 § Header image_header  
 § BoundingBox[] bounding_boxes

The BoundingBox displaying object information is shown below.(BoundingBox.msg)

float64 probability  
 int64 xmin  
 int64 ymin  
 int64 xmax  
 int64 ymax  
 int16 id  
 string class

It consists of probability that indicates the accuracy of the detection, x and y coordinates of the boundary box on the image of the detected object, id number of the object, and class that indicates the type of the object.

§ /darknet_ros/detection_image [sensor_msgs/Image]

Result image that contains the information of the object detected in the image used for detection.

§ /darknet_ros/found_object [std_msgs/Int8]

Displays the number of objects detected.

**2. Actions**

- camera_reading [sensor_msgs::Image]
  Sends an action containing images and result values (boundary boxes of the detected objects).

**3. Parameters**

Setting parameters related to detection can be done in a file with a similar name to darknet_ros/config/yolo.yaml. ROS related parameter settings can be done in darknet_ros/config/ros.yaml.

- image_view/enable_opencv (bool)

  Turn on/off open cv viewer which shows the detection image containing the bounding box.

- image_view/wait_key_delay (int)

  wait key delay(ms) in open cv viewer

- yolo_model/config_file/name (string)

  cgf name of the networks used for detection. Load cfg with corresponding name from darknet_ros/yolo_network_config/cfg to use the program.

- yolo_model/weight_file/name (string)

  weight filename of the network used for detection. Load weights file with corresponding name from darknet_ros/yolo_network_config/weights to use the program.

- yolo_model/threshold/value (float)

  The threshold of detection algorithm. Ranges between 0 and 1.

- yolo_model/detection_classes/names (array of strings)

  Names of the objects detectable by the network.

**GPU Acceleration / speed up**

If you have NVdia GPU, using CUDA can detect several times faster than using only CPU. If CUDA is installed, it is automatically recognized in CMakeLists.txt file and compiled in GPU mode when compiling(catkin_make).

- CUDA Toolkit

**Reference Sites**

- darknet
- darknet_ros

© 2020 ROBOTIS. Powered by Jekyll & Minimal Mistakes.
