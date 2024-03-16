# 3D-BIN-PACKING
This project uses the Franka Emika Panda arm to solve a 3D bin packing problem which is an optimization challenge that involves efficiently packing a set of items of different sizes into a container, while minimizing wasted space and maximizing space utilization. It uses computer vision to detect the dimension and location of the object needed to be packed, and it uses Moveit2 to plan the trajectories.
## VIDEO DEMO

[Video](https://www.youtube.com/embed/CVzJDIRWqrI)

## OBJECT DETECTION
Detecting the dimension of the object and finding the precise location of the project are the keys in this project. A realsense D435 is mounted on the robot. The object was detected and tracked using the RGB camera data and depth data provided by the Intel RealSense camera. All potential objects are a red color, and their location is determined using color masking in OpenCV to isolate the red pixels in the camera’s view. A contour was drawn around the red area, and the centroid of the contour and four more points on the edges were found. Then the grasp position and orientation of the object were found.
The object will be placed on the “bin” and the robot will move to the observe position first. Once the camera detects an object appears, the robot will move to the top of the object to make sure the object is at the center of the camera to better detect the dimension of the object.

## GRASPING
In order to finish grasping/placing process accurately and reliably every time, a custom gripper was designed, as well as the shape of the objects.
The grasping process include 3 steps

Move to the observe position

Move to the checking position (camera is on the top of the object)

Move to the actual position of the object and grasp it.

A custom wrapper interface was used in controlling the robot during both grasping and placing. The purpose of the wrapper interface was to make implantation easier; it offers a simpler way of planning trajectories. The wrapper was write in Making Pour Over Coffee with a Robot Arm project

## PACKING
Here are some constraints in this project: 1. All objects are rectangular. 2. the high is the same. 3. The size of the container is 9cm*9cm*10cm. 4. the dimension of the incoming object is unknown, and the order is random. 5. The object can be placed wherever as long as it is in the bin. For this project, Best-fit algorithm was used to solve this 3D rectangular packing problem. Its input is a list of items of different sizes, the output the the location of the item to place. The best-fit algorithm uses the following heuristic:

* It keeps a list of open bins, which is initially empty.

* When an item arrives, it finds the bin with the maximum load into which the item can fit, if any. The load of a bin is defined as the sum of sizes of existing items in the bin before placing the new item.

[algorithm](https://github.com/JihaiZhao/Winter-project/assets/99274626/e4339a79-82e5-4b08-babe-ccb37b7ddc13)

