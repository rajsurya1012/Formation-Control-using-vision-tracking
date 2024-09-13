
# Formation Control Using Vision Tracking

**Authors:** Raj Surya, Santo Santhosh

## Objective

The main objective of this project is to implement formation control using vision tracking techniques.

## Methodology

### Environment Setup
- Establish the environment for image processing and control.

### Image Processing
1. **Raw Image Acquisition**
   - Capture raw images for processing.
2. **Red Grayscaling**
   - Convert images to grayscale focusing on the red channel.
3. **Gaussian Blur**
   - Apply Gaussian blur to smooth the images.
4. **Thresholding**
   - Use thresholding to isolate relevant features.

### Robot Pose Estimation
- **Homography Update**
  - Update the homography for perspective transformations.
- **Contouring and Area Thresholding**
  - Detect contours and apply area thresholds for object detection.
- **Center Calculation**
  - Calculate the center of detected objects for tracking.

### PD Control Plant
- Implement a Proportional-Derivative (PD) control system for maintaining formation.

### Follower State Update
- Continuously update the state of follower robots based on new image data.


### Limitations
- Speed constraints
- Difficulty with sharp turns
- Occlusions and similar objects causing tracking errors
- Limited


## Setup Required to run

1. ROS 1 Noetic
2. Open CV
3. Python 3
4. Gazebo
5. Teleop Twist keyboard package

## Steps to run

1. Run `catkin_make` in your workspace.
2. Run `roslaunch multi_robot main.launch `, this should open the environment in gazebo.
3. In a new terminal, run `rosrun multi_robot image.py`.
4. In a new terminal run, `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot1/cmd_vel`.
5. By using the keys, you should be able to control the robot and see the other robots following its motion.
