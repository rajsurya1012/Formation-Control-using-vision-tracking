
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
   - ![Raw Image](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p1.jpg)
2. **Red Grayscaling**
   - Convert images to grayscale focusing on the red channel.
   - ![Red Grayscaling](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p6.jpg)
3. **Gaussian Blur**
   - Apply Gaussian blur to smooth the images.
   - ![Gaussian Blur](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p3.jpg)
4. **Thresholding**
   - Use thresholding to isolate relevant features.
   - ![Thresholding](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p10.jpg)

### Robot Pose Estimation
- **Homography Update**
  - Update the homography for perspective transformations.
  - ![Homography Update](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/homography_update.jpg)
- **Contouring and Area Thresholding**
  - Detect contours and apply area thresholds for object detection.
  - ![Contouring and Area Thresholding](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p5.jpg)
- **Center Calculation**
  - Calculate the center of detected objects for tracking.
  - ![Center Calculation](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p2.jpg)

### PD Control Plant
- Implement a Proportional-Derivative (PD) control system for maintaining formation.

### Follower State Update
- Continuously update the state of follower robots based on new image data.

## Results

### Motion Tests
- **Speed 3X Forward Motion**
  - ![Forward Motion](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p9.jpg)
- **Speed 3X Zig Zag Motion**
  - ![Zig Zag Motion](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p8.jpg)
- **Speed 3X Reverse Motion**
  - ![Reverse Motion](https://github.com/rajsurya1012/Formation-Control-using-vision-tracking/raw/main/Results/p7.jpg)

### Limitations
- Speed constraints
- Difficulty with sharp turns
- Occlusions and similar objects causing tracking errors
- Limited field of view and resolution issues

## Applications

- **Warehouse Automation**
- **Farm Land Management**
- **Drone Shows**
- **Exploration Missions**

## Conclusion

Thank you for reviewing this project. Please feel free to reach out with any questions or for further discussion.


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
