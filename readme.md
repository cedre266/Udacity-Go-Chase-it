# Udacity-Go-Chase-It

[![Demo Video](https://img.youtube.com/vi/IW3tygQMhBc/0.jpg)](https://www.youtube.com/watch?v=IW3tygQMhBc)

Second project in Udacity's Robotics Software Engineer Nanodegree.
The goal of the project was to build a little robot from a URDF file with a camera and a lidar. The camera image is continuously processed to detect white objects. Upon detecting a white object, commands are sent automatically to the wheels so that the robot drives itself towards the white object.

Made under ROS-Noetic and Gazebo-11

## Instructions
- Clone the repository
- Then launch gazebo with the world and the spawned robot. From the root directory:
```
roslaunch my_robot world.launch
```
- Then launch the code to monitor the camera image and send services to the robot to drive it:
```
roslaunch ball_chaser ball_chaser.launch
```
- Rviz should open automatically and the camera view can be viewed from there (need to add camera and subscribe to the camera topic)
- Alternatively, the camera view can be displayed in a separate window withe the following command:
```
rosrun rqt_image_view rqt_image_view
```
