# LIDAR car tracking
Point cloud object tracking using OpenCV, PCL and ROS

## Information
This project aims to create a robust system to track and identify cars/object. It is made to provide positional data of Connected Autonomous Vehicles/CAVs (modeled by Raspberry-Pi cars made by SunFounder) to enable traffic flow management using Artificial Intelligence.

As shown below, the robot cars are moving around a figure 8 track with one junction to navigate through. Using LIDAR data and ultrasonic sensors, the cars are able to navigate through junction without crashing.

![4 Cars Demo](https://media.giphy.com/media/fsJx47EhhA4HDu85lc/giphy.gif)

Here is a demo of the system working with 4 cars. As seen on the left side of the screen, the cars are tracked as they go around the track. Added to that, the position of the cars are published in an array in order of the car id (car1_x, car1_y, car1_z, car2_x, car2_y, etc...)

![4 Cars Tracking](https://media.giphy.com/media/XZ0kGoPJTgm52Y4oTR/giphy.gif)

Although ultrasonic sensors might sound sufficient to allow the cars to not crash. The objective of this project is to provide localisation/position data of the CAVs, which could be used for Deep Reinforcement Learning/DRL to improve efficiency of traffic flow.
**This may sound like over-engineering. However, by combining position data and DRL, the acceleration and speed of each cars could be increased/decrease in the most efficient way where the cars _don't need to stop_ to navigate through the junction**. This would not only tackle traffic congestion issues, but also improve energy efficiency. The impact would be negligible with a small-scale model with only 4 cars. In a large-scale application with thousands of cars, the impact would be a lot more noticeable. 

Here a GIF of it with 6 cars.

![6 Cars Tracking](https://media.giphy.com/media/XDXWXwmfKUJWz9w4G0/giphy.gif)

(Note: The stuttering in rviz is due to occasional packet losses as the point cloud data from the 2 LIDAR sensors are being streamed wirelessly using a Raspberry Pi)

## How to use
_It is recommended to read the thesis to understand the functionality of each code before using it_

1. Install ROS on your computer and create catkin workspace and add it to ROS environment (edit .bashrc file ~ tons of tutorial on the web)
This project was done on Linux Ubuntu (Ubuntu 18.04.4 LTS) [INSTALLATION TUTORIAL HERE](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. git clone this repository to the src folder of your catkin workspace and catkin_make
3. Customise the clustering criteria and initial positions (read thesis for more information)
4. Run scripts by rosrun lidar_car_tracking (script_name)
4. Enjoy and be creative!

## Thesis
A thesis was made for this project with analysis on consistency of tracking and identification. Also includes a more thorough elaboration on the system.

Link to thesis:
```
TBD
```

## Acknowledgements
Special thanks to Prof. Robert Piechocki, Ahmed Khalil and Lucia Cipolina-Kun for the supervision throughout this project. 

## References
1. Stanford   Artificial   Intelligence   Laboratory   et   al.,   “Robotic   operating system.” [Online]. Available:  https://www.ros.org
2. R.  B.  Rusu  and  S.  Cousins,  “3D  is  here:   Point  Cloud  Library  (PCL),”inIEEE International Conference on Robotics and Automation (ICRA),Shanghai, China, May 9-13 2011
3. P.Palanisamy, “praveen-palanisamy/multiple-object-tracking-lidar:Multiple-Object-Tracking-from-Point-Cloudsv1.0.2,′′ Dec.2019.[Online]. Available:https://doi.org/10.5281/zenodo.3559186

## Citing
If you use any piece of code from this repo or any information from the thesis, please cite:
```
TBD
```
