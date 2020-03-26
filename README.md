# LIDAR car tracking
using OpenCV, PCL and ROS

## Information
This project aims to create a robust system to track and identify cars/object. It is made to provide positional data of Connected Autonomous Vehicles/CAVs (modeled by Raspberry-Pi cars made by SunFounder) to enable traffic flow management using Artificial Intelligence.

As shown below, the cars (could be more than 4) are moving around a figure 8 track with one junction to navigate through. Using LIDAR data and ultrasonic sensors, the cars are able to navigate through junction without crashing.

![4 Cars Demo](https://media.giphy.com/media/fsJx47EhhA4HDu85lc/giphy.gif)

Although ultrasonic sensors might sound sufficient to allow the cars to not crash. The objective of this project is to provide localisation/positional data of the CAVs, which could be used for Deep Reinforcement Learning/DRL to improve efficiency of traffic flow.
**This may sound like over-engineering. However, with positional data and AI, the acceleration and speed of each cars could be increased/decrease in the most efficient way where the cars _don't need to stop_ to navigate through the junction**. This would not only tackle traffic congestion issues (costs billions of dollars around the world), but also improve energy efficiency. The impact would be negligible with a small-scale model with only 4 cars. In a large-scale application with thousands of cars, the impact would be a lot more noticeable. 

Here a GIF of it working with 6 cars.

## How to use
_It is recommended to read the thesis to understand the functionality of each code before using it_

1. Install ROS on your computer and create catkin workspace and add it to ROS environment (edit .bashrc file ~ tons of tutorial on the web)
This project was done on Linux Ubuntu (Ubuntu 18.04.4 LTS) [INSTALLATION TUTORIAL HERE](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. git clone this repository to the src folder of your catkin workspace and catkin_make
3. rosrun any of the scripts 

## Thesis
A thesis was made for this project with analysis on consistency of tracking and identification. Also includes a more thorough elaboration on the whole system.

Link to thesis:
```
TBD
```

## Acknowledgements
Special thanks to Prof. Robert Piechocki, Ahmed Khalil and Lucia Cipolina-Kun for the supervision throughout this project. 

## References
1. Stanford   Artificial   Intelligence   Laboratory   et   al.,   “Robotic   operatingsystem.” [Online]. Available:  https://www.ros.org
2. R.  B.  Rusu  and  S.  Cousins,  “3D  is  here:   Point  Cloud  Library  (PCL),”inIEEE International Conference on Robotics and Automation (ICRA),Shanghai, China, May 9-13 2011
3. P.Palanisamy,“praveen-palanisamy/multiple-object-tracking-lidar:Multiple-Object-Tracking-from-Point-Cloudsv1.0.2,′′Dec.2019.[Online].Available:https://doi.org/10.5281/zenodo.3559186

## Citing
If you use any piece of code from this repo or any information from the thesis, please cite:
```
TBD
```
