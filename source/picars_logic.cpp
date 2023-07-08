#include "common.h"

// ROS
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// ROS publisher
ros::Publisher picarflag_pub;

// Subscriber callback
void callback(const std_msgs::Float32MultiArray::ConstPtr &cc)
{
  std::vector<geometry_msgs::Point> picar_pos;
  for(std::vector<float>::const_iterator it=(*cc).data.begin(); it!=(*cc).data.end(); it+=3) // it+=3 as data in Float32MultiArray
  {
    geometry_msgs::Point pt;
    pt.x=*it;
    pt.y=*(it+1);
    pt.z=0.0f; // will be 0 because 2D
    picar_pos.push_back(pt);
  }

  std_msgs::Int32MultiArray car_flags;
  for(auto it=picar_pos.begin(); it!=picar_pos.end(); it++)
  {
    if((*it).x >= -0.40 && (*it).x <= 0.40 && (*it).y >= 0.25 && (*it).y <= 1.05) // zone X
    {
      car_flags.data.push_back(1); // produce '1' if vehicle is inside zone X
    }
    else
    {
      car_flags.data.push_back(0);
    }
  }

  // for debugging
  std::cout << "car_flags: " << endl;
  for(auto it=car_flags.data.begin(); it!=car_flags.data.end(); it++)
  {
    std::cout << *it << endl;
  }

  picarflag_pub.publish(car_flags);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "picars_logic");
  ros::NodeHandle n;
  ros::Subscriber _sub = n.subscribe<std_msgs::Float32MultiArray>("picar_positions", 1, callback);

  picarflag_pub = n.advertise<std_msgs::Int32MultiArray>("picar_flags", 1);

  ros::spin();

  return 0;
}
