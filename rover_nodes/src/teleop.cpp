/*
  ROS teleop node for the pixhawk autopilot

  subscribes to joy topic and sends AcutatorControl msg to pixhawk via mavros
  
  Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros/ActuatorControl>

class PixhawkTeleop {
 private:
  ros::Subscriber joy_sub_;
  ros::Publisher actuator_control_pub_;

};



int main(int argc, char *argv[])
{
    //Initiate ROS
    ros::init(argc, argv, "pixhawk_teleop");
    //Create an object of class that will take care of everything
    PixhawkTeleop p;
    ros::spin();
  return 0;
}
