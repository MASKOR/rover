/*
  ROS teleop node for the pixhawk autopilot

  subscribes to joy topic and sends AcutatorControl msg to pixhawk via mavros
  
  Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros/ActuatorControl.h>
#include <string>

class PixhawkTeleop {
 public:
  //constructor
  PixhawkTeleop() :
      yaw_(0.0),
      throttle_(0.0)
  {
    nh_.param<std::string>("joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>("actuator_controls_topic", acutator_controls_topic_, "/mavros/actuator_controls");

    joy_sub_ = nh_.subscribe(joy_topic_, 1, &PixhawkTeleop::_joy_cb, this); 
    actuator_control_pub_ = nh_.advertise<mavros::ActuatorControl>(acutator_controls_topic_, 1);
  }

  void publish() {
    PixhawkTeleop::_publish();
  }
  
 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher actuator_control_pub_;
  std::string joy_topic_;
  std::string acutator_controls_topic_;
  double yaw_, throttle_;

  //read joystick data
  void _joy_cb(const sensor_msgs::Joy::ConstPtr &msg) {

  }

  //publish actuator control msg to pixhawk
  void _publish() {
    mavros::ActuatorControl a_c_msg;
    a_c_msg.header.stamp = ros::Time::now();
    a_c_msg.group_mix = 0;
    a_c_msg.controls[0] = 0.0;
    a_c_msg.controls[1] = 0.0;
    a_c_msg.controls[2] = yaw_;
    a_c_msg.controls[3] = throttle_;
    a_c_msg.controls[4] = 0.0;
    a_c_msg.controls[5] = 0.0;
    a_c_msg.controls[6] = 0.0;
    a_c_msg.controls[7] = 0.0;
    actuator_control_pub_.publish(a_c_msg);
  }
};



int main(int argc, char *argv[])
{
    //Initiate ROS
    ros::init(argc, argv, "pixhawk_teleop");
    ros::Rate r(10);

    printf("test\n");
    //Create an object of class that will take care of everything
    PixhawkTeleop p;
    while(ros::ok()) {
      //need to publish frequently because pixhawk will go into failsafe mode else
      p.publish();
      ros::spin();
      r.sleep();
    }
  return 0;
}
