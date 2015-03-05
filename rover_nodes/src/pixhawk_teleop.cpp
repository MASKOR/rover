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
      throttle_(0.0),
      last_joy_msg_received_(0)
  {
    nh_.param<std::string>("joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>("actuator_controls_topic", actuator_controls_topic_, "/mavros/actuator_controls");
    nh_.param<int>("deadman_button", deadman_button_, 1);
    nh_.param<int>("throttle_axis" , throttle_axis_ , 0);
    nh_.param<int>("steering_axis" , steering_axis_ , 1);
   
    nh_.param<double>("joy_msg_timeout" , joy_msg_timeout_ , 0.1);
    
    joy_sub_ = nh_.subscribe(joy_topic_, 1, &PixhawkTeleop::_joy_cb, this); 
    actuator_control_pub_ = nh_.advertise<mavros::ActuatorControl>(actuator_controls_topic_, 1);

    ROS_INFO("***** LOADING PIXHAWK_TELEOP *****");
    ROS_INFO("joy_topic: %s", joy_topic_.c_str());
    ROS_INFO("actuator_contols_topic: %s", actuator_controls_topic_.c_str());
    ROS_INFO("deadman_button: %d", deadman_button_);
    ROS_INFO("throttle_axis: %d", throttle_axis_);
    ROS_INFO("steering_axis: %d", steering_axis_);
    ROS_INFO("joy_msg_timeout: %f", joy_msg_timeout_);
    ROS_INFO("**********************************");
    
    ros::Rate r(10);
    while(ros::ok()) {
      _publish();
      ros::spinOnce();
      r.sleep();
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher actuator_control_pub_;
  std::string joy_topic_;
  std::string actuator_controls_topic_;
  ros::Time last_joy_msg_received_;
  double yaw_, throttle_;
  int deadman_button_;
  int throttle_axis_;
  int steering_axis_;
  double joy_msg_timeout_;
  

  //read joystick data
  void _joy_cb(const sensor_msgs::Joy::ConstPtr &msg) {
    //update timestamp
    last_joy_msg_received_ = ros::Time::now();

    //todo: read joystick data and handle deadman button

    //todo: add transfer function to center servos and weight the raw steering/throttle data
  }

  //publish actuator control msg to pixhawk
  void _publish() {
    //check for joy msg timeout
    if(ros::Time::now().toSec() - last_joy_msg_received_.toSec() > joy_msg_timeout_) {
      ROS_WARN_NAMED("pixhawk_teleop","[pixhawk_teleop]: no joy message received for %0.1fs, setting throttle = 0.0 and yaw = 0.0", joy_msg_timeout_);
      yaw_ = 0.0;
      throttle_ = 0.0;
    }
    else {
      //yaw_= joystick_value
      //throttle = joystick_value

    }
    //send command
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
  
  //Create an object of class that will take care of everything
  PixhawkTeleop p;

  return 0;
}
