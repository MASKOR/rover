/*
  ROS teleop node for can controlled vehicles

  subscribes to joy topic and sends can msgs via libpcan
  
  Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <libpcan.h>
#include <fcntl.h>

class CmdVelToCAN {
 public:
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
  {
     throttle_ = vel_cmd.linear.x;
     yaw_ = -vel_cmd.angular.z;
  }
  void _publish()
  {

  }

  //constructor
  CmdVelToCAN() :
      yaw_(0.0),
      throttle_(0.0)
  {
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    nh_.param<std::string>("can_device", can_device_, "/dev/pcanusb0");

    cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &CmdVelToCAN::cmd_vel_callback, this);
    

    h_ = LINUX_CAN_Open(can_device_.c_str(), O_RDWR);
    if (!h_) {
      printf("transmitest: can't open %s\n", can_device_.c_str());
    }
    
    /* clear status */
    CAN_Status(h_);




    ros::Rate r(10);

    while(ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }

 }


 private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  std::string cmd_vel_topic_;
  std::string can_device_;
  double yaw_, throttle_;
  
  //CAN specific stuff (from PEAK Systems SDK)
  int nType_ = HW_USB;
  HANDLE h_;
  TPCANMsg canmsg_;
 
};



int main(int argc, char *argv[])
{
    //Initiate ROS
    ros::init(argc, argv, "can_teleop");
    //Create an object of class that will take care of everything
    CmdVelToCAN p;
    while(ros::ok()) {
      ros::spin();
    }
  return 0;
}
