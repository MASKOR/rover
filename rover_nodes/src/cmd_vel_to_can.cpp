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
#include <stdlib.h>
#include <stdio.h>

class CmdVelToCAN {
 public:
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
  {
     throttle_ = vel_cmd.linear.x;
     yaw_ = -vel_cmd.angular.z;
  }
  void _publish()
  {
    printf("yaw: %f\n", yaw_);
    yaw_IEEE754 = pack754(yaw_, 32, 8);
    yaw_IEEE754_2 = yaw_IEEE754;
    std::cout << "yaw ieee754: " << "0x" <<std::hex << yaw_IEEE754 << std::endl;
    std::cout << "yaw ieee754_2: " << "0x" <<std::hex << yaw_IEEE754_2 << std::endl;
    std::cout << "sizeof(yawyaw_IEEE754_2): " << sizeof(yaw_IEEE754_2) << std::endl;
    int a = (yaw_IEEE754_2 >> (8*0)) & 0xff;
    int b = (yaw_IEEE754_2 >> (8*1)) & 0xff;
    int c = (yaw_IEEE754_2 >> (8*2)) & 0xff;
    int d = (yaw_IEEE754_2 >> (8*3)) & 0xff;
    std::cout << "int a: " << std::hex << a << std::endl;
    std::cout << "int b: " << std::hex << b << std::endl;
    std::cout << "int c: " << std::hex << c << std::endl;
    std::cout << "int d: " << std::hex << d << std::endl;

    //steering msg
    canmsg_.ID = 2;
    canmsg_.MSGTYPE = MSGTYPE_STANDARD;    
    canmsg_.LEN = sizeof(yaw_IEEE754_2);
    for(int i=0 ; i<canmsg_.LEN; i++ )
      {
	canmsg_.DATA[i] = (yaw_IEEE754_2 >> (8*(canmsg_.LEN-(i+1)) )) & 0xff;
      }
     
    CAN_Write(h_, &canmsg_);


  }

  //constructor
  CmdVelToCAN() :
      yaw_(20.0),
      throttle_(0.0)
  {
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    nh_.param<std::string>("can_device", can_device_, "/dev/pcanusb3");

    cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &CmdVelToCAN::cmd_vel_callback, this);
    

    h_ = LINUX_CAN_Open(can_device_.c_str(), O_RDWR);
    if (!h_) {
      ROS_ERROR("CMD_VEL_TO_CAN: can't open %s\n", can_device_.c_str());
    }
    else {
      ROS_INFO("CMD_VEL_TO_CAN: succesfully opened %s\n", can_device_.c_str());
    }
    
    /* clear status */
    CAN_Status(h_);




    ros::Rate r(10);

    while(ros::ok()) {
      _publish();
      ros::spinOnce();
      r.sleep();
    }

 }


 private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  std::string cmd_vel_topic_;
  std::string can_device_;
  float yaw_, throttle_;
  uint64_t yaw_IEEE754;
  uint yaw_IEEE754_2;
  
  //CAN specific stuff (from PEAK Systems SDK)
  int nType_ = HW_USB;
  HANDLE h_;
  TPCANMsg canmsg_;

uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
	long double fnorm;
	int shift;
	long long sign, exp, significand;
	unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	if (f == 0.0) return 0; // get this special case out of the way

	// check sign and begin normalization
	if (f < 0) { sign = 1; fnorm = -f; }
	else { sign = 0; fnorm = f; }

	// get the normalized form of f and track the exponent
	shift = 0;
	while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
	while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
	fnorm = fnorm - 1.0;

	// calculate the binary form (non-float) of the significand data
	significand = fnorm * ((1LL<<significandbits) + 0.5f);

	// get the biased exponent
	exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

	// return the final answer
	return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}
 
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
