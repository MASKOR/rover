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
#include <errno.h>

#define CAN_BAUD_1M 0x0014 //1 Mbit/s
#define CAN_BAUD_500K 0x001C // 500 kBit/s
#define CAN_BAUD_250K 0x011C // 250 kBit/s
#define CAN_BAUD_125K 0x031C // 125 kBit/s
#define CAN_BAUD_100K 0x432F // 100 kBit/s
#define CAN_BAUD_50K 0x472F // 50 kBit/s
#define CAN_BAUD_20K 0x532F // 20 kBit/s
#define CAN_BAUD_10K 0x672F // 10 kBit/s
#define CAN_BAUD_5K 0x7F7F //5 kBit/s

#define MAX_STEERING_ANGLE 25
#define THROTTLE_ZERO 2000
#define THROTTLE_MAX 3600


class CmdVelToCAN {
 public:
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
  {
    //ROS_INFO("CMD_VEL MESSAGE RECEIVED");
     
    yaw_ = -vel_cmd.angular.z * (float)MAX_STEERING_ANGLE;
    
    //project throttle from [-1:1] to [THROTTLE_ZERO:THROTTLE_MAX]
    double m = (THROTTLE_MAX - THROTTLE_ZERO) / 1.0;
    double x = vel_cmd.linear.x;

    if (x >= 0.05) {
      forward=true;
      reverse=false;
    }

    if(x < -0.05) {
      forward=false;
      reverse=true;
      x = (-1.0) * x; //don't send negative values to quad motor!
    }
    
    double b = THROTTLE_ZERO;
    throttle_ = m * x + b ;
  }

  void _publish()
  {
    ROS_INFO("_publish()");
    printf("yaw: %f\n", yaw_);
    printf("throttle: %f\n" , throttle_);
    yaw_IEEE754 = pack754(yaw_, 32, 8);
    throttle_IEEE754 = pack754(throttle_, 32 , 8 );
    // std::cout << "yaw ieee754: " << "0x" <<std::hex << yaw_IEEE754 << std::endl;
    // std::cout << "throttle ieee754: " << "0x" <<std::hex << throttle_IEEE754 << std::endl;
    // std::cout << "zero ieee754: " << "0x" <<std::hex << zero_IEEE754 << std::endl;
    // std::cout << "one ieee754: " << "0x" <<std::hex << one_IEEE754 << std::endl;

    std::cout << "forward: " << forward << std::endl;
    std::cout << "reverse: " << reverse << std::endl;
    std::cout << "reverse_on: " << reverse_on << std::endl;
    
    
    if(forward) {
      if(reverse_on) {
        ROS_WARN_NAMED("test","writing forward throttle msg");
        if(!CAN_Write(h_, &normal_throttle_msg_)) {
          ROS_INFO("Success");
          reverse_on=false;
          usleep(250000);
        }
        else
          ROS_ERROR("ERROR");     
      }
    }
       
    if(reverse) {
      if(!reverse_on) {
        ROS_ERROR("writing reverse throttle msg...\n");
        if(!CAN_Write(h_, &reverse_throttle_msg_)) { 
          ROS_INFO("Success");
          reverse_on=true;
          usleep(250000);
        }
        else
          ROS_ERROR("ERROR");     
      }
    }
         
    //steering msg
    can_steering_msg_.ID = 2;
    can_steering_msg_.MSGTYPE = MSGTYPE_STANDARD;    
    can_steering_msg_.LEN = sizeof(yaw_IEEE754);
    for(int i=0 ; i<can_steering_msg_.LEN; i++ )
      {
	can_steering_msg_.DATA[i] = (yaw_IEEE754 >> (8*i)) & 0xff;
      }
  
    //throttle msg
    can_throttle_msg_.ID = 1;
    can_throttle_msg_.MSGTYPE = MSGTYPE_STANDARD;
    can_throttle_msg_.LEN = sizeof(throttle_IEEE754);
    for(int i=0 ; i<can_throttle_msg_.LEN; i++ )
      {
	can_throttle_msg_.DATA[i] = (throttle_IEEE754 >> (8*i)) & 0xff;
      }

    //clear CAN status
    CAN_Status(h_);

    std::cout << "writing steering msg..." << std::endl;
   
    if(!CAN_Write(h_, &can_steering_msg_))
      ROS_INFO("Success");
    else
      ROS_ERROR("ERROR");
    
    std::cout << "writing throttle msg..." << std::endl;
     
    if(!CAN_Write(h_, &can_throttle_msg_))
      ROS_INFO("Success");
    else
      ROS_ERROR("ERROR");
  
    std::cout << "i'm finished" << std::endl;

  }

  //constructor
  CmdVelToCAN() :
      yaw_(0.0),
      throttle_(2000.0),
      reverse_on(false),
      forward(true),
      reverse(false)
  {
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    nh_.param<std::string>("can_device", can_device_, "/dev/pcanusb0");

    cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &CmdVelToCAN::cmd_vel_callback, this);
    
    errno=0;
    
    char baudrate[7] = "0x0014"; //1 Mbit/s
    //char baudrate[7] = "0x001C"; //500 kbit/s
    //char baudrate[7] = "0x011C"; //250 kbit/s

    std::cout << "here scheppert dat" << std::endl;
    int wBTR0BTR1 = (int)strtoul(baudrate, NULL, 16);
    int nExtended = 0;
    
    std::cout << wBTR0BTR1 << std::endl;
    std::cout << nExtended << std::endl;
    
    h_ = LINUX_CAN_Open(can_device_.c_str(), O_RDWR);
    if (!h_) {
      ROS_ERROR("CMD_VEL_TO_CAN: can't open %s\n", can_device_.c_str());
      return;
    }
    else {
      ROS_INFO("CMD_VEL_TO_CAN: succesfully opened %s\n", can_device_.c_str());
    }
    
    /* clear status */
    CAN_Status(h_);

    errno = CAN_Init(h_, wBTR0BTR1, 0);
    std::cout << "bla bla" << std::endl;

    //prepare reverse / normal throttle msgs
    one_IEEE754 = 1;
    zero_IEEE754 = 0;
    
    reverse_throttle_msg_.ID = 3;
    reverse_throttle_msg_.MSGTYPE = MSGTYPE_STANDARD;
    reverse_throttle_msg_.LEN = 1;
    reverse_throttle_msg_.DATA[0] = one_IEEE754;

    normal_throttle_msg_.ID = 3;
    normal_throttle_msg_.MSGTYPE = MSGTYPE_STANDARD;
    normal_throttle_msg_.LEN = 1;
    normal_throttle_msg_.DATA[0] = zero_IEEE754;

    ros::Rate r(20);

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
  uint64_t throttle_IEEE754;
  uint64_t one_IEEE754;
  uint64_t zero_IEEE754;
  bool forward, reverse;
  bool reverse_on;
  
  //CAN specific stuff (from PEAK Systems SDK)
  int nType_ = HW_USB;
  HANDLE h_;
  TPCANMsg can_steering_msg_;
  TPCANMsg can_throttle_msg_;
  TPCANMsg reverse_throttle_msg_;
  TPCANMsg normal_throttle_msg_;
  

  //converst double value to IEEE754 float
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
    ros::init(argc, argv, "cmd_vel_to_can");
    //Create an object of class that will take care of everything
    CmdVelToCAN p;
    while(ros::ok()) {
      ros::spin();
    }
  return 0;
}
