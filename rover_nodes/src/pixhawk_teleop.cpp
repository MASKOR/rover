/*
  ROS teleop node for the pixhawk autopilot

  subscribes to joy topic and sends Command Velocity msg to pixhawk via mavros
  
  Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
  Modified by Christian Schnieder, April 27th 2015
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <mavros/ActuatorControl.h>
#include <mavros/CommandBool.h>
#include <string>
#include <math.h>
//#include <mavros/mavros_plugin.h>
//namespace mavplugin;

class PixhawkTeleop {
 public:
  //constructor
  PixhawkTeleop() :
      yaw_(0.0),
      throttle_(0.0),
      last_joy_msg_received_(0)
  {
    nh_.param<std::string>("joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    nh_.param<int>("deadman_button", deadman_button_, 5);
    nh_.param<int>("speed_button", speed_button_, 7);
    nh_.param<int>("throttle_axis" , throttle_axis_ , 0);
    nh_.param<int>("steering_axis" , steering_axis_ , 3);
    nh_.param<double>("throttle_scale" , throttle_scale_ , 0.25);
    nh_.param<double>("steering_scale" , steering_scale_ , 1.0);   
    nh_.param<double>("joy_msg_timeout" , joy_msg_timeout_ , 0.1);
    nh_.param<bool>("steering_wheel" , steering_wheel_ , false);
    nh_.param<int>("gear_1" , gear_1_ , 12);
    nh_.param<int>("gear_2" , gear_2_ , 13);
    nh_.param<int>("gear_3" , gear_3_ , 14);
    nh_.param<int>("gear_4" , gear_4_ , 15);
    nh_.param<int>("gear_5" , gear_5_ , 16);
    nh_.param<int>("gear_6" , gear_6_ , 17);
    nh_.param<int>("gear_reverse" , gear_reverse_ , 18);    
    
    joy_sub_ = nh_.subscribe(joy_topic_, 1, &PixhawkTeleop::_joy_cb, this); 
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    ROS_INFO("***** LOADING PIXHAWK_TELEOP *****");
    ROS_INFO("joy_topic: %s", joy_topic_.c_str());
    ROS_INFO("cmd_vel_topic: %s", cmd_vel_topic_.c_str());
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
  ros::Publisher cmd_vel_pub_;
  std::string joy_topic_;
  std::string cmd_vel_topic_;
  ros::Time last_joy_msg_received_;
  double yaw_, throttle_;
  int deadman_button_, speed_button_;
  bool steering_wheel_;
  int throttle_axis_, steering_axis_;
  int gear_reverse_, gear_1_, gear_2_, gear_3_, gear_4_, gear_5_, gear_6_ ;
  double joy_msg_timeout_;
  bool deadman, speed, arming;

  //int linear_ = 0 ;
  //int angular_ = 3;
  double l_scale_, a_scale_;
  double throttle_scale_, steering_scale_;
  

  //read joystick data
  void _joy_cb(const sensor_msgs::Joy::ConstPtr &msg) {
    //update timestamp
    last_joy_msg_received_ = ros::Time::now();
    
    //todo: read joystick data and handle deadman button
    //ROS_INFO("Deadman-Button: %d",  msg->buttons[deadman_button_]);

      throttle_ = msg->axes[throttle_axis_];
      yaw_ =  msg->axes[steering_axis_];   
      deadman = msg->buttons[deadman_button_];
      speed = msg->buttons[speed_button_];
      arming = msg->buttons[0];
      if (msg->buttons[gear_reverse_]){
	l_scale_ = -throttle_scale_;
      }else{
	l_scale_ = 1.0 * (0.3 * msg->buttons[gear_1_] + 0.4 * msg->buttons[gear_2_] + 0.5 * msg->buttons[gear_3_] + 0.6 * msg->buttons[gear_4_] + 0.7 * msg->buttons[gear_5_] + 1.0 * msg->buttons[gear_6_]);

      }
      
    //todo: add transfer function to center servos and weight the raw steering/throttle data
    
  }

  //publish actuator control msg to pixhawk
  void _publish() {

    /** Trying to run a ROS Service
    if (arming){
	ros::ServiceClient client = nh_.serviceClient<mavros::CommandBool>("/mavros/cmd/arming");
	mavos::CommandBool srv;
	srv.request.parameter = true;
	client.call(srv);

      //bool req;
      //ros::service::call("/mavros/cmd/arming", "true" , req);
      //ros::ServiceServer arming_srv;
      //arming_srv = nh_.advertiseService("arming", mavros::CommandBool, true);
    }
    **/

    geometry_msgs::Twist cmd_vel;
    //check for joy msg timeout
    if(ros::Time::now().toSec() - last_joy_msg_received_.toSec() > joy_msg_timeout_) {
      ROS_WARN_NAMED("pixhawk_teleop","[pixhawk_teleop]: no joy message received for %0.1fs, setting throttle = 0.0 and yaw = 0.0", joy_msg_timeout_);
      yaw_ = 0.0;
      throttle_ = 0.0;
    }
    else {
      
      if(steering_wheel_){
	
	// configuration for steeringwheels
	// --------------------------------
	ROS_INFO("STEERINGWHEEL IS ONLINE");

	// calculating throttle and write it to cmd_vel
	a_scale_ = steering_scale_;
	cmd_vel.linear.x = l_scale_ * (throttle_ + 1 ) * 0.5 ;
	
	// calculating yaw and write it to cmd_vel
	// making sure yaw is not biger than 1	
	double tmp_a_scale = (double) a_scale_ * yaw_; 
	if (tmp_a_scale > 1.0){
	  cmd_vel.angular.z = 1.0;
	} else {
	  cmd_vel.angular.z = tmp_a_scale;
	}
      } else {

	// configuration for joypads
	// -------------------------	
	if (deadman){
	  
	  // calculating throttle and write it to cmd_vel
	  if (speed){
	    l_scale_ = 1.0;
	    ROS_INFO("--> SPEED BUTTON PUSHED ... MEEP, MEEP");
	  } else {
	    l_scale_ = throttle_scale_;
	    ROS_INFO("--> SPEED BUTTON NOT PUSHED");
	  }
	  cmd_vel.linear.x = l_scale_ * throttle_ ;

	  // calculating yaw and write it to cmd_vel
	  // making sure yaw is not biger than 1	
	  a_scale_ = steering_scale_;
	  double tmp_a_scale = (double) a_scale_ * yaw_; 
	  if (tmp_a_scale > 1.0){
	    cmd_vel.angular.z = 1.0;
	  } else {
	    cmd_vel.angular.z = tmp_a_scale;
	  }
	  ROS_INFO("YEAH YEAH YEAH");
	
	} else {
	  ROS_INFO("Deadman-Button NOT PUSHED");
	  cmd_vel.linear.x = 0.0;
	  cmd_vel.angular.z = 0.0; 
	}
      }
    }
    //send command
    cmd_vel_pub_.publish(cmd_vel);
  }
};



int main(int argc, char *argv[])
{
  //Initiate ROS
  ros::init(argc, argv, "pixhawk_teleop");
  //ros::init(argc, argv, "thrustmaster");
  //Create an object of class that will take care of everything
  PixhawkTeleop p;

  return 0;
}
