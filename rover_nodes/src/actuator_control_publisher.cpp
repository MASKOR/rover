/*

Copyright Marcel Stüttgen 2015 <stuettgen@fh-aachen.de>

simple ros node that will publish ActuatorControl message to /mavros/setpoint/actuator_control

 */

#include <ros/ros.h>
#include <mavros/ActuatorControl.h>



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "actuator_control_publisher");
  ros::NodeHandle nh;
  ros::Publisher actuator_controls_pub = nh.advertise<mavros::ActuatorControl>("/mavros/actuator_control", 1000);   
  ros::Rate loop_rate(10);

  double ros_roll;
  double ros_pitch;
  double ros_yaw;
  double ros_throttle;
  
  while (ros::ok())
  {
    mavros::ActuatorControl actuator_control_msg;

    nh.param<double>("ros_roll", ros_roll, 0.0);
    nh.param<double>("ros_pitch", ros_pitch, 0.0);
    nh.param<double>("ros_yaw", ros_yaw, 0.0);
    nh.param<double>("ros_throttle", ros_throttle, 0.0);

    ROS_INFO("sening actuator_control msg to /mavros/actutor_control");
    ROS_INFO("ros_roll: %f", ros_roll);
    ROS_INFO("ros_pitch: %f", ros_pitch);
    ROS_INFO("ros_yaw: %f", ros_yaw);
    ROS_INFO("ros_throttle: %f", ros_throttle);

    /*send to control group 0:
      0:roll
      1:pitch 
      2:yaw
      3:throttle
      4:flaps
      5:spoilers
      6:airbrakes
      7:landing gear

      see https://pixhawk.ethz.ch/mavlink/#SET_ACTUATOR_CONTROL_TARGET
      and https://pixhawk.org/dev/mixing
     */
    actuator_control_msg.header.stamp = ros::Time::now();
    actuator_control_msg.group_mix = 0; 
    actuator_control_msg.controls[0] = ros_roll;
    actuator_control_msg.controls[1] = ros_pitch;
    actuator_control_msg.controls[2] = ros_yaw;
    actuator_control_msg.controls[3] = ros_throttle;
    actuator_control_msg.controls[4] = 0.0;
    actuator_control_msg.controls[5] = 0.0;
    actuator_control_msg.controls[6] = 0.0;
    actuator_control_msg.controls[7] = 0.0;
    actuator_controls_pub.publish(actuator_control_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

