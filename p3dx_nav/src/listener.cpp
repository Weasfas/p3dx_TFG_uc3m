#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odometry;
std_msgs::Float32 charge;
std_msgs::Bool motor_sta;

void getPose(const nav_msgs::Odometry::ConstPtr& odom){
	odometry=*odom;		
}

void getMotors(const std_msgs::Bool::ConstPtr& motor){
	motor_sta=*motor;		
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  //ros::NodeHandle battery_voltage;
  //ros::NodeHandle bumpers;
  ros::NodeHandle moto;
  ros::NodeHandle pose;
  //ros::NodeHandle pose2;
  //ros::NodeHandle sonar;


  //ros::Publisher publicadorSensores;
  //ros::Subscriber battery_volt = 
  //ros::Subscriber bumps = 
  ros::Subscriber mot = moto.subscribe("RosAria/motors_state", 10, &getMotors);
  ros::Subscriber posee = pose.subscribe("RosAria/pose", 10, &getPose);

  ROS_INFO("Odometria position init -> x=%f", odometry.pose.pose.position.x);
  ROS_INFO("Odometria position init -> y=%f", odometry.pose.pose.position.y);
  ROS_INFO("Odometria position init -> z=%f", odometry.pose.pose.position.z);

  ROS_INFO("Odometria orientation init -> x=%f", odometry.pose.pose.orientation.x);
  ROS_INFO("Odometria orientation init -> y=%f", odometry.pose.pose.orientation.y);
  ROS_INFO("Odometria orientation init -> z=%f", odometry.pose.pose.orientation.z);
  ROS_INFO("Odometria orientation init -> w=%f", odometry.pose.pose.orientation.w);

  printf("%s", motor_sta.data ? "true" : "false");

  ROS_INFO("Estado de los motores -> %d", motor_sta.data);

  //ros::Subscriber sonaar = 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this
   * version, all callbacks will be called from within this thread
   * (the main one).  ros::spin() will exit when Ctrl-C is pressed,
   * or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
