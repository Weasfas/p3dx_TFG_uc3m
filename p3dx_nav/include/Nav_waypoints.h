#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>

using geometry_msgs::Twist;
using namespace std;

class Nav_waypoints{

public:
	Nav_waypoints(ros::NodeHandle &n);
	double degree2radian(double degreeAngle);
	double radian2degree(double radianAngle);
	double angulus (double deltax, double deltay);
    double calculateDeltaTh(double desiredAngle, double currentAngle);
	std::vector<double> followTrajectory(double xActual, double yActual, double wActual, double xObj, double yObj, double wObj);
	bool run(double x, double y, double z);
	void stop();

	tf::TransformListener listener;

	tf::StampedTransform current_transform;
	tf::Transform target_transform;
	tf::StampedTransform target_transform_final;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad;
	ros::Subscriber subscriptorVelocidad;
	nav_msgs::Odometry odometry;
	void getPose(const nav_msgs::Odometry::ConstPtr& odom);
    double samplingRate;
    
};
