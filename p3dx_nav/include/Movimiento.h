#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

using geometry_msgs::Twist;
using namespace std;

class Movimiento{

public:
	Movimiento(ros::NodeHandle &n);
	bool avanza(float speed, float distance , int isForward);
    bool gira(double speed, double degrees, int clockwise);
	void stop();
	tf::TransformListener listener;
	
private:
	ros::NodeHandle n;	
	ros::Publisher publicadorVelocidad;
	ros::Subscriber subscriptorVelocidad;
	nav_msgs::Odometry odometry;
	void getPose(const nav_msgs::Odometry::ConstPtr& odom);
};
