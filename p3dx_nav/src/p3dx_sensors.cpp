#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "tf/tf.h"
#include <p3dx_nav/Sensors.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <rosaria/BumperState.h>
#include <iomanip> // for std :: setprecision and std :: fixed

bool motorState, frontBump, rearBump;
double battery_state, posX, posY, posW, velinX, velinY, velinZ, velanX, velanY, velanZ;

int battery_msg_count = 0, bumper_msg_count = 0;

/* checks pose messages and outputs them to user */
void poseMessageReceived(const nav_msgs::Odometry& msg) 
{

    posX = msg.pose.pose.position.x;
	posY = msg.pose.pose.position.y;

    tf::Quaternion q (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w);

	double yaw, pitch, roll;
	tf::Matrix3x3 mat(q);

	mat.getEulerYPR(yaw, pitch, roll);

	posW = yaw*57.2957795f;

}

/* output the state of the bumpers using rosaria */
void bumperStateMessageReceived(const rosaria::BumperState &msg)
{
	int front_size, rear_size;    
    if (bumper_msg_count == 0)
    {
//    	ROS_INFO_STREAM("The front bumpers are "<<msg.front_bumpers<<std::endl<<"The rear bumpers are "<<msg.rear_bumpers);
    	front_size = sizeof(msg.front_bumpers) / sizeof(bool);
    	rear_size = sizeof(msg.rear_bumpers) / sizeof(bool);
    	//ROS_INFO_STREAM("The front bumpers state are('1' means good): ");
    	for (int i=0;i<front_size;i++)
	  if (msg.front_bumpers[i])
    	    frontBump=true;
	  else
	    frontBump=false;
    	//ROS_INFO_STREAM("The rear bumpers state are('1' means good): ");

    	for (int i=0;i<rear_size;i++)
	  if (msg.rear_bumpers[i])
	    rearBump=true;
	  else
	    rearBump=false;
    	++bumper_msg_count;
    }
}

/* check the status of the battery charge */
/*void batteryStateOfChargeMessageReceived(const std_msgs::Float32 msg)
{
	// Right now this feature is not included in the pioneer-3 robot	
  	ROS_INFO_STREAM("The battery state of charge is "<<msg);
}*/

/* check + output the voltage of the battery using standard output */
void batteryVoltageMessageReceived(const std_msgs::Float64 msg)
{  
    if (battery_msg_count == 0)
    {
      battery_msg_count++;
      battery_state = msg.data;
    }
}

/*void batteryChargeStateMessageReceived(const std_msgs::Int8 msg)
{
	// Right now this feature is not included in the pioneer-3 robot
	ROS_INFO_STREAM("The battery charge state message received is "<< msg);
}*/

/* check the state of the motor and output using standard output */
void motorsStateMessageReceived(const std_msgs::Bool msg)
{
	if (msg.data)
		motorState = true;
	else
		motorState = false;
}

/* call all of the functions implemented above and provide user with robot state info */
int main(int argc, char **argv)
{
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "p3dx_sensor"); 
	ros::NodeHandle nh;

	// Create a subscriber object .
	ros::Subscriber pose, bumper_state, battery_state_of_charge, battery_voltage, battery_charge_state, motors_state;
	ros::Publisher myPub = nh.advertise<p3dx_nav::Sensors>("/p3dx_sensors", 1000);

	pose = nh.subscribe("RosAria/pose", 1000, &poseMessageReceived) ; //supply pose
	bumper_state = nh.subscribe("RosAria/bumper_state", 1000, &bumperStateMessageReceived) ; //inform bumper state
	/*battery_state_of_charge = nh.subscribe("RosAria/bumper_state_of_charge", 1000, &batteryStateOfChargeMessageReceived) ; //inform state of charge*/
	battery_voltage = nh.subscribe("RosAria/battery_voltage", 1000, &batteryVoltageMessageReceived) ; //inform battery voltage level
	/*battery_charge_state = nh.subscribe("RosAria/battery_charge_state", 1000, &batteryChargeStateMessageReceived) ; //inform charge state*/
	motors_state = nh.subscribe("RosAria/motors_state", 1000, &motorsStateMessageReceived) ; //inform motor state
	
	

	

	ros::Rate loop_rate(1);

	while(ros::ok()){

	p3dx_nav::Sensors msg;

   	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";

 	msg.motors_state = motorState;
	msg.frontBumpers = frontBump;
	msg.rearBumpers = rearBump;

    msg.battery_state = battery_state;
	msg.posX= posX;
	msg.posY= posY;
	msg.posW= posW;

	myPub.publish(msg);




/*printf("%s", motorState ? "Estado de los motores: Bien\n" : "Estado de los motores: Mal\n");

	printf("%s", frontBump ? "Estado de los front Bumpers: Bien\n" : "Estado de los front Bumpers: Mal\n");
	printf("%s", rearBump ? "Estado de los rear Bumpers: Bien\n" : "Estado de los rear Bumpers: Mal\n");

	std::cout << "Voltage de la bateria " << battery_state << "\n";
	
	std::cout << std::setprecision(2) << std::fixed << /* output the pose information using standard output 
	  "Current position=(" << posX << ", " << posY << ") " << 
	  "Current direction=" << std::setprecision(2) << std::fixed << posW<<"\r";
	std::flush(std::cout);*/

	// Let ROS take over.
	ros::spinOnce();

	loop_rate.sleep();
	}

	return 0; 
}
