#include "../include/Move_base_mov.h"

Move_base_mov::Move_base_mov(std::string good, std::string bad){
	ROS_INFO("Inicialización...");
	g = good;
	b = bad;
}

bool Move_base_mov::setGlobalGoal(const float &x, const float &y, const float &angle)
{
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/odom";
	goal.target_pose.header.stamp = ros::Time::now();
	
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	
	ROS_INFO("Sending GLOBAL goal");
	ac.sendGoal(goal);
	
	ac.waitForResult();
	
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, the base moved 1 meter forward");
		return true;
	}
    else
    {
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		return false;
	}
}

/*int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  setGlobalGoal(1.0, 1.0, 2.0); //rosa
  //setGlobalGoal(-1.0, -1.0, 1.0);
  //setGlobalGoal(2, 3, 1.0); //invernadero
  //setGlobalGoal(2.118, -8.223, 1.0); //comau
  //setGlobalGoal(-1.073, -9.271, 1.0); //puerta principal
  
  return 0;
}*/
