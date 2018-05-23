#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <string>

using geometry_msgs::Twist;
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Move_base_mov{

public:
	Move_base_mov(std::string good, std::string bad);
	bool setGlobalGoal(const float &x, const float &y, const float &angle);

private:
	std::string g;
	std::string b;	
};
