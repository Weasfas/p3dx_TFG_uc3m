#include "p3dx_nav/ServerClientPelea.h"
#include "../include/Movimiento.h"
#include "../include/Nav_waypoints.h"
#include "../include/Move_base_mov.h"
#include <stdlib.h>
#include <sys/wait.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>

bool run(p3dx_nav::ServerClientPelea::Request  &req,
         p3dx_nav::ServerClientPelea::Response &res)
{

	time_t t = time(0);
	struct tm * timeStruct = localtime(&t); 

	ofstream logFile;
 	logFile.open ("../catkin_ws/server_log.txt", ios::app);
  	logFile << "Server starts at " << (timeStruct->tm_year+1900) << '-' << (timeStruct->tm_mon) << '-'<<  (timeStruct->tm_mday) << '\t' << (timeStruct->tm_hour) << ':' << (timeStruct->tm_min) << ':' << (timeStruct->tm_sec) << std::endl;

	ros::NodeHandle nh;

	Movimiento moving_alone(nh);
	Nav_waypoints navigation(nh);
	Move_base_mov nav ("Goal reached!!", "ERROR - Goal not reached!!");
	


  int tarea = req.codigo_tarea;

 /* if(tarea==0){
	
	pid_t pid;

	switch(pid = fork()){
		case 0:
  		int b = system("rosrun p3dx_nav enable_motors"); 
   		exit(1); 
	}
       int resultado = 1;
	if(resultado){
 		res.feedback =  std::string("Tarea - mover finalizada34567890...");
   		res.eval = 1;
	}else{
		res.feedback =  std::string("Tarea - mover no se ha podido finalizar...");
   		res.eval = 0;
	}
	//a = system("rosrun rosaria_client spin_clockwise"); 
	int a = system("rosnode kill /enable_motors "); 
*/

  if(tarea==0){

	logFile << "Server received task  --> Code:0     Enabling motors...\n";
	//ROS_INFO_STREAM("Enabling motors...");

 	std::system("rosservice call /RosAria/enable_motors");

	res.feedback =  std::string("Task 0 - Motors are enabled!\n");

 	//ROS_INFO_STREAM("Motors are enabled!");

  }else if(tarea==1){ //Moverse hacia delante

	float speed = atof (req.vector_mover[0].c_str());
    //ROS_INFO("%f",speed);
	float distance = atof (req.vector_mover[1].c_str());
    //ROS_INFO("%f",distance);
	int isForward = atoi (req.vector_mover[2].c_str());
    //ROS_INFO("%d",isForward);

	logFile << "Server received task  --> Code:1     Ref: Move linear "<< speed << " m/s " << distance << " m " << isForward << " direction\n";

	bool resultado = moving_alone.avanza(speed, distance, isForward);
	
		if(resultado){
			res.feedback =  std::string("Task 1 - Move ended succesfully...");
			res.eval = 1;
		}else{
			res.feedback =  std::string("Task 1 - Move ERROR...");
			res.eval = 0;
		}
   
   //ROS_INFO("Vector_mover[0] --> %d, Vector_mover[1] --> %d", req.vector_mover[0], req.vector_mover[1]);
  }else if(tarea==2){ // Girar
	
	float speed = atof(req.vector_girar[0].c_str());
	float degrees = atof(req.vector_girar[1].c_str());
	float cloakw = atoi(req.vector_girar[2].c_str());

	logFile << "Server received task  --> Code:2     Ref: Twist robot "<< speed << " deg/s " << degrees << " sex. grades " << cloakw << " direction\n";

	bool resultado = moving_alone.gira(speed, degrees, cloakw);

		if(resultado){
			res.feedback =  std::string("Task 2 - Twist ended succesfully...");
			res.eval = 1;
		}else{
			res.feedback =  std::string("Task 2 - Twist ERROR...");
			res.eval = 0;
		}
	
	//ROS_INFO("Vector_girar[0] --> %d, Vector_girar[1] --> %d", req.vector_girar[0], req.vector_girar[1]);
  }else if(tarea==3){

	float pos_x = atof(req.vector_x_y[0].c_str());
	float pos_y = atof(req.vector_x_y[1].c_str());
	float pos_w = atof(req.vector_x_y[2].c_str());

	//ROS_INFO("%f, %f, %f",pos_x,pos_y,pos_w);

	logFile << "Server received task  --> Code:3     Ref: Move to position "<< pos_x << " X " << pos_y << " Y " << pos_w << " grades\n";

	pos_w = pos_w/57.295779513;

	bool resultado = nav.setGlobalGoal(pos_x, pos_y, pos_w);

		if(resultado){
			res.feedback =  std::string("Task 3 - Move to Position ended successfully...");
			res.eval = 1;
		}else{
			res.feedback =  std::string("Task 3 - Move to Position ERROR...");
			res.eval = 0;
		}

	}else if(tarea==4){
		//Up mapping

	logFile << "Server received task  --> Code:4     Ref: Init mapping service\n";

//child


	pid_t pid;

	switch(pid = fork()){
		case 0:
  		int b = system("roslaunch p3dx_nav mapping.launch"); 
   		exit(1); 
	}

	res.feedback =  std::string("Task 4 - Mapping node init successfully...");
	res.eval = 1;

	}else if(tarea==5){
		//End mapping

	logFile << "Server received task  --> Code:5     Ref: End mapping service\n";

	system("rosnode kill slam_gmapping"); 

	res.feedback =  std::string("Task 4 - Mapping node ended successfully...");
	res.eval = 1;

	}else if(tarea==6){//Init tp node

    logFile << "Server received task  --> Code:6     Ref: Teleop node\n";
	
	system("rosrun p3dx_nav teleop");

	res.feedback =  std::string("Task 6 - Teleop node ended successfully...");
	res.eval = 1;

	}else { //Tarea desconocida
		res.feedback =  std::string("Unknown task in server");
		res.eval = 0;

  		logFile.close();
    	return false;
  	}

  logFile.close();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3dx_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("p3dx_actions", run);
  ROS_INFO("Server is up!!");
  ros::spin();

  return 0;
}
