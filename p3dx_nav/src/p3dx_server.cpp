#include "p3dx_nav/ServerClientPelea.h"
#include "../include/Movimiento.h"
#include "../include/Nav_waypoints.h"
#include "../include/Move_base_mov.h"
#include <stdlib.h>
#include <sys/wait.h>
#include <cstdlib>

bool run(p3dx_nav::ServerClientPelea::Request  &req,
         p3dx_nav::ServerClientPelea::Response &res)
{

	ros::NodeHandle nh;

	Movimiento moving_alone(nh);
	Nav_waypoints navigation(nh);
	Move_base_mov nav ("Goal reached!!", "ERROR - Goal not reached!!");
	


  int tarea = req.codigo_tarea;
  
  if(tarea==0){ //Moverse hacia delante
  	
	float speed = atof (req.vector_mover[0].c_str());
    //ROS_INFO("%f",speed);
	float distance = atof (req.vector_mover[1].c_str());
    //ROS_INFO("%f",distance);
	int isForward = atoi (req.vector_mover[2].c_str());
    //ROS_INFO("%d",isForward);

	bool resultado = moving_alone.avanza(speed, distance, isForward);
	
	if(resultado){
 		res.feedback =  std::string("Tarea - mover finalizada...");
   		res.eval = 1;
	}else{
		res.feedback =  std::string("Tarea - mover no se ha podido finalizar...");
   		res.eval = 0;
	}
  
   
   //ROS_INFO("Vector_mover[0] --> %d, Vector_mover[1] --> %d", req.vector_mover[0], req.vector_mover[1]);
  }else if(tarea==1){ // Girar
	
	bool resultado = moving_alone.gira(atof(req.vector_girar[0].c_str()), atof(req.vector_girar[1].c_str()), atoi(req.vector_girar[2].c_str()));

  	if(resultado){
 		res.feedback =  std::string("Tarea - girar finalizada...");
   		res.eval = 1;
	}else{
		res.feedback =  std::string("Tarea - girar no se ha podido finalizar...");
   		res.eval = 0;
	}
	
	//ROS_INFO("Vector_girar[0] --> %d, Vector_girar[1] --> %d", req.vector_girar[0], req.vector_girar[1]);
  }else if(tarea==2){

	bool resultado = nav.setGlobalGoal(atof(req.vector_x_y[0].c_str()), atof(req.vector_x_y[1].c_str()), atof(req.vector_x_y[2].c_str()));

  	if(resultado){
 		res.feedback =  std::string("Tarea - navegar finalizada...");
   		res.eval = 1;
	}else{
		res.feedback =  std::string("Tarea - navegar no se ha podido finalizar...");
   		res.eval = 0;
	}
  }else{ //Tarea desconocida
	ROS_INFO("Tarea desconocida en servidor");
	res.eval = 0;
    return false;
  }
   
 //res.sum = req.a + req.b;
  

  
  
  
  //ROS_INFO("Feedback --> %s", res.feedback);
  //ROS_INFO("Eval --> %s", res.eval);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3dx_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("p3dx_actions", run);
  ROS_INFO("Ready to move!");
  ros::spin();

  return 0;
}
