#include "ros/ros.h"
#include "p3dx_nav/ServerClientPelea.h"
#include <stdlib.h>
#include <sys/wait.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>
#include <string.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3dx_client");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  time_t t = time(0);
  struct tm * timeStruct = localtime(&t); 

  ofstream logFile;
  logFile.open ("client_log.txt", ios::app);
  
  logFile << "Client starts at " << asctime(timeStruct) << std::endl;
//logFile << "Client starts at " << (timeStruct->tm_year+1900) << '-' << (timeStruct->tm_mon + 1) << '-'<<  (timeStruct->tm_mday) << '\t' << (timeStruct->tm_hour) << ':' << (timeStruct->tm_min) << ':' << (timeStruct->tm_sec) << std::endl;
  //logFile.close();

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<p3dx_nav::ServerClientPelea>("p3dx_actions");
  p3dx_nav::ServerClientPelea srv;

  std::string tarea = argv[1];
  srv.request.codigo_tarea = tarea;	

  if(strcmp(tarea.c_str(),"startMotor")==0){
    //Enable motors call
  }else if(strcmp(tarea.c_str(),"moveForward")==0){ //Moverse hacia delante
    std::vector<std::string> aux;
    aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);

    srv.request.vector_mover = aux;

    logFile << "Request send to server  --> Code:1     Ref: Move linear "<< srv.request.vector_mover[0].c_str() << " m/s " << srv.request.vector_mover[1].c_str() << " m " << srv.request.vector_mover[2].c_str() << " direction\n";

  }else if (strcmp(tarea.c_str(),"twist")==0){ //Girar
    std::vector<std::string> aux;
    aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);
	
    srv.request.vector_girar = aux;     // 0 --> izq; 1 --> der;
    
    logFile << "Request send to server  --> Code:2     Ref: Twist robot "<< srv.request.vector_girar[0].c_str() << " deg/s " << srv.request.vector_girar[1].c_str() << " sex. grades " << srv.request.vector_girar[2].c_str() << " direction\n";

  }else if(strcmp(tarea.c_str(),"moveTo")==0){
    std::vector<std::string> aux;
	  aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);

	srv.request.vector_x_y = aux;

	logFile << "Request send to server  --> Code:3     Ref: Move to position "<< srv.request.vector_x_y[0].c_str() << " X " << srv.request.vector_x_y[1].c_str() << " Y " << srv.request.vector_x_y[2].c_str() << " grades\n";
  
  }else if(strcmp(tarea.c_str(),"startMapping")==0){

	  logFile << "Request send to server  --> Code:4     Ref: Init mapping process\n";

  }else if(strcmp(tarea.c_str(),"endMapping")==0){

    logFile << "Request send to server  --> Code:5     Ref: Finish mapping process\n";

  }else if(strcmp(tarea.c_str(),"teleop")==0){

	  logFile << "Request send to server  --> Code:6     Ref: Init teleoperation mode\n";

  }else{//Tarea desconocida
    logFile << "Unknown task in client\n";
    logFile.close();
    return 1;
  }
  //////////////////////////////RESPUESTA //////////////////////////////
  if (client.call(srv))
  {
	  logFile << "SERVER RESPONSE --> " << srv.response.feedback << " with code " << srv.response.eval << std::endl;
  }else{
    logFile << "Failed to call service";
    return 1;
  }

  logFile.close();
  return 0;
}
