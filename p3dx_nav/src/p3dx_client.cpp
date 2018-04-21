#include "ros/ros.h"
#include "p3dx_nav/ServerClientPelea.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3dx_client");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<p3dx_nav::ServerClientPelea>("p3dx_actions");
  p3dx_nav::ServerClientPelea srv;

  int tarea = atoi(argv[1]);
  srv.request.codigo_tarea = tarea;	

  if(tarea==0){ //Moverse hacia delante
    std::vector<std::string> aux;
    aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);

    srv.request.vector_mover = aux;
    ROS_INFO("Parametros introducidos: Velocidad - %s, Distancia - %s y Direccion - %s",srv.request.vector_mover[0].c_str(),srv.request.vector_mover[1].c_str(),srv.request.vector_mover[2].c_str());
  }else if (tarea==1){ //Girar
    std::vector<std::string> aux;
    aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);
	
    srv.request.vector_girar = aux;     // 0 --> izq; 1 --> der;
    ROS_INFO("Parametros introducidos: Velocidad - %s, Grados - %s y Direccion - %s",srv.request.vector_girar[0].c_str(),srv.request.vector_girar[1].c_str(),srv.request.vector_girar[2].c_str());
  }else if(tarea==2){
    std::vector<std::string> aux;
	aux.push_back(argv[2]);
    aux.push_back(argv[3]);
    aux.push_back(argv[4]);

	srv.request.vector_x_y = aux;
 	ROS_INFO("Parametros introducidos: Point X - %s, Point Y - %s y Thita - %s",srv.request.vector_x_y[0].c_str(),srv.request.vector_x_y[1].c_str(),srv.request.vector_x_y[2].c_str());
  }else{ //Tarea desconocida
    ROS_INFO("Tarea desconocida en cliente");
    return 1;
  }
  //////////////////////////////RESPUESTA //////////////////////////////
  if (client.call(srv))
  {
	ROS_INFO("----------RESPUESTA DEL SERVER--------");
    ROS_INFO("Feedback --> %s", srv.response.feedback);
    ROS_INFO("Eval --> %d", srv.response.eval);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
