#include "../include/Movimiento.h"

void Movimiento::getPose(const nav_msgs::Odometry::ConstPtr& odom){
	odometry=*odom;		
}

void Movimiento::stop(){
	ROS_INFO("Odometria init -> x=%f", odometry.twist.twist.linear.x);
	ROS_INFO("Odometria init -> z=%f", odometry.twist.twist.angular.z);
	ROS_INFO("Parando el robot...");

	while(odometry.twist.twist.linear.x!=0.0f || odometry.twist.twist.angular.z!=0.0f){
		Twist vel_stop;
		vel_stop.linear.y = 0.0f;
		vel_stop.linear.z = 0.0f;
		vel_stop.angular.x = 0.0f;
		vel_stop.angular.y = 0.0f;	
		vel_stop.angular.z = 0.0f;
		publicadorVelocidad.publish(vel_stop);
		ros::spinOnce();
	}

	ROS_INFO("El robot se ha parado");
	ROS_INFO("Odometria fin -> x=%f", odometry.twist.twist.linear.x);
	ROS_INFO("Odometria fin -> z=%f", odometry.twist.twist.angular.z);
	
}

Movimiento::Movimiento(ros::NodeHandle &n){
	ROS_INFO("Inicialización...");

	publicadorVelocidad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
	subscriptorVelocidad = n.subscribe("RosAria/pose", 10, &Movimiento::getPose, this);

	ROS_INFO("Publicador OK");
}

bool Movimiento::gira(double speed, double degrees, int clockwise){

	double radians=(degrees/360.0f)*2*M_PI;

	ROS_INFO("Rotando %f radianes", radians);
	
	while(radians < 0) radians += 2*M_PI;
    while(radians > 2*M_PI) radians -= 2*M_PI;

    //wait for the listener to get the first message
    listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = speed;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    bool done = false;
    //send the drive command
    
    
    while (!done && n.ok()){
		publicadorVelocidad.publish(base_cmd);
		ros::spinOnce();

        //get the current transform
        try{
            listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            break;
        }

        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
      
        double angle_turned = relative_transform.getRotation().getAngle();		

        if(fabs(angle_turned) < 1.0e-2) continue;

        if(actual_turn_axis.dot( desired_turn_axis ) < 0)
		    angle_turned = 2 * M_PI - angle_turned;

        if(angle_turned > radians) {
            ROS_INFO("Angulo final: %f", angle_turned);
            done=true;
        }
		
		ros::spinOnce();
    }

    stop();

    if (done) return true;
    return false;
}

bool Movimiento::avanza(float speed, float distance, int isForward){

	speed=fabs(speed);

	ROS_INFO("Iniciando movimiento...");
	
	if (isForward){
		speed=speed;
	}
	else if (!isForward){
		speed=-speed;
	}else{
   		stop();
    	ROS_INFO_STREAM("Parámetro de dirección incorrecto");
		exit(EXIT_FAILURE);
    }
		
	Twist vel;

	listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;
	listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    
    vel.linear.y = 0.0f;
	vel.linear.z = 0.0f;
	vel.angular.x = 0.0f;
	vel.angular.y = 0.0f;
	vel.angular.z = 0.0f;
	vel.linear.x = speed;
		
	ros::Rate rate(10.0);
    bool done = false;

    
    while(!done && n.ok()){

		publicadorVelocidad.publish(vel);
		ros::spinOnce();

		//get the current transform
		try{
			listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			break;
		}
		
		//see how far we've traveled
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();
		
		if(fabs(dist_moved) > pow(fabs(distance), 0.50f)){
			ROS_INFO("Objetivo cerca - disminuyendo velocidad");
			vel.linear.x=speed*0.1f;
			vel.linear.y = 0.0f;
			vel.linear.z = 0.0f;
			vel.angular.x = 0.0f;
			vel.angular.y = 0.0f;
			vel.angular.z = 0.0f;
			publicadorVelocidad.publish(vel);
			
		}
		
		if(fabs(dist_moved) > fabs(distance)){
            ROS_INFO("Distancia final: %f", dist_moved);
			done = true;
		}
		ros::spinOnce();
	}

	stop();
	
	if (done) return true;
    return false;
}
