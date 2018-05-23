#include "../include/Nav_waypoints.h"
using namespace std;

double Nav_waypoints::degree2radian(double degreeAngle){
	return (degreeAngle/57.2957795);
}

double Nav_waypoints::radian2degree(double radianAngle){
	return (radianAngle*57.2957795);
}

void Nav_waypoints::stop(){
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

void Nav_waypoints::getPose(const nav_msgs::Odometry::ConstPtr& odom){
	odometry=*odom;		
}

Nav_waypoints::Nav_waypoints(ros::NodeHandle &n){
	ROS_INFO("Inicialización...");

	publicadorVelocidad = n.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
	subscriptorVelocidad = n.subscribe("RosAria/pose", 10, &Nav_waypoints::getPose, this);
    samplingRate = 0.1;

	ROS_INFO("Publicador OK");
}

/**Calculates the desired angulus to point to
	 * @param deltax the difference in x
	 * @param deltay the difference in y
	 * @return
*/
double Nav_waypoints::angulus (double deltax, double deltay){
	
	double ang;
	ang = (float) atan2(deltay,deltax) * 180/M_PI;

	//ROS_INFO("atan2 - %f", (float) atan2(deltay,deltax));
	//	if the angle is >180 we change it to be negative so it always turns in the appropriate direction
	if (ang > 180){
		ang = ang-360;
	}

		return ang;
}

/** Auxiliary function to calculate the angle and find the smallest turn to reach it
	 * @param desiredAngle
	 * @param currentAngle
	 * @return
*/
double Nav_waypoints::calculateDeltaTh(double desiredAngle, double currentAngle){
	
	double delta_tita;

	if (desiredAngle<0){
		desiredAngle=360+desiredAngle;
	}

	if (currentAngle<0){
		currentAngle=360+currentAngle;
	}

	//First-second, third and fourth behave the same, but I will let it by now
	//If the desiredAngle is in the first quadrant
	if (desiredAngle>=0 && desiredAngle<=90){
		delta_tita = desiredAngle-currentAngle;

		if (delta_tita<-180){
			delta_tita = 360+delta_tita;
		}
	} else if (desiredAngle<=180){//second quadrant
		delta_tita = desiredAngle-currentAngle;

		if (delta_tita<-180){
			delta_tita = 360+delta_tita;
		}
	} else if (desiredAngle<=270){//third quadrant
		delta_tita = desiredAngle-currentAngle;

		if (delta_tita>180){
			delta_tita = delta_tita-360;
		}
	} else {//fourth quadrant
		delta_tita = desiredAngle-currentAngle;

		if (delta_tita>180){
			delta_tita = delta_tita-360;
		}
	}	
		
	return delta_tita;

}

std::vector<double> Nav_waypoints::followTrajectory(double xActual, double yActual, double wActual, double xObj, double yObj, double wObj){

	//Defining the variables
	//The linear and rotational speed we are sending to the robot to follow the desired trajectory
	double futureLinealSpeed, futureAngularSpeed;
	//The current robot position
	double currentX, currentY, currentTh;
	//The current target position
	double targetCurrentX, targetCurrentY, targetCurrentTh;
	//The future target position (estimated). 2014-11 not used yet
	double targetFutureX, targetFutureY, targetFuturePhi;
	//The method constants: the lower the more greedy we tend to the desired position
	double kx=0.9, ky=0.9, ktita=0.8;

	std::vector<double> ret (2);

	//Giving values to the variables
	targetCurrentX = xObj;
	targetCurrentY = yObj;
	targetCurrentTh = wObj;
	//ROS_INFO("Target - %f %f %f", targetCurrentX, targetCurrentY, targetCurrentTh);

	//if the angle is negative we convert it to positive
	if (targetCurrentTh<0){
		//ROS_INFO("CAMBIO 1");
		targetCurrentTh += 360;
	}

	//These are in mm
	currentX = xActual;
	currentY = yActual;
	currentTh = wActual; //odometry.pose.pose.orientation.w;
	//ROS_INFO("Current - %f %f %f", currentX, currentY, currentTh);

	//if the angle is negative we convert it to positive
	if (currentTh<0){
		//ROS_INFO("CAMBIO 2");
		currentTh += 360;
	}

	double deltaX = targetCurrentX-kx*(targetCurrentX-currentX)-currentX;				
	double deltaY = targetCurrentY-ky*(targetCurrentY-currentY)-currentY;
	//ROS_INFO("deltaX %f, deltaY %f", deltaX, deltaY);

	//Calculating angle
	double angle = angulus(deltaX,deltaY);
	//ROS_INFO("angle - %f", angle);
	//now we can calculate the speed
	futureLinealSpeed = 1/samplingRate*(deltaX*cos(angle*M_PI/180)+deltaY*sin(angle*M_PI/180));
	//ROS_INFO("futureLinealSpeed 1 - %f", futureLinealSpeed);
	//and truncate it by the maximum speed
	futureLinealSpeed = fmin(futureLinealSpeed,1.0f);
	//ROS_INFO("futureLinealSpeed 2 - %f", futureLinealSpeed);

	//calculating the heading
	//In this first approach we use current angle instead of future one
	//The formula to calculate the desired angle is (angle-ktita*(angle-currenTh))
	//We use the calculateDeltaTh method to correctly subtract angles
	double desiredAngle = calculateDeltaTh(angle, currentTh);
	//ROS_INFO("desiredAngle 1 - %f", desiredAngle);
	desiredAngle = calculateDeltaTh(angle, ktita*desiredAngle);
	//ROS_INFO("desiredAngle 2 - %f", desiredAngle);
	double deltaTh = calculateDeltaTh (desiredAngle, currentTh);
	//ROS_INFO("deltaTh - %f", deltaTh);
	futureAngularSpeed = 0.174533*deltaTh; 
	//ROS_INFO("Future Angular speed - %f", futureAngularSpeed);

	double velx = (sqrt(pow((xObj-xActual),2) + pow((yObj-yActual),2)));
	ROS_INFO("Tutle vel x - %f", velx);
	double velth =(atan2((yObj-yActual),(xObj-xActual))-degree2radian(wActual));
	ROS_INFO("Tutle vel th - %f", velth);

	ret[0] = futureLinealSpeed/2.0f;  //*0.1;//futureLinealSpeed;
	ret[1] = futureAngularSpeed*0.2f; //*0.1;//futureAngularSpeed;

	return ret;	

}



bool Nav_waypoints::run(double x, double y, double z){

    geometry_msgs::Twist base_cmd; // Twist para mandar comandos de movimiento.
	std::vector<double> velocidades (2); // Vector para guardar velocidades.
	double radians = degree2radian(z); // Radianes introducidos.

	// Creamos las tranformada objetivo.
	target_transform.setOrigin( tf::Vector3(x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, radians);
	target_transform.setRotation(q);
	
	target_transform_final = tf::StampedTransform(target_transform, ros::Time::now(), "odom", "target");

	ROS_INFO("Target tf --> tarX: %f, tarY: %f, tarW: %f",target_transform_final.getOrigin().getX(),target_transform_final.getOrigin().getY(),target_transform_final.getRotation().getAngle());

	ros::Rate rate(10.0);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(60.0); //sec
    bool done = false;
	bool done_bad = false;

	while (!done && n.ok()){
		
		// Guardamos la transformada actual -> Estado actual.
		try{
			listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(5.0));
      		listener.lookupTransform("odom", "base_link", ros::Time(0), current_transform);
    	}catch (tf::TransformException &ex) {
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
      		continue;
    	}
	
		double currX = current_transform.getOrigin().getX();
		double currY = current_transform.getOrigin().getY();

    	tf::Quaternion start_q = current_transform.getRotation();
		double yaw, pitch, roll;
		tf::Matrix3x3 mat(start_q);
		mat.getEulerYPR(yaw, pitch, roll);

		double currW = yaw; //En radianes

		ROS_INFO("Current tf --> currX: %f, currY: %f, currW: %f", currX, currY, currW);

		// Calculamos la distancia relativa que hemos avanzado hasta el objetivo.
		tf::Transform relative_transform = target_transform_final.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();
		double angle_moved = fabs(radians-current_transform.getRotation().getAngle());
		ROS_INFO("Dist - %f, Ang - %f", dist_moved, angle_moved);

		double test = sqrt(pow((x-current_transform.getOrigin().getX()),2)+ pow((y-current_transform.getOrigin().getY()),2));

		// Si estamos muy cerca del objetivo nos paramos.
		if(test<=0.5){
			done = true;
		}else if (ros::Time::now() - start_time > timeout){ // Si han pasado x segundo asumimos que el robot no ha podido llegar al objetivo o esta "stall".
			done=true;
			done_bad=true;
		}
	
		// Calculamos las velocidades - Proportional controller.
		velocidades = followTrajectory(currX, currY, radian2degree(currW), x, y, z);

		base_cmd.linear.x = velocidades[0];
		base_cmd.angular.z = velocidades[1];
		
		// Establemos la velocidad publicando en el topic de ROS
		publicadorVelocidad.publish(base_cmd);

	    ROS_INFO("Robot en -> Px=%f Py=%f Pw=%f", currX, currY, currW);
	    ROS_INFO("Velocidad -> x=%f y=%f", velocidades[0], velocidades[1]);
		ROS_INFO("Tolerancia -> dist=%f ang=%f", dist_moved, angle_moved);
		
		rate.sleep();
    }

    stop();
	
	int idone = done ? 1 : 0;
	int idone_bad = done_bad ? 1 : 0;

	ROS_INFO("Done es: %d", idone);
	ROS_INFO("Done_bad es: %d", idone_bad);

    if (done) return true;
    return false;
	
}

/*int main(int argc, char** argv){

	ros::init(argc, argv, "Nav_waypoints");
	
	ROS_INFO("Nav_waypoints started");

	double PX = atof(argv[1]);
	double PY = atof(argv[2]);
	double Thi = atof(argv[3]);


	ros::NodeHandle nh;

	Nav_waypoints nav(nh);
	nav.run(PX, PY, Thi);

	return 0;
}*/
