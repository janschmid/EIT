#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <ctime>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define N 4
#define TIMING 0.5

bool mission = true;
bool startLanding = true;
bool running = true;
bool threadSaysLandingEnded = false;
bool SIMULATION = true;
double rotatedCam[3];

tf2::Quaternion arucoQuaternion;

//Callbacks
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double localPos[3];
double localOrient[4];
void local_pos_cb(geometry_msgs::PoseStamped msg){
	localPos[0] = msg.pose.position.x;
	localPos[1] = msg.pose.position.y;
	localPos[2] = msg.pose.position.z;
	localOrient[0] = msg.pose.orientation.x;
	localOrient[1] = msg.pose.orientation.y;
	localOrient[2] = msg.pose.orientation.z;
	localOrient[3] = msg.pose.orientation.w;
}

double camPos[3];
double camOrient[4];
bool frameSeen = false;
void cam_pos_cb(geometry_msgs::PoseStamped msg){
	camPos[0] = msg.pose.position.x;
	camPos[1] = msg.pose.position.y;
	camPos[2] = msg.pose.position.z;
	camOrient[0] = msg.pose.orientation.x;
	camOrient[1] = msg.pose.orientation.y;
	camOrient[2] = msg.pose.orientation.z;
	camOrient[3] = msg.pose.orientation.w;

	arucoQuaternion = tf2::Quaternion(camOrient[0], camOrient[1], camOrient[2], camOrient[3]);
	/*arucoQuaternion[0] = camOrient[0];
	arucoQuaternion[1] = camOrient[1];
	arucoQuaternion[2] = camOrient[2];
	arucoQuaternion[3] = camOrient[3];*/
	

	if(msg.header.frame_id ==  "aruco_marker"){
		frameSeen = true;
		mission=false;
	}else{
		frameSeen = false;
	}
}
// End callbacks

geometry_msgs::PoseStamped waypoint(double xRel, double yRel, double zRel){
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = xRel;
	pose.pose.position.y = yRel;
	pose.pose.position.z = zRel;
	return pose;
}

void cam_rotation(double deg, double *vect){
	double RMat[3][3] = {{cos(deg),-sin(deg),0},{sin(deg),cos(deg),0},{0,0,1.0}};
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			rotatedCam[i] += RMat[i][j]*vect[j];
		}
	}
}

// For matrix mathematics
void getCofactor(double mat[N][N], double temp[N][N], int p, int q, int n){
    int i = 0, j = 0;
    // Looping for each element of the matrix
    for (int row = 0; row < n; row++){
        for (int col = 0; col < n; col++){
            //  Copying into temporary matrix only those
            //  element which are not in given row and
            //  column
            if (row != p && col != q){
                temp[i][j++] = mat[row][col];
                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1){
                    j = 0;
                    i++;
                }
            }
        }
    }
}
 
/* Recursive function for finding determinant of matrix.
   n is current dimension of mat[][]. */
double determinant(double mat[N][N], int n){
    double D = 0; // Initialize result
    //  Base case : if matrix contains single element
    if (n == 1){
        return mat[0][0];
	}
    double temp[N][N]; // To store cofactors
    int sign = 1; // To store sign multiplier
    // Iterate for each element of first row
    for (int f = 0; f < n; f++){
        // Getting Cofactor of mat[0][f]
        getCofactor(mat, temp, 0, f, n);
        D += sign * mat[0][f] * determinant(temp, n - 1);
        // terms are to be added with alternate sign
        sign = -sign;
    }
    return D;
}
// End for matrix calculations

//Calculate the altitude with a spline
double getZ(double t, double a[4]){
	return (1*a[0]+t*a[1]+t*t*a[2]+t*t*t*a[3]);
}

void altitudeControlThread(double *localPos, double *camPos, double zEnd, ros::ServiceClient sm, geometry_msgs::PoseStamped *waypoint){
	double t = 10;
	double B[4][4] = {{1,0,0,0},{0,1,0,0},{1,t,t*t,t*t*t},{0,1,2*t,3*t*t}};
	double BA11 = B[1][1]*B[2][2]*B[3][3]+B[1][2]*B[2][3]*B[3][1]+B[1][3]*B[2][1]*B[3][2] -B[1][3]*B[2][2]*B[3][1]-B[1][2]*B[2][1]*B[3][3]-B[1][1]*B[2][3]*B[3][2];
	double BA12 = -B[0][1]*B[2][2]*B[3][3]-B[0][2]*B[2][3]*B[3][1]-B[0][3]*B[2][1]*B[3][2] +B[0][3]*B[2][2]*B[3][1]+B[0][2]*B[2][1]*B[3][3]+B[0][1]*B[2][3]*B[3][2];
	double BA13 = B[0][1]*B[1][2]*B[3][3]+B[0][2]*B[1][3]*B[3][1]+B[0][3]*B[1][1]*B[3][2] -B[0][3]*B[1][2]*B[3][1]-B[0][2]*B[1][1]*B[3][3]-B[0][1]*B[1][3]*B[3][2];
	double BA14 = -B[0][1]*B[1][2]*B[2][3]-B[0][2]*B[1][3]*B[2][1]-B[0][3]*B[1][1]*B[2][2] +B[0][3]*B[1][2]*B[2][1]+B[0][2]*B[1][1]*B[2][3]+B[0][1]*B[1][3]*B[3][3];
	
	double BA21 = -B[1][0]*B[2][2]*B[3][3]-B[1][2]*B[2][3]*B[3][0]-B[1][3]*B[2][0]*B[3][2] +B[1][3]*B[2][2]*B[3][0]+B[1][2]*B[2][0]*B[3][3]+B[1][0]*B[2][3]*B[3][2];
	double BA22 = B[0][0]*B[2][2]*B[3][3]+B[0][2]*B[2][3]*B[3][0]+B[0][3]*B[2][0]*B[3][2] -B[0][3]*B[2][2]*B[3][0]-B[0][2]*B[2][0]*B[3][3]-B[0][0]*B[2][3]*B[3][2];
	double BA23 = -B[0][0]*B[1][2]*B[3][3]-B[0][2]*B[1][3]*B[3][0]-B[0][3]*B[1][0]*B[3][2] +B[0][3]*B[1][2]*B[3][0]+B[0][2]*B[1][0]*B[3][3]+B[0][0]*B[1][3]*B[3][2];
	double BA24 = B[0][0]*B[1][2]*B[2][3]+B[0][2]*B[1][3]*B[2][1]+B[0][3]*B[1][0]*B[2][2] -B[0][3]*B[1][2]*B[2][1]-B[0][2]*B[1][0]*B[2][3]-B[0][0]*B[1][3]*B[2][2];
	
	double BA31 = B[1][0]*B[2][1]*B[3][3]+B[1][1]*B[2][3]*B[3][0]+B[1][3]*B[2][0]*B[3][1] -B[1][3]*B[2][1]*B[3][0]-B[1][1]*B[2][0]*B[3][3]-B[1][0]*B[2][3]*B[3][1];
	double BA32 = -B[0][0]*B[2][1]*B[3][3]-B[0][1]*B[2][3]*B[3][0]-B[0][3]*B[2][0]*B[3][1] +B[0][3]*B[2][1]*B[3][0]+B[0][1]*B[2][0]*B[3][3]+B[0][0]*B[2][3]*B[3][1];
	double BA33 = B[0][0]*B[1][1]*B[3][3]+B[0][1]*B[1][3]*B[3][0]+B[0][3]*B[1][0]*B[3][1] -B[0][3]*B[1][1]*B[3][0]-B[0][1]*B[1][0]*B[3][3]-B[0][0]*B[1][3]*B[3][1];
	double BA34 = -B[0][0]*B[1][1]*B[2][3]-B[0][1]*B[1][3]*B[2][0]-B[0][3]*B[1][0]*B[2][1] +B[0][3]*B[1][1]*B[2][0]+B[0][1]*B[1][0]*B[2][3]+B[0][0]*B[1][3]*B[2][1];
	
	double BA41 = -B[1][0]*B[2][1]*B[3][2]-B[1][1]*B[2][2]*B[3][0]-B[1][2]*B[2][0]*B[3][1] +B[1][2]*B[2][1]*B[3][0]+B[1][1]*B[2][0]*B[3][2]+B[1][0]*B[2][2]*B[3][1];
	double BA42 = B[0][0]*B[2][1]*B[3][2]+B[0][1]*B[2][2]*B[3][0]+B[0][2]*B[2][0]*B[3][1] -B[0][2]*B[2][1]*B[3][0]-B[0][1]*B[2][0]*B[3][2]-B[0][0]*B[2][2]*B[3][1];
	double BA43 = -B[0][0]*B[1][1]*B[3][2]-B[0][1]*B[1][2]*B[3][0]-B[0][2]*B[1][0]*B[3][1] +B[0][2]*B[1][1]*B[3][0]+B[0][1]*B[1][0]*B[3][2]+B[0][0]*B[1][2]*B[3][1];
	double BA44 = B[0][0]*B[1][1]*B[2][2]+B[0][1]*B[1][2]*B[2][0]+B[0][2]*B[1][0]*B[2][1] -B[0][2]*B[1][1]*B[2][1]-B[0][1]*B[1][0]*B[2][2]-B[0][0]*B[1][2]*B[2][1];
	double BAdjuncate[4][4] = {{BA11, BA12, BA13, BA14},{BA21, BA22, BA23, BA24},{BA31, BA32, BA33, BA34},{BA41, BA42, BA43, BA44}};

	double pInvB[4][4];
	double d = determinant(B, 4);
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			pInvB[i][j] = (1/d)*BAdjuncate[i][j];
		}
	}
	double alpha[4];
	double initVal[4] = {localPos[2],0,zEnd,0};	//Starting altitude + velocity + ending altitude + velocity
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			alpha[i] += pInvB[i][j]*initVal[j];
		}
	}
	
	(*waypoint).pose.position.z = localPos[2];	//I know I know this is not necessary...
	double zNow = localPos[2];
	double timer = 0;
	clock_t startTime = clock();	//Start timer
	while(running){
		//std::cout << std::abs(camPos[0]+camPos[1]) << "Campos : \n";
		if(std::abs(camPos[0]+camPos[1]) < 0.05 && frameSeen == true){
			//std::cout << localPos[2]-zNow << "\n";
			if(std::abs(localPos[2]-zNow) <= 0.10 & (clock()-startTime)/CLOCKS_PER_SEC > TIMING){
				timer += 1.0;
				zNow = getZ(timer, alpha);
				(*waypoint).pose.position.z = zNow;	//I know I know this is not necessary...
				startTime = clock();
				std::cout << zNow << " landing\n";
				std::cout << localPos[2] << " Current altitude\n";
			}
		}	
		if(localPos[2] <= zEnd+0.05 || timer > 10){
			//Let thread end when landing occurs
			std::cout << "End landing\n";
			running = false;
    		mavros_msgs::SetMode land_set_mode;
    		land_set_mode.request.custom_mode = "AUTO.LAND";
			sm.call(land_set_mode); 
			threadSaysLandingEnded = true;
			break;
		}
	}
}

void orintControl(geometry_msgs::PoseStamped *waypoint, double *curOrient, double *newOrient){

	double QR[4];
	QR[0] = newOrient[0]*curOrient[0];
	QR[1] = newOrient[1]*curOrient[1];
	QR[2] = newOrient[2]*curOrient[2];
	QR[3] = newOrient[3]*-curOrient[3];
	QR[0] = QR[0]*curOrient[0];
	QR[1] = QR[1]*curOrient[1];
	QR[2] = QR[2]*curOrient[2];
	QR[3] = (QR[3]*curOrient[3])*-1;
	double mag = sqrt(pow(QR[0],2)+pow(QR[1],2)+pow(QR[2],2));
	(*waypoint).pose.orientation.x = QR[0]/mag;
	(*waypoint).pose.orientation.y = QR[1]/mag;
	(*waypoint).pose.orientation.z = QR[2]/mag;
	(*waypoint).pose.orientation.w = QR[3]/mag;
}

void posControl(geometry_msgs::PoseStamped *waypoint, double *curPos, double fX, double fY){
	double kp = 1;
	double roll, pitch, yaw;
    tf2::Matrix3x3(arucoQuaternion).getRPY(roll, pitch, yaw);
	
	double R[2][2] = { {cos(yaw),-sin(yaw)}, {sin(yaw),cos(yaw)} };
	double camRx = fX*R[0][0] + fY*R[0][1];
	double camRy = fX*R[1][0] + fY*R[1][1];

	
	//(*waypoint).pose.position.x = (curPos[0]+camRx)*kp;
	//(*waypoint).pose.position.y = (curPos[1]+camRy)*kp;
	

	(*waypoint).pose.position.x = (curPos[0]+fY*-1)*kp;
	(*waypoint).pose.position.y = (curPos[1]+fX*-1)*kp;
	//(*waypoint).pose.position.z = curPos[2];
}

void altControl(geometry_msgs::PoseStamped *waypoint, double *curPos, double *camPos, ros::ServiceClient sm){
	(*waypoint).pose.position.z = curPos[2]-10;
	if(camPos[2] <= 0.5){
    	mavros_msgs::SetMode land_set_mode;
    	land_set_mode.request.custom_mode = "AUTO.LAND";
		sm.call(land_set_mode);
		threadSaysLandingEnded = true;	
	}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber local_cam_pos = nh.subscribe<geometry_msgs::PoseStamped> ("aruco_pose", 10, cam_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	//Subscribe to the position topic
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pos_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    std::thread landingThread;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	//Variables
	int i = 0;
	double R = 0.3;			//30cm radius
	double landingR = 0.3; //1cm radius
	double filterX = 0.0, filterY = 0.0;
	double degree = 90.0;
	
	//waypoint array
	std::vector<geometry_msgs::PoseStamped> waypoints;
	waypoints.push_back(waypoint(0,0,2));
	waypoints.push_back(waypoint(1,1,2));
	waypoints.push_back(waypoint(2,2,2));
	waypoints.push_back(waypoint(3,3,2));

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(waypoints[0]);
        ros::spinOnce();
        rate.sleep();
    }

    // OFFBOARD mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // AUTO.LAND mode
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    // ARM command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time ts_start = ros::Time::now();

    ROS_INFO(">> Starting the offb_node");
	geometry_msgs::PoseStamped targetWaypoint;
   

	while(ros::ok()){
        if(SIMULATION == true){
			if( current_state.mode != "OFFBOARD" &&
            	(ros::Time::now() - last_request > ros::Duration(1))){
            	ROS_INFO("Trying to enable Offboard...");
            	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                	ROS_INFO(">> Offboard enabled");
            	}
            	last_request = ros::Time::now();
        	} else {
            	if( !current_state.armed &&
                	(ros::Time::now() - last_request > ros::Duration(1))){
                	ROS_INFO("Trying to arm...");
                	if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    	ROS_INFO(">> Vehicle armed");
                	}
                	last_request = ros::Time::now();
            	}
        	}
		}
		// Run waypoint mission + landing if no marker is seen.
		if(mission == true){
			if(std::abs(localPos[0]-waypoints[i].pose.position.x)+std::abs(localPos[1]-waypoints[i].pose.position.y)+std::abs(localPos[2]-waypoints[i].pose.position.z) < R){
				i++;
				std::cout << "Next waypoint\n";
			}
			if(i>waypoints.size()-1){
				i = waypoints.size()-1;
				mission = false;
				targetWaypoint.pose.position.x = localPos[0];
				targetWaypoint.pose.position.y = localPos[1];
				targetWaypoint.pose.position.z = localPos[2];
				local_pos_pub.publish(waypoints[i]);
				std::cout << "Last waypoint reached\n";
				continue;
			}	
        	local_pos_pub.publish(waypoints[i]);
		
		// Run guided landing with spline if marker is seen. 
		}else{
			filterX = (filterX+camPos[0])/2;
			filterY = (filterY+camPos[1])/2;
			if(filterX < 0.01) filterX = 0.0;
			if(filterY < 0.01) filterY = 0.0;
			std::cout << camPos[0] << " " << camPos[1] << "\n";
						
			posControl(&targetWaypoint, localPos, filterX, filterY);
			//cam_rotation(degree, camPos);
			//altControl(&targetWaypoint, localPos, camPos, set_mode_client);	
			if(startLanding == true){
				//orintControl(&targetWaypoint, localOrient, camOrient);
				landingThread = std::thread(altitudeControlThread, localPos, camPos, 0.0, set_mode_client, &targetWaypoint);
				startLanding = false;
			}
			local_pos_pub.publish(targetWaypoint);
		}
		
		if(threadSaysLandingEnded == true){
			landingThread.join();
			break;
		}
		ros::spinOnce();
		rate.sleep();
    //End og game-loop
	}
    ROS_INFO("Done");

    return 0;
}
