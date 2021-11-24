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
double closeEnough = false;
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

void mulQuaternion(double *q2, double *q1, double *qret){
	double q[4];
	qret[0] = q[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	qret[1] = q[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
	qret[2] = q[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1];
	qret[3] = q[0]*q2[3]-q1[1]*q2[2]-q1[2]*q2[1]-q1[3]*q2[0];
}

void invQuaternion(double *q){
	q[3] = q[3]*-1;
}

void normQuaternion(double *q){
	double mag = sqrt(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2));
	q[0] /= mag;
	q[1] /= mag;
	q[2] /= mag;
	q[3] /= mag;
}

void orintControl(geometry_msgs::PoseStamped *waypoint, double *curOrient, double *camOrient){
	double qRet[4] = {0,0,0,0};
	double result[4];


	invQuaternion(curOrient);
	mulQuaternion(camOrient,curOrient,qRet);
	invQuaternion(curOrient);
	mulQuaternion(qRet,curOrient,result);
	normQuaternion(result);
	
	(*waypoint).pose.orientation.x = result[0];
	(*waypoint).pose.orientation.y = result[1];
	(*waypoint).pose.orientation.z = result[2];
	(*waypoint).pose.orientation.w = result[3];

	std::cout <<"-----------------------\n";
	std::cout << "camOrient: " << camOrient[0] << " " << camOrient[1] << " " << camOrient[2] << " " << camOrient[3] << "\n";
	std::cout << "result: " << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << "\n";	
}

void posControl(geometry_msgs::PoseStamped *waypoint, double *curPos, double fX, double fY){
	(*waypoint).pose.position.x = (curPos[0]+fY);
	(*waypoint).pose.position.y = (curPos[1]+fX);
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

void rotateDrone(double *quat, double *coord){
	double copy[3];
	double Mat[3][3];
	Mat[0][0] = 1-2*(pow(quat[1],2)+pow(quat[2],2));
	Mat[0][1] = 2*(quat[0]*quat[1]-quat[2]*quat[3]);
	Mat[0][2] = 2*(quat[0]*quat[2]-quat[1]*quat[3]);
	Mat[1][0] = 2*(quat[0]*quat[1]+quat[2]*quat[3]);
	Mat[1][1] = 1-2*(pow(quat[0],2)+pow(quat[2],2));
	Mat[1][2] = 2*(quat[1]*quat[2]-quat[1]*quat[3]);
	Mat[2][0] = 2*(quat[0]*quat[2]-quat[1]*quat[3]);
	Mat[2][1] = 2*(quat[1]*quat[2]+quat[0]*quat[3]);
	Mat[2][2] = 1-2*(pow(quat[0],1)+pow(quat[1],2));
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			copy[i] += Mat[i][j]*coord[j];
		}
	}
	coord[0] = copy[0];
	coord[1] = copy[1];
	coord[2] = copy[2];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber local_cam_pos = nh.subscribe<geometry_msgs::PoseStamped> ("aruco_pose", 1, cam_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	//Subscribe to the position topic
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pos_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(4.0);
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
	waypoints.push_back(waypoint(0,0,1));
	waypoints.push_back(waypoint(1,1,1));
	waypoints.push_back(waypoint(2,2,1));
	waypoints.push_back(waypoint(3,3,1));

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
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
			double rotation[3];
			rotation[0] = targetWaypoint.pose.orientation.x;
			rotation[1] = targetWaypoint.pose.orientation.y;
			rotation[2] = targetWaypoint.pose.orientation.z;
			rotation[3] = targetWaypoint.pose.orientation.w;
			targetWaypoint.pose.position.z = 1;
			rotateDrone(localPos, rotation);
			posControl(&targetWaypoint, localPos, camPos[0], camPos[1]);			
			//if(abs(pow(localPos[0]-camPos[0],2)+pow(localPos[1]-camPos[1],2)) < R){
			orintControl(&targetWaypoint, localOrient, camOrient);
			//}

			if(startLanding == true){
				//landingThread = std::thread(altitudeControlThread, localPos, camPos, 0.0, set_mode_client, &targetWaypoint);
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
