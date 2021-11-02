#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <ctime>
#include <math.h>

#define N 4
#define TIMING 1.0

bool mission = true;
bool startLanding = true;
double zLanding;
bool running = true;

//Callbacks
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double localPosX = 0.0;
double localPosY = 0.0;
double localPosZ = 0.0;
void local_pos_cb(geometry_msgs::PoseStamped msg){
	localPosX = msg.pose.position.x;
	localPosY = msg.pose.position.y;
	localPosZ = msg.pose.position.z;
}

double camPosX = 0.0;
double camPosY = 0.0;
double camPosZ = 0.0;
void cam_pos_cb(geometry_msgs::PoseStamped msg){
	camPosX = msg.pose.position.x;
	camPosY = msg.pose.position.y;
	camPosZ = msg.pose.position.z;
}
// End callbacks

geometry_msgs::PoseStamped waypoint(double xRel, double yRel, double zRel){
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = xRel;
	pose.pose.position.y = yRel;
	pose.pose.position.z = zRel;
	return pose;
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

void altitudeControlThread(double t, double zStart, double zEnd){
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
	double initVal[4] = {zStart,0,zEnd,0};	//Starting altitude + velocity + ending altitude + velocity
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			alpha[i] += pInvB[i][j]*initVal[j];
		}
	}
	double zNow = 0;
	double timer = 0;
	clock_t startTime = clock();	//Start timer
	while(running){
		if((clock()-startTime)/CLOCKS_PER_SEC > TIMING){
			timer += 1.0;
			zNow = getZ(timer, alpha);
			zLanding = zNow;	//I know I know this is not necessary...
			startTime = clock();
			std::cout << "landing\n";
			if(sqrt((zEnd-zNow)*(zEnd-zNow)) < 0.05){
				//Let thread end when landing occurs
				std::cout << "End landing\n";
				running = false;
				break;
			}
		}
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
	double landingR = 0.01; //1cm radius
	//waypoint array
	std::vector<geometry_msgs::PoseStamped> waypoints;
	waypoints.push_back(waypoint(0,0,2));
	waypoints.push_back(waypoint(1,1,2));
	waypoints.push_back(waypoint(1.5,1.5,1.5));
	waypoints.push_back(waypoint(1,2,2));
	waypoints.push_back(waypoint(1,2,2));

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
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("Trying to enable Offboard...");
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO(">> Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                ROS_INFO("Trying to arm...");
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO(">> Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
		// Run waypoint mission + landing if no marker is seen.
		if(mission == true){
			if(std::abs(localPosX-waypoints[i].pose.position.x)+std::abs(localPosY-waypoints[i].pose.position.y)+std::abs(localPosZ-waypoints[i].pose.position.z) < R){
				i++;
			}
			if(i>waypoints.size()-1){
				i = waypoints.size()-1;
				mission = false;
				continue;
				//if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
				//	ROS_INFO("AUTO.LAND enabled");
				//	break;
				//}
			}	
        	local_pos_pub.publish(waypoints[i]);
		// Run guided landing with spline if marker is seen. 
		}else{
			targetWaypoint = waypoint(localPosX+camPosX, localPosY+camPosY, localPosZ);
			if(std::abs(camPosX+camPosY) < landingR){
				if(startLanding == true){
					landingThread = std::thread(altitudeControlThread, 10.0, localPosZ, 0.0);
					startLanding = false;
				}
				std::cout << zLanding << "\n";
				targetWaypoint.pose.position.z = zLanding;
			}	
			local_pos_pub.publish(targetWaypoint);
		}
        ros::spinOnce();
        rate.sleep();
    }
	if(mission == false){
    	landingThread.join();
	}
    ROS_INFO("Done");

    return 0;
}
