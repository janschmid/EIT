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


void posControl(geometry_msgs::PoseStamped *waypoint, double *curPos, double fX, double fY){
	(*waypoint).pose.position.x = (curPos[0]+fY);
	(*waypoint).pose.position.y = (curPos[1]+fX);
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
			posControl(&targetWaypoint, localPos, camPos[0], camPos[1]);
			targetWaypoint.pose.position.z = 2;
			local_pos_pub.publish(targetWaypoint);
			std::cout << targetWaypoint.pose.position.z <<"\n";
			
		}
		
		ros::spinOnce();
		rate.sleep();
    //End og game-loop
	}
    ROS_INFO("Done");

    return 0;
}
