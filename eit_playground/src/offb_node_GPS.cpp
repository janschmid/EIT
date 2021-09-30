#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

///////////////////////////////////////////////////////////////
// kmlClass
struct coordinate{
	double latitude;
	double longitude;
	double altitude;
};

class KMLClass{
public:
	std::vector<struct coordinate> extract_coordinates(std::string name="/home/user/eit_ws/src/eit_playground/src/MissionPlan.kml"){
		std::vector<struct coordinate>coordinates;
		struct coordinate crd;
		std::string tempString;
		size_t pos;
		int i = 0;
		std::string line;
		std::ifstream missionFile (name);
		if(missionFile.is_open()){
			while(getline(missionFile, line)){
				if(line.rfind("Lat:", 0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					crd.latitude = std::stod(tempString);
					i++;
				}else if(line.rfind("Lon:",0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					crd.longitude = std::stod(tempString);
					i++;
				}else if(line.rfind("Alt AMSL:",0)==0){
					//Above mean sea level - no nothing for now
				}else if(line.rfind("Alt Rel:",0)==0){
					pos = line.find_first_of(' ');
					tempString = line.substr(pos+1);
					pos = tempString.find_first_of(' ');
					tempString = tempString.substr(pos+1);
					crd.altitude = std::stod(tempString);
					i++;
				}
				if(i==3){
					coordinates.push_back(crd);
					i=0;
				}
			}
		}
		return coordinates;
	}
};

/////////////////////////////////////////////////////////////////////////////////

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

double cur_global_lat = 0.0;
double cur_global_lon = 0.0;
double cur_global_alt = 0.0;
void global_pos_cb(sensor_msgs::NavSatFix msg){
	cur_global_lat = msg.latitude;
	cur_global_lon = msg.longitude;
	cur_global_alt = msg.altitude;
}

mavros_msgs::Waypoint create_waypoint(double lat, double lon, double alt, bool isCurrent=false){
	mavros_msgs::Waypoint waypoint;
	waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	waypoint.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
	waypoint.is_current = isCurrent;
	waypoint.autocontinue = true;
	waypoint.x_lat = lat;
	waypoint.y_long = lon;
	waypoint.z_alt = alt;
	waypoint.param1 = 0;	//Hold time in waypoint
	waypoint.param3 = 0;	//0 to pass through waypoint, positive to orbit clockwise, negative to orbit counter-clockwise. 

	return waypoint;
}

/*
*	Get current location from GPS before takeoff
*	Use the current location as first waypoint
*	Create mission in QGC and use mission for this waypoint mission
*/


int main(int argc, char **argv){

	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;

	//Subscribe to topics
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state/", 10, state_cb);
	ros::Subscriber global_pos_cur = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, global_pos_cb);
	
	//Publish to topics
	// - Nothing yet	

	//Mode & Command selector
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	ros::ServiceClient waypoint_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");


	//Extract waypoints from KLM
	std::vector<struct coordinate>coordinates;
	KMLClass kml;
	coordinates = kml.extract_coordinates();
	
	//Create waypoint list
	mavros_msgs::WaypointPush waypoints;
	
	ros::Duration(1.0).sleep();

	//Add waypoints to list
	/*waypoints.request.waypoints.push_back(create_waypoint(cur_global_lat, cur_global_lon, cur_global_alt, true));
	waypoints.request.waypoints.push_back(create_waypoint(cur_global_lat+0.2, cur_global_lon+0.2, cur_global_alt+0.2));
	waypoints.request.waypoints.push_back(create_waypoint(cur_global_lat+0.4, cur_global_lon+0.4, cur_global_alt+0.4));
	waypoints.request.waypoints.push_back(create_waypoint(cur_global_lat+0.6, cur_global_lon+0.6, cur_global_alt+0.6));*/
	
	waypoints.request.waypoints.push_back(create_waypoint(cur_global_lat, cur_global_lon, cur_global_alt, true));
	for(int i =0; i<coordinates.size();i++){
		waypoints.request.waypoints.push_back(create_waypoint(coordinates[i].latitude, coordinates[i].longitude, coordinates[i].altitude));
	}

	
	//Update rate
	ros::Rate rate(20.0);

	while(ros::ok() && current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

	// Arm copter
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	if(arming_client.call(arm_cmd) && arm_cmd.response.success){
		ROS_INFO("Drone armed");
	}
	

	mavros_msgs::SetMode auto_pilot_mode;
	auto_pilot_mode.request.custom_mode = "AUTO.MISSION";
	//Send waypoint list to copter
	if(waypoint_client.call(waypoints)){
		ROS_INFO("Sending waypoint list to copter OK: %d", waypoints.response.success);
		if(current_state.mode != "AUTO.MISSION"){
			if(set_mode_client.call(auto_pilot_mode)){
				ROS_INFO("Auto mission enabled");
			}
		}
	} else{
		ROS_ERROR("Failed to send waypoint list to copter");
	}

	int i = 0;
	while(ros::ok()){
		//Infinite loop, do something
		ROS_INFO("%f %f %f", cur_global_lat, cur_global_lon, cur_global_alt);
		ros::spinOnce();
		rate.sleep();
	}
	
return 0;
}
