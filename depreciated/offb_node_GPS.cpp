/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 *
 * Originally from: https://dev.px4.io/master/en/ros/mavros_offboard.html
 *
 * Stack and node tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double global_pos_x = 0.0;
double global_pos_y = 0.0;
void global_pos_cb(geometry_msgs::PoseStamped msg){
	global_pos_x = msg.latitude;
	global_pos_y = msg.longitude;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	//Subscribe to the global position topic
	//The number - here 1 - is the amount of messages that we will buffer up - set to 1 so we only have the current position to process
    // topic "/mavros/global_position/global" is filtered position coordinates. 
	// topic "/mavros/global_position/raw/fix" is unfiltered
	ros::Subscriber global_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/global_position/raw/fix", 1, global_pos_cb);

	//Publish to the global setpoint topic
	//The number - here 10 - is the amount of messages that we buffer 
	ros::Publisher global_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_raw/global",10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

	//Variables
	int i = 0;
	double simDuration = 50.0;
	double R = 0.3;

	//waypoint array
	std::vector<geometry_msgs::PoseStamped> waypoints;

    geometry_msgs::PoseStamped waypoint;
    waypoint.latitude = 55.0;
    waypoint.longitude = 8.0;
    waypoint.altitude = 535.0;
	waypoints.push_back(waypoint);

    waypoint.latitude = 55.5;
    waypoint.longitude = 8.3;
    waypoint.altitude = 535.0;
	waypoints.push_back(waypoint);
    
    waypoint.latitude = 56.0;
    waypoint.longitude = 8.6;
    waypoint.altitude = 535.0;
	waypoints.push_back(waypoint);

    waypoint.latitude = 56.5;
    waypoint.longitude = 8.9;
    waypoint.altitude = 535.0;
	waypoints.push_back(waypoint);

	//send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        global_pos_pub.publish(waypoints[0]);
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
   

	while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("Trying to enable Offboard...");
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO(">> Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                ROS_INFO("Trying to arm...");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO(">> Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if( current_state.mode != "LAND" &&
            (ros::Time::now() - ts_start > ros::Duration(simDuration))){
            ROS_INFO("Trying to enable AUTO.LAND...");
            if( set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent){
                ROS_INFO("AUTO.LAND enabled");
                break;
            }
        }
		
		if(std::abs(global_pos_x-waypoints[i].latitude)+std::abs(global_pos_y-waypoints[i].longitude) < R){
			i++;
		}
		if(i>waypoints.size()-1) i = waypoints.size()-1;

        global_pos_pub.publish(waypoints[i]);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Done");

    return 0;
}
