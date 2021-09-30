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

double local_pos_x = 0.0;
double local_pos_y = 0.0;
void local_pos_cb(geometry_msgs::PoseStamped msg){
	local_pos_x = msg.pose.position.x;
	local_pos_y = msg.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
	waypoints.push_back(pose);
    
	pose.pose.position.x = 10;
    pose.pose.position.y = 10;
    pose.pose.position.z = 2;
	waypoints.push_back(pose);

	pose.pose.position.x = 15;
    pose.pose.position.y = 15;
    pose.pose.position.z = 4;
	waypoints.push_back(pose);

	pose.pose.position.x = 15;
    pose.pose.position.y = -5;
	pose.pose.position.z = 4;
	waypoints.push_back(pose);

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
		
		if(std::abs(local_pos_x-waypoints[i].pose.position.x)+std::abs(local_pos_y-waypoints[i].pose.position.y) < R){
			i++;
		}
		if(i>waypoints.size()-1) i = waypoints.size()-1;

        local_pos_pub.publish(waypoints[i]);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Done");

    return 0;
}
