#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

using namespace std;


int loop_count=0;
#define COUNT (20)

enum Mission_STATE {
    IDLE,
    TAKEOFF,
    PT1,
    PT2,
    LANDING,
} mission_state=IDLE;

mavros_msgs::State current_state;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //update the uav position
        //from motion capture system
        //local postion estrimator(in simulation platform)
        /*offboard and arm*****************************************************/
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    mission_state = TAKEOFF;
                }
                last_request = ros::Time::now();
            }
        }

        /*takeoff*****************************************************/
        //PLEASE DEFINE THE LANDING PARAMETER HERE

        if(mission_state==TAKEOFF)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.2;
            if(secs > 5.0)// - 0.3 to restrain the overshoot
            {
                cout << "Takeoff P1 finished" << endl;
                mission_state = PT1;
                last_request = ros::Time::now();
            }
        }
        if(mission_state==PT1)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = 1.0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.2;
            if(secs > 5.0)// - 0.3 to restrain the overshoot
            {
                cout << "Takeoff P1 finished" << endl;
                mission_state = PT2;
                last_request = ros::Time::now();
            }
        }
        if(mission_state==PT2)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = -1.0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.2;
            if(secs > 5.0)// - 0.3 to restrain the overshoot
            {
                loop_count++;
                cout << "Takeoff P1 finished" << endl;
                if(loop_count >= 20)
                {
                    mission_state = LANDING;
                    last_request = ros::Time::now();
                } else
                {
                    mission_state = PT1;
                    last_request = ros::Time::now();
                }
            }
            
        }

        /*hover*****************************************************/
        if(mission_state==LANDING)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.2-(secs)*0.3;
            if(pose.pose.position.z < 0)
            {
                pose.pose.position.z = 0; 
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
