#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include "std_msgs/String.h"
#include "generalmove.h"
#include "euler_q_rmatrix.h"


#define PI (3.1415926)

using namespace std;

enum Mission_STATE {
    IDLE,
    TAKEOFFP1,
    TAKEOFFP2,
    HOVER,
    GO_STRAIGHT,
    HOVER2,
    LANDING,
    END,
} mission_state=IDLE;

mavros_msgs::State current_state;

double uavposition_x,uavposition_y,uavposition_z;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    uav_lp_x = pose->pose.position.x;
    uav_lp_y = pose->pose.position.y;
    uav_lp_z = pose->pose.position.z;
    uav_lp_qx = pose->pose.orientation.x;
    uav_lp_qy = pose->pose.orientation.y;
    uav_lp_qz = pose->pose.orientation.z;
    uav_lp_qw = pose->pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, uav_lp_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //triggle (vio platform test)
    ros::Publisher tri_start_pub = nh.advertise<std_msgs::String>("tri_start", 10);
    ros::Publisher tri_end_pub = nh.advertise<std_msgs::String>("tri_end", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);
    //wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = uav_lp_x;
    pose.pose.position.y = uav_lp_y;
    pose.pose.position.z = uav_lp_y;
    pose.pose.orientation.x=0.0;
    pose.pose.orientation.y=0.0;
    pose.pose.orientation.z=0.0;
    pose.pose.orientation.w=1.0;

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
        uavposition_x = uav_lp_x;
        uavposition_y = uav_lp_y;
        uavposition_z = uav_lp_z;

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
                    mission_state = TAKEOFFP1;
                }
                last_request = ros::Time::now();
            }
        }

        /*takeoff*****************************************************/
        //PLEASE DEFINE THE LANDING PARAMETER HERE
#define TAKEOFF_X            (0.0)
#define TAKEOFF_Y            (0.0)
#define END_TAKEOFF_Z        (1.0)
#define GROUND_Z             (0.0)
#define TAKEOFF_SPEED        (0.1)

        if(mission_state==TAKEOFFP1)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = TAKEOFF_X;
            pose.pose.position.y = TAKEOFF_Y;
            pose.pose.position.z = GROUND_Z+(secs)*TAKEOFF_SPEED;
            if(pose.pose.position.z > (END_TAKEOFF_Z-0.3))// - 0.3 to restrain the overshoot
            {
                pose.pose.position.z= END_TAKEOFF_Z-0.3;
                cout << "Takeoff P1 finished" << endl;
                mission_state = TAKEOFFP2;
                last_request = ros::Time::now();
            }
        }
        if(mission_state==TAKEOFFP2)
        {
            static int once = 0;
            static generalMove m_to_point_1;
            if(once==0)
            {
                once = 1;
                m_to_point_1.setStartTimePos(ros::Time::now().toSec(),uavposition_x,uavposition_y,uavposition_z);
                m_to_point_1.setDestPos(TAKEOFF_X,TAKEOFF_Y,END_TAKEOFF_Z);
            }
            m_to_point_1.getsetpoint(ros::Time::now().toSec(),pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
            if(m_to_point_1.finished())
            {
                mission_state = HOVER;
                cout << "Takeoff P2 finished" << endl;
                last_request = ros::Time::now();
            }
        }

        /*hover*****************************************************/
        if(mission_state==HOVER)
        {
            if(ros::Time::now() - last_request > ros::Duration(1.0))
            {
                std_msgs::String msg;
                msg.data = "trigger";
                tri_start_pub.publish(msg);

                mission_state=GO_STRAIGHT;
                last_request = ros::Time::now();
            }
        }

#define DESTINATION_X (1.0)//Horizon Speed m/s
#define DESTINATION_Y (0.0)
#define DESTINATION_Z (1.0)


        if(mission_state==GO_STRAIGHT)
        {
          static int once = 0;
          static generalMove straight_path;
          if(once==0)
          {
              once = 1;
              straight_path.setStartTimePos(ros::Time::now().toSec(),uavposition_x,uavposition_y,uavposition_z);
              straight_path.setDestPos(DESTINATION_X,DESTINATION_Y,DESTINATION_Z);
          }
          straight_path.getsetpoint(ros::Time::now().toSec(),pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
          if(straight_path.finished())
          {
              mission_state = HOVER2;
              cout << "GO_STRAIGHT P2 finished" << endl;
              last_request = ros::Time::now();
          }
        }

        if(mission_state==HOVER2)
        {
            if(ros::Time::now() - last_request > ros::Duration(2.0))
            {
                std_msgs::String msg;
                msg.data = "trigger";
                tri_end_pub.publish(msg);

                mission_state=LANDING;
                last_request = ros::Time::now();
            }
        }

        //PLEASE DEFINE THE LANDING PARAMETER HERE
#define START_LANDING_Z   (1.0)
#define GROUND_Z_05      (-0.5)
#define LANDING_SPEED     (0.1)

        if(mission_state==LANDING)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = DESTINATION_X;
            pose.pose.position.y = DESTINATION_Y;
            pose.pose.position.z = START_LANDING_Z-(secs)*LANDING_SPEED;
            if(pose.pose.position.z < GROUND_Z_05)
            {
                pose.pose.position.z=GROUND_Z_05;
                arm_cmd.request.value = false;
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    mission_state==END;
                    cout << "Landing P2 finished" << endl;
                    return 0;//break the control UAV will land automatically
                }
            }
        }

        if(mission_state==END)
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0;
            return 0;
        }
        cout  << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << endl;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
