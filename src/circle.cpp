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

#define PI (3.1415926)

using namespace std;



enum Mission_STATE {
    IDLE,
    TAKEOFFP1,
    TAKEOFFP2,
    HOVER,
    CICLE,
    HOVER2,
    LANDINGP1,
    LANDINGP2,
    END,
} mission_state=IDLE;

mavros_msgs::State current_state;

double uavposition_x,uavposition_y,uavposition_z;
double uav_mc_x,uav_mc_y,uav_mc_z;
double uav_lp_x,uav_lp_y,uav_lp_z;
double uav_lp_qx,uav_lp_qy,uav_lp_qz,uav_lp_qw;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void uav_mc_cb(const geometry_msgs::TransformStamped::ConstPtr& transform){
    uav_mc_x = transform->transform.translation.x;
    uav_mc_y = transform->transform.translation.y;
    uav_mc_z = transform->transform.translation.z;
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
    ros::Subscriber uavposmc_sub = nh.subscribe<geometry_msgs::TransformStamped>
            ("/mavros/vision_pose/pose", 10, uav_mc_cb);
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
    pose.pose.orientation.z=-0.707;
    pose.pose.orientation.w=0.707;

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
#define TAKEOFF_X            (0.6)
#define TAKEOFF_Y            (0.4)
#define END_TAKEOFF_Z        (1.0)
#define GROUND_Z             (0.0)
#define TAKEOFF_SPEED        (0.1)

        if(mission_state==TAKEOFFP1)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = uavposition_x;
            pose.pose.position.y = uavposition_y;
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
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                std_msgs::String msg;
                msg.data = "trigger";
                tri_start_pub.publish(msg);

                mission_state=CICLE;
                last_request = ros::Time::now();
            }
        }

#define HSPEED (0.3)//Horizon Speed m/s
#define RADIUS (0.6)
#define CENTER_X (0.0)
#define CENTER_Y (0.4)

        if(mission_state==CICLE)
        {
            static int once = 0;
            static double est_time;
            static double rot_speed;

            double secs = (ros::Time::now()- last_request).toSec();
            if(once==0)
            {
                double one_circle_time = (2*PI*RADIUS/HSPEED);
                rot_speed = 360/one_circle_time;
                est_time = 4*one_circle_time;
            }
            pose.pose.position.x = (CENTER_X + RADIUS * cos (secs*rot_speed * PI/180));
            pose.pose.position.y = (CENTER_Y + RADIUS * sin (secs*rot_speed * PI/180));
            if(secs>=est_time)
            {
                mission_state = HOVER2;
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

                mission_state=LANDINGP1;
                last_request = ros::Time::now();
            }
        }

        //PLEASE DEFINE THE LANDING PARAMETER HERE
#define LANDING_X         (0.0)
#define LANDING_Y         (0.5)
#define START_LANDING_Z   (1.0)
#define GROUND_Z_05      (-0.5)
#define LANDING_SPEED     (0.1)
        if(mission_state==LANDINGP1)
        {
            static int once = 0;
            static generalMove m_to_landing;
            if(once==0)
            {
                once = 1;
                m_to_landing.setStartTimePos(ros::Time::now().toSec(),uavposition_x,uavposition_y,uavposition_z);
                m_to_landing.setDestPos(LANDING_X,LANDING_Y,START_LANDING_Z);
            }
            m_to_landing.getsetpoint(ros::Time::now().toSec(),pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
            if(m_to_landing.finished())
            {
                mission_state = LANDINGP2;
                last_request = ros::Time::now();
                cout << "Landing P1 finished" << endl;
            }
        }
        if(mission_state==LANDINGP2)
        {
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = uavposition_x;
            pose.pose.position.y = uavposition_y;
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

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
