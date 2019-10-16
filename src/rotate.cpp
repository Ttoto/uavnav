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
using namespace Eigen;


enum Mission_STATE {
    IDLE,
    TAKEOFFP1,
    TAKEOFFP2,
    HOVER,
    ROTATE,
    HOVER2,
    LANDINGP1,
    LANDINGP2,
    END,
} mission_state=IDLE;

double takfoff_speed;
double hover_x,hover_y,hover_z;
double rotation_speed,rotation_speed_max,rotation_speed_min;
double landing_speed;
double landing_x,landing_y,landing_z;

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
    ros::init(argc, argv, "rotate_node");
    ros::NodeHandle nh("~");

    nh.getParam("takfoff_speed", takfoff_speed);

    nh.getParam("hover_x", hover_x);
    nh.getParam("hover_y", hover_y);
    nh.getParam("hover_z", hover_z);

    nh.getParam("rotation_speed",     rotation_speed);
    nh.getParam("rotation_speed_max", rotation_speed_max);
    nh.getParam("rotation_speed_min", rotation_speed_min);

    nh.getParam("landing_speed", landing_speed);
    nh.getParam("landing_x", landing_x);
    nh.getParam("landing_y", landing_y);
    nh.getParam("landing_z", landing_z);
    if(rotation_speed>=rotation_speed_max) rotation_speed=rotation_speed_max;
    if(rotation_speed<=rotation_speed_min) rotation_speed=rotation_speed_min;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, uav_lp_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");


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
    pose.pose.orientation.x = uav_lp_qx;
    pose.pose.orientation.y = uav_lp_qy;
    pose.pose.orientation.z = uav_lp_qz;
    pose.pose.orientation.w = uav_lp_qw;

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
        //PLEASE DEFINE THE PARAMETER HERE
#define TAKEOFF_X            (0.6)
#define TAKEOFF_Y            (0.4)
#define END_TAKEOFF_Z        (1.0)
#define GROUND_Z             (0.0)
#define TAKEOFF_SPEED        (0.1)

        if(mission_state==TAKEOFFP1)
        {
            cout << "TAKEOFFP1";
            double secs = (ros::Time::now()- last_request).toSec();
            pose.pose.position.x = uavposition_x;
            pose.pose.position.y = uavposition_y;
            pose.pose.position.z = (secs)*takfoff_speed;
            if(pose.pose.position.z > (hover_z))
            {
                pose.pose.position.z= hover_z;
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
                m_to_point_1.setDestPos(hover_x,hover_y,hover_z);
            }
            m_to_point_1.getsetpoint(ros::Time::now().toSec(),pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
            if(m_to_point_1.finished())
            {
                mission_state = HOVER;
                cout << "Takeoff P2 finished" << endl;
                last_request = ros::Time::now();
            }
        }

        /*Hover*****************************************************/
        if(mission_state==HOVER)
        {
            if(ros::Time::now() - last_request > ros::Duration(3.0))
            {
                std_msgs::String msg;
                mission_state=ROTATE;
                last_request = ros::Time::now();
                cout << "Hover finished start rotate" << endl;
            }
        }


        if(mission_state==ROTATE)
        {
            static int once = 0;
            static double est_time;

            Eigen::Quaterniond init_q;
            double secs = (ros::Time::now()- last_request).toSec();
            if(once==0)
            {
                once=1;
                double one_circle_time = ((2*PI)/rotation_speed);
                est_time = 2*one_circle_time;
                init_q = Eigen::Quaterniond(uav_lp_qw,uav_lp_qx,uav_lp_qy,uav_lp_qz);
                cout << "est time: " << est_time << endl;
            }
            Eigen::Vector3d rpy= euler_from_rotation_matrix(Matrix3d(init_q));
            rpy[2] += secs*rotation_speed;
            //            while(1){
            //                if(rpy[2]>=(2*PI))
            //                {
            //                    rpy[2]-=(2*PI);
            //                }
            //            }
            Eigen::Quaterniond q(rotation_matrix_from_euler(rpy));
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            if(secs>=est_time)
            {
                mission_state = HOVER2;
                last_request = ros::Time::now();
            }
        }

        if(mission_state==HOVER2)
        {
            if(ros::Time::now() - last_request > ros::Duration(3.0))
            {
                mission_state=LANDINGP1;
                last_request = ros::Time::now();
            }
        }

        //PLEASE DEFINE THE LANDING PARAMETER HERE
        if(mission_state==LANDINGP1)
        {
            static int once = 0;
            static generalMove m_to_landing;
            if(once==0)
            {
                once = 1;
                m_to_landing.setStartTimePos(ros::Time::now().toSec(),uavposition_x,uavposition_y,uavposition_z);
                m_to_landing.setDestPos(landing_x,landing_y,0.3);
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
            pose.pose.position.z = 0.3-(secs)*landing_speed;
            if(pose.pose.position.z < 0.0)
            {
                pose.pose.position.z = 0.0;
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
        cout <<"("
           << pose.pose.position.x << " "
           << pose.pose.position.y << " "
           << pose.pose.position.z << " ) ("
           << pose.pose.orientation.w << " "
           << pose.pose.orientation.x << " "
           << pose.pose.orientation.y << " "
           << pose.pose.orientation.z << " )" << endl;
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
