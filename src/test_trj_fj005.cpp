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
#include "utils/yamlRead.h"
#include "utils/eigen_typedef.h"
#include "movement/generalmove.h"
#include "movement/circletrj.h"

Vec3 takeoff;
double takeoff_x,takeoff_y,takeoff_z;
int    half_circle;
bool   force_start;

#define PI (3.1415926)

using namespace std;

enum Mission_STATE {
  IDLE,
  TAKEOFFP1,
  TAKEOFFP2,
  HOVER,
  GOTO_CIRCLE_START,
  CIRCLE1,
  HOVER1,
  RECT1,
  RECT2,
  RECT3,
  RECT4,
  RECT5,
  HOVER2,
  RETURN,
  LANDING,
  END,
} mission_state=IDLE;

mavros_msgs::State current_state;
double uavposition_x,uavposition_y,uavposition_z;
double tmpposition_x,tmpposition_y,tmpposition_z,tmporientation_yaw;
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
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh("~");

  string configFilePath;

  nh.getParam("force_start", force_start);
  cout << force_start << endl;

  takeoff_x = 0.0;
  takeoff_y = 0.0;
  takeoff_z = 1.2;
  takeoff = Vec3(takeoff_x,takeoff_y,takeoff_z);
  cout << "takeoff_x:" << takeoff_x << endl;
  cout << "takeoff_y:" << takeoff_y << endl;
  cout << "takeoff_z:" << takeoff_z << endl;

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

  //triggle (vio platform test)
  //    ros::Publisher tri_start_pub = nh.advertise<std_msgs::String>("tri_start", 10);
  //    ros::Publisher tri_end_pub = nh.advertise<std_msgs::String>("tri_end", 10);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  //wait for FCU connection
  if(force_start)
  {
    cout << "force start " << endl;
  }
  else
  {
    cout << "Waiting for FCU connection " << endl;
    while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
      cout << "Waiting for FCU connection " << endl;
    }
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x=0.0;
  pose.pose.orientation.y=0.0;
  pose.pose.orientation.z=0.0;
  pose.pose.orientation.w=1.0;

  //send a few setpoints before starting
  if(force_start)
  {
    cout << "force start " << endl;
  }
  else
  {
    for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  cout << "change last_request A" << endl;
  ros::Time last_request = ros::Time::now();

  while(ros::ok()){
    //update the uav position
    //from motion capture system
    //local postion estimator(in simulation platform)
    uavposition_x = uav_lp_x;
    uavposition_y = uav_lp_y;
    uavposition_z = uav_lp_z;

    /*offboard and arm*****************************************************/
    if(force_start)
    {
      static bool once=true;
      if(once)
      {
        mission_state = TAKEOFFP1;
        last_request = ros::Time::now();
        cout << "force start the mission " << endl;
        once = false;
      }
    }
    else
    {
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
    }

    /*takeoff*****************************************************/
    //PLEASE DEFINE THE LANDING PARAMETER HERE
    if(mission_state==TAKEOFFP1)
    {
      static generalMove takeoff1(ros::Time::now().toSec(),
                                  0,0,0-0.2,0.0,
                                  takeoff_x,takeoff_y,takeoff_z-0.2,0.0,(takeoff_z-0.2)/0.1);
      takeoff1.getPose(ros::Time::now().toSec(),pose);
      if(takeoff1.finished())
      {
        cout << "Takeoff P1 finished" << endl;
        mission_state = TAKEOFFP2;
        last_request = ros::Time::now();
      }
    }
    if(mission_state==TAKEOFFP2)
    {
      static generalMove takeoff2(ros::Time::now().toSec(),
                                  takeoff_x,takeoff_y,takeoff_z-0.2,0.0,
                                  takeoff_x,takeoff_y,takeoff_z,0.0,5);
      takeoff2.getPose(ros::Time::now().toSec(),pose);
      if(takeoff2.finished())
      {
        mission_state = GOTO_CIRCLE_START;
        cout << "Takeoff P2 finished" << endl;
        last_request = ros::Time::now();
      }
    }

    /*go to touch point and adjust orientation*/
    if(mission_state==GOTO_CIRCLE_START)
    {
      static generalMove go2cirlestart(ros::Time::now().toSec(),
                                       takeoff_x,    takeoff_y,  takeoff_z,  0.0,
                                       takeoff_x+0.8,takeoff_y,  takeoff_z,  0.0, 3);
      go2cirlestart.getPose(ros::Time::now().toSec(),pose);
      if(go2cirlestart.finished())
      {
        mission_state = CIRCLE1;
        cout << "reached touch point" << endl;
        last_request = ros::Time::now();
      }
    }


    if(mission_state==CIRCLE1)
    {
      static circleTrj circle1(ros::Time::now().toSec(),
                               takeoff_x+0.8,takeoff_y,  takeoff_z,
                               takeoff_x,takeoff_y,takeoff_z,
                               -18.85,30,CIRCLE_TRJ_FACING_FIXED);
      circle1.getPose(ros::Time::now().toSec(),pose);
      if(circle1.finished())
      {
        circle1.getEnding(tmpposition_x,tmpposition_y,tmpposition_z,tmporientation_yaw);
        mission_state = HOVER2;
        cout << "circle1_finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER1)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RECT1;
        cout << "Hover finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT1)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            takeoff_x+0.7, takeoff_y,  takeoff_z,   0.0,
                            takeoff_x+0.7, takeoff_y+1, takeoff_z,  0.0, 3);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = RECT2;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT2)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            takeoff_x+0.7, takeoff_y+1, takeoff_z,  0.0,
                            takeoff_x-0.7, takeoff_y+1, takeoff_z,  0.0, 5);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = RECT3;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT3)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            takeoff_x-0.7, takeoff_y+1, takeoff_z,  0.0,
                            takeoff_x-0.7, takeoff_y-1, takeoff_z,  0.0, 6);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = RECT4;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT4)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            takeoff_x-0.7, takeoff_y-1, takeoff_z,  0.0,
                            takeoff_x+0.7, takeoff_y-1, takeoff_z,  0.0, 6);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = RECT5;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RECT5)
    {
      static generalMove gm(ros::Time::now().toSec(),
                            takeoff_x+0.7, takeoff_y-1, takeoff_z,  0.0,
                            takeoff_x+0.7, takeoff_y,   takeoff_z,  0.0, 3);
      gm.getPose(ros::Time::now().toSec(),pose);
      if(gm.finished())
      {
        mission_state = HOVER2;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==HOVER2)
    {
      if(ros::Time::now()-last_request > ros::Duration(1.0))
      {
        mission_state = RETURN;
        cout << "Hover finished" << endl;
        last_request = ros::Time::now();
      }
    }

    if(mission_state==RETURN)
    {
      static generalMove gotolanding(ros::Time::now().toSec(),
                                     takeoff_x+0.8, takeoff_y,   takeoff_z,  0.0,
                                     takeoff_x,takeoff_y,takeoff_z,0.0,
                                     3);
      gotolanding.getPose(ros::Time::now().toSec(),pose);
      if(gotolanding.finished())
      {
        mission_state = LANDING;
        cout << "reached landing place" << endl;
        last_request = ros::Time::now();
      }
    }

    //PLEASE DEFINE THE LANDING PARAMETER HERE
    if(mission_state==LANDING)
    {
      double secs = (ros::Time::now() - last_request).toSec();
      //cout << secs << endl;
      pose.pose.position.z = takeoff_z-(secs)*0.1;
      if(pose.pose.position.z < -0.3)
      {
        pose.pose.position.z=-0.3;
        arm_cmd.request.value = false;
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          mission_state=END;
          cout << "Landing P2 finished" << endl;
          return 0;//break the control UAV will land automatically
        }
      }
    }

    if(mission_state==END)
    {
      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = -0.3;
      return 0;
    }
    //cout  << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << endl;

    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
