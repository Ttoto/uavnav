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
#include "config.h"

enum Mission_STATE {
  on_platform,
  take_off,
  hover,
  to_board_1,
  writing_1,
  to_board_2,
  writing_2,
  to_platform,
  landing
} mission_state=on_platform;

mavros_msgs::State current_state;

double uavposition_x,uavposition_y,uavposition_z;
double platform_x=10.0;
double platform_y=10.0;
double board1_x=10.0;
double board1_y=10.0;
double board2_x=10.0;
double board2_y=10.0;

extern double letterH[10][3];
extern double letterK[8][3];

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void uav_cb(const geometry_msgs::TransformStamped::ConstPtr& transform){
  uavposition_x = transform->transform.translation.x;
  uavposition_y = transform->transform.translation.y;
  uavposition_z = transform->transform.translation.z;
}
void platform_cb(const geometry_msgs::TransformStamped::ConstPtr& transform){
  platform_x = transform->transform.translation.x;
  platform_y = transform->transform.translation.y;
}
void board1_cb(const geometry_msgs::TransformStamped::ConstPtr& transform){
  board1_x = transform->transform.translation.x;
  board1_y = transform->transform.translation.y;
}
void board2_cb(const geometry_msgs::TransformStamped::ConstPtr& transform){
  board2_x = transform->transform.translation.x;
  board2_y = transform->transform.translation.y;
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

  ros::Subscriber uavpos_sub = nh.subscribe<geometry_msgs::TransformStamped>
      ("/mavros/vision_pose/pose", 10, uav_cb);
  ros::Subscriber platform_sub = nh.subscribe<geometry_msgs::TransformStamped>
      ("/mavros/vision_pose/pose", 1, platform_cb);
  ros::Subscriber board1_sub = nh.subscribe<geometry_msgs::TransformStamped>
      ("/vicon/board1/board1", 10, board1_cb);
  ros::Subscriber board2_sub = nh.subscribe<geometry_msgs::TransformStamped>
      ("/vicon/board2/board2", 10, board2_cb);
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);


  //platform board location
  for(int i=10; i>0; i--)
  {
    ros::spinOnce();
    rate.sleep();
  }

  if(platform_x!=10.0 && platform_y!=10.0)
  {
    platform_sub.shutdown();
    std::cout << "platform_x: " << platform_x << std::endl;
    std::cout << "platform_y: " << platform_y << std::endl;
  }
  if(board1_x!=10.0 && board1_y!=10.0)
  {
    board1_sub.shutdown();
    std::cout << "board1_x: " << board1_x << std::endl;
    std::cout << "board1_y: " << board1_y << std::endl;
  }
  if(board2_x!=10.0 && board2_y!=10.0)
  {
    board2_sub.shutdown();
    std::cout << "board2_x: " << board2_x << std::endl;
    std::cout << "board2_y: " << board2_y << std::endl;
  }
  std::cout << "uav_x: " << uavposition_x << std::endl;
  std::cout << "uav_y: " << uavposition_y << std::endl;
  std::cout << "uav_z: " << uavposition_z << std::endl;



  //wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = platform_x;
  pose.pose.position.y = platform_y;
  pose.pose.position.z = PLATFORM_HEIGHT;
//  pose.pose.orientation.x=0.0;
//  pose.pose.orientation.y=0.0;
//  pose.pose.orientation.z=0.707;
//  pose.pose.orientation.w=0.707;

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

  /*//////////////////////////////////////////////////////////////////////////////////////////*/

  std::cout << "hello: " << std::endl;
  while(ros::ok()){

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
          mission_state = take_off;
        }
        last_request = ros::Time::now();
      }
    }



    /*takeoff*****************************************************/
    if((mission_state==take_off))
    {
      double secs = (ros::Time::now()- last_request).toSec();
      pose.pose.position.x = platform_x;
      pose.pose.position.y = platform_y;
      pose.pose.position.z = PLATFORM_HEIGHT+(secs)*TAKE_OFF_SPEED;
      if(pose.pose.position.z > TAKE_OFF_LANDING_HEIGHT)
      {
        pose.pose.position.z= TAKE_OFF_LANDING_HEIGHT;
        ROS_INFO("take-off process finished");
        mission_state = hover;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl;
      std::cout << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl;
    }

    /*hover*****************************************************/
    if((mission_state==hover) && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(board1_x != 10.0 && board1_y!=10.0)
      {
        mission_state = to_board_1;
      }
      else
      {
        mission_state=landing;
      }
      last_request = ros::Time::now();
    }

    /*to_board_1*****************************************************/
    if(mission_state==to_board_1)
    {
      static int flag = 0;
      double velocity_x;
      double velocity_y;
      double moving_time_h;
      double moving_time_v;
      double init_pos_x;
      double init_pos_y;
      double init_pos_z;
      if(flag==0)
      {
        init_pos_x = uavposition_x;
        init_pos_y = uavposition_y;
        init_pos_z = uavposition_z;
        double distance_p2b1 = sqrt((uavposition_x-board1_x)*(uavposition_x-board1_x)
                                    +(uavposition_y-board1_y)*(uavposition_y-board1_y));
        moving_time_h = distance_p2b1/HORIZON_SPEED;
        velocity_x = (board1_x-uavposition_x)/moving_time_h;
        velocity_y = (board1_y-uavposition_y)/moving_time_h;
        moving_time_v = ((TAKE_OFF_LANDING_HEIGHT - CALI_HOLD_HEIGHT)/VERTICAL_SPEED);
        flag++;
        std::cout << "to Board " << std::endl;
        std::cout << "distance_p2b1: " << distance_p2b1 << std::endl;
        std::cout << "moving_time_h: " << moving_time_h << std::endl;
        std::cout << "velocity_x: " << velocity_x << std::endl;
        std::cout << "velocity_y: " << velocity_y << std::endl;
        std::cout << "moving_time_v: " << moving_time_v << std::endl;
      }

      double secs = (ros::Time::now()- last_request).toSec();

      if(secs<=moving_time_h)
      {
        pose.pose.position.x = platform_x + velocity_x*secs;
        pose.pose.position.y = platform_y + velocity_y*secs;
        pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT;
      }
      if(secs>moving_time_h && secs< (moving_time_h+moving_time_v))
      {
        pose.pose.position.x = board1_x;
        pose.pose.position.y = board1_y;
        pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT-VERTICAL_SPEED*(secs-moving_time_h);
      }
      if(secs>(moving_time_h+moving_time_v))
      {
        pose.pose.position.x = board1_x;
        pose.pose.position.y = board1_y;
        pose.pose.position.z = CALI_HOLD_HEIGHT;
      }
      if(secs>((moving_time_h+moving_time_v)+2))
      {
        mission_state = writing_1;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl;
      std::cout << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl;
    }

    if(mission_state==writing_1)
    {
      double tmp_x, tmp_y, tmp_z;
      double init_x = board1_x;
      double init_y = board1_y;
      double init_z = CALI_HOLD_HEIGHT;
      double secs = (ros::Time::now()- last_request).toSec();
      if(writing1(tmp_x,tmp_y,tmp_z, init_x, init_y, init_z,secs,letterH,10))
      {
        pose.pose.position.x = tmp_x;
        pose.pose.position.y = tmp_y;
        pose.pose.position.z = tmp_z;
      }
      else
      {
        mission_state = to_board_2;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl
                << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl << std::endl;
    }

    if((mission_state==to_board_2))
    {
      static int flag = 0;
      double velocity_x;
      double velocity_y;
      double moving_time_h;
      double moving_time_v;
      double init_pos_x;
      double init_pos_y;
      double init_pos_z;
      if(flag==0)
      {
        init_pos_x = uavposition_x;
        init_pos_y = uavposition_y;
        init_pos_z = uavposition_z;
        double distance_b12b2 = sqrt(pow(board2_x-uavposition_x,2)+ pow(board2_y-uavposition_y,2));
        moving_time_h = distance_b12b2/HORIZON_SPEED;
        velocity_x = (board2_x-init_pos_x)/moving_time_h;
        velocity_y = (board2_y-init_pos_y)/moving_time_h;
        moving_time_v = ((CROSS_BOARD_HEIGHT - CALI_HOLD_HEIGHT)/VERTICAL_SPEED);
        flag++;
        std::cout << "to platform " << std::endl;
        std::cout << "distance_b12b2: " << distance_b12b2 << std::endl;
        std::cout << "moving_time_h: " << moving_time_h << std::endl;
        std::cout << "velocity_x: " << velocity_x << std::endl;
        std::cout << "velocity_y: " << velocity_y << std::endl;
        std::cout << "moving_time_v: " << moving_time_v << std::endl;
      }

      double secs = (ros::Time::now()- last_request).toSec();
      if(secs<=moving_time_v)
      {
        pose.pose.position.x = init_pos_x;
        pose.pose.position.y = init_pos_y;
        pose.pose.position.z = CALI_HOLD_HEIGHT+secs*VERTICAL_SPEED;
      }
      if(secs>moving_time_v && secs< (moving_time_h+moving_time_v))
      {
        pose.pose.position.x = init_pos_x + velocity_x*(secs-moving_time_v);
        pose.pose.position.y = init_pos_y + velocity_y*(secs-moving_time_v);
        pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT;
      }
      if(secs>(moving_time_h+moving_time_v))
      {
        pose.pose.position.x = board2_x;
        pose.pose.position.y = board2_y;
        pose.pose.position.z = CALI_HOLD_HEIGHT;
      }
      if(secs>((moving_time_h+moving_time_v)+2))
      {
        mission_state = writing_2;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl;
      std::cout << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl;
    }

    if(mission_state==writing_2)
    {
      double tmp_x, tmp_y, tmp_z;
      double init_x = board2_x;
      double init_y = board2_y;
      double init_z = CALI_HOLD_HEIGHT;
      double secs = (ros::Time::now()- last_request).toSec();
      if(writing2(tmp_x,tmp_y,tmp_z, init_x, init_y, init_z,secs,letterK,8))
      {
        pose.pose.position.x = tmp_x;
        pose.pose.position.y = tmp_y;
        pose.pose.position.z = tmp_z;
      }
      else
      {
        mission_state = to_platform;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl
                << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl << std::endl;
    }

    if((mission_state==to_platform))
    {
      static int flag = 0;
      double velocity_x;
      double velocity_y;
      double moving_time_h;
      double moving_time_v;
      double init_pos_x;
      double init_pos_y;
      double init_pos_z;
      if(flag==0)
      {
        init_pos_x = uavposition_x;
        init_pos_y = uavposition_y;
        init_pos_z = uavposition_z;
        double distance_b12p = sqrt((platform_x-uavposition_x)*(platform_x-uavposition_x)+
                                    (platform_y-uavposition_y)*(platform_y-uavposition_y));
        moving_time_h = distance_b12p/HORIZON_SPEED;
        velocity_x = (platform_x-init_pos_x)/moving_time_h;
        velocity_y = (platform_y-init_pos_y)/moving_time_h;
        moving_time_v = ((TAKE_OFF_LANDING_HEIGHT - CALI_HOLD_HEIGHT)/VERTICAL_SPEED);
        flag++;
        std::cout << "to platform " << std::endl;
        std::cout << "distance_b12p: " << distance_b12p << std::endl;
        std::cout << "moving_time_h: " << moving_time_h << std::endl;
        std::cout << "velocity_x: " << velocity_x << std::endl;
        std::cout << "velocity_y: " << velocity_y << std::endl;
        std::cout << "moving_time_v: " << moving_time_v << std::endl;
      }

      double secs = (ros::Time::now()- last_request).toSec();
      if(secs<=moving_time_v)
      {
        pose.pose.position.x = init_pos_x;
        pose.pose.position.y = init_pos_y;
        pose.pose.position.z = CALI_HOLD_HEIGHT+secs*VERTICAL_SPEED;
      }
      if(secs>moving_time_v && secs< (moving_time_h+moving_time_v))
      {
        pose.pose.position.x = init_pos_x + velocity_x*(secs-moving_time_v);
        pose.pose.position.y = init_pos_y + velocity_y*(secs-moving_time_v);
        pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT;
      }
      if(secs>(moving_time_h+moving_time_v))
      {
        pose.pose.position.x = platform_x;
        pose.pose.position.y = platform_y;
        pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT;
      }
      if(secs>((moving_time_h+moving_time_v)+2))
      {
        mission_state = landing;
        last_request = ros::Time::now();
      }
      std::cout << "secs: " << secs << std::endl;
      std::cout << " x: " << pose.pose.position.x
                << " y: " << pose.pose.position.y
                << " z: " << pose.pose.position.z
                << std::endl;
    }

    if((mission_state==landing))
    {
      double secs = (ros::Time::now()- last_request).toSec();
      pose.pose.position.x = platform_x;
      pose.pose.position.y = platform_y;
      pose.pose.position.z = TAKE_OFF_LANDING_HEIGHT-(secs)*LANDING_SPEED;
      if(pose.pose.position.z < PLATFORM_HEIGHT)
      {
        pose.pose.position.z=PLATFORM_HEIGHT;
        arm_cmd.request.value = false;
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          mission_state==landing;
        }
      }

    }

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
