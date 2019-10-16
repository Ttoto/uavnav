#include "math.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <iostream>

extern double uavposition_x,uavposition_y,uavposition_z;

#define CALI_DOWN (1)
#define CALI_UP   (0)


double letterH[10][3] = {
  {  -0.3,    0.4,   CALI_UP},//1

  {  -0.3,    0.4,   CALI_DOWN},//2
  {  -0.3,   -0.4,   CALI_DOWN},//3

  {  -0.3,    0.0,   CALI_UP},//4

  {  -0.3,    0.0,   CALI_DOWN},//5
  {   0.3,    0.0,   CALI_DOWN},//6

  {   0.3,    0.4,   CALI_UP},//7

  {   0.3,    0.4,   CALI_DOWN},//8
  {   0.3,   -0.4,   CALI_DOWN},//9

  {   0.3,   -0.4,   CALI_UP},//10
};

double letterK[8][3] = {
  {  -0.3,    0.4,   CALI_UP},//1

  {  -0.3,    0.4,   CALI_DOWN},//2
  {  -0.3,   -0.4,   CALI_DOWN},//3

  {   0.3,    0.4,  CALI_UP},//4

  {   0.3,    0.4,  CALI_DOWN},//5
  {  -0.3,    0.0,  CALI_DOWN},//6
  {   0.3,   -0.4,  CALI_DOWN},//7

  {   0.3,   -0.4,  CALI_UP},//8

};


int writing1(double &pos_x, double &pos_y, double &pos_z,
             double init_x, double init_y, double init_z,
             double sec, double array[][3], double length)
{

  static double tmp_time=0;
  static double target_x,target_y,target_z;
  static double cur_x,cur_y,cur_z;
  static double time_h, time_v;
  static double speed_x, speed_y;
  static int calistate = CALI_UP;
  static int flag = 0;
  static int i=0;
  for (;i<length;)
  {
    if(flag==0)
    {
      //target & current position
      target_x= init_x+ array[i][0];
      target_y = init_y+ array[i][1];
      if(array[i][2]==CALI_UP) target_z = CALI_HOLD_HEIGHT;
      if(array[i][2]==CALI_DOWN) target_z = CALI_WRITING_HEIGHT;

      if(i==0)
      {
        cur_x = uavposition_x;
        cur_y = uavposition_y;
        cur_z = uavposition_z;
      }
      if(i>=1)
      {
        cur_x = init_x + array[i-1][0];
        cur_y = init_y+ array[i-1][1];
      }

      //calculate the time
      time_v = (fabs(target_z-cur_z))/CALI_WRITING_SPEED_V;
      time_h = sqrt(pow(target_x-cur_x, 2) + pow(target_y-cur_y, 2))/CALI_WRITING_SPEED_H;
      speed_x = (target_x-cur_x)/time_h;
      speed_y = (target_y-cur_y)/time_h;
      flag = 1;
      std::cout << "cur_x: " << cur_x << " ---> target_x: " << target_x << std::endl;
      std::cout << "cur_y: " << cur_y << " ---> target_y: " << target_y << std::endl;
      std::cout << "cur_z: " << cur_z << " ---> target_z: " << target_z <<std::endl;

      std::cout << "time_v: " << time_v << std::endl;
      std::cout << "time_h: " << time_h << std::endl;
      std::cout << "speed_x: " << speed_x << "   speed_y:"<< speed_y <<std::endl;
    }


    //moving in v
    if(calistate!=array[i][2])
    {
      std::cout<<"Vertical "<<i << "   Estimate time: "<< time_v <<std::endl;
      pos_x = cur_x;
      pos_y = cur_y;
      pos_z = cur_z+(sec-tmp_time)*(target_z-cur_z)/time_v;

      if((sec-tmp_time) >= time_v)
      {
        pos_x = cur_x;
        pos_y = cur_y;
        pos_z = target_z;
        if((sec-tmp_time) >= time_v+1.0){
          std::cout<<"Finish Vertical movement"<<i<<std::endl;
          tmp_time = sec;
          calistate = array[i][2];
        }
      }
    }
    //moving in H
    if(calistate==array[i][2])
    {
      std::cout<<"Horizon "<< i << "   Estimate time: "<< time_h <<std::endl;
      if(array[i][2]==CALI_UP) pos_z = CALI_HOLD_HEIGHT;
      if(array[i][2]==CALI_DOWN) pos_z = CALI_WRITING_HEIGHT;
      pos_x = cur_x+(sec-tmp_time)*speed_x;
      pos_y = cur_y+(sec-tmp_time)*speed_y;

      if((sec-tmp_time) >= time_h)
      {
        pos_x = target_x;
        pos_y = target_y;
        pos_z = target_z;
        if((sec-tmp_time) >= time_h+1.0){
          std::cout<<"Finish horizon movement"<<i<<std::endl;
          tmp_time = sec;
          calistate = array[i][2];
          flag = 0;
          i++;
        }
      }
    }
    return 1;
  }
  return 0;
}


int writing2(double &pos_x, double &pos_y, double &pos_z,
             double init_x, double init_y, double init_z,
             double sec, double array[][3], double length)
{

  static double tmp_time=0;
  static double target_x,target_y,target_z;
  static double cur_x,cur_y,cur_z;
  static double time_h, time_v;
  static double speed_x, speed_y;
  static int calistate = CALI_UP;
  static int flag = 0;
  static int i=0;
  for (;i<length;)
  {
    if(flag==0)
    {
      //target & current position
      target_x= init_x+ array[i][0];
      target_y = init_y+ array[i][1];
      if(array[i][2]==CALI_UP) target_z = CALI_HOLD_HEIGHT;
      if(array[i][2]==CALI_DOWN) target_z = CALI_WRITING_HEIGHT;

      if(i==0)
      {
        cur_x = uavposition_x;
        cur_y = uavposition_y;
        cur_z = uavposition_z;
      }
      if(i>=1)
      {
        cur_x = init_x + array[i-1][0];
        cur_y = init_y+ array[i-1][1];
      }

      //calculate the time
      time_v = (fabs(target_z-cur_z))/CALI_WRITING_SPEED_V;
      time_h = sqrt(pow(target_x-cur_x, 2) + pow(target_y-cur_y, 2))/CALI_WRITING_SPEED_H;
      speed_x = (target_x-cur_x)/time_h;
      speed_y = (target_y-cur_y)/time_h;
      flag = 1;
      std::cout << "cur_x: " << cur_x << " ---> target_x: " << target_x << std::endl;
      std::cout << "cur_y: " << cur_y << " ---> target_y: " << target_y << std::endl;
      std::cout << "cur_z: " << cur_z << " ---> target_z: " << target_z <<std::endl;

      std::cout << "time_v: " << time_v << std::endl;
      std::cout << "time_h: " << time_h << std::endl;
      std::cout << "speed_x: " << speed_x << "   speed_y:"<< speed_y <<std::endl;
    }


    //moving in v
    if(calistate!=array[i][2])
    {
      std::cout<<"Vertical "<<i << "   Estimate time: "<< time_v <<std::endl;
      pos_x = cur_x;
      pos_y = cur_y;
      pos_z = cur_z+(sec-tmp_time)*(target_z-cur_z)/time_v;

      if((sec-tmp_time) >= time_v)
      {
        pos_x = cur_x;
        pos_y = cur_y;
        pos_z = target_z;
        if((sec-tmp_time) >= time_v+1.0){
          std::cout<<"Finish Vertical movement"<<i<<std::endl;
          tmp_time = sec;
          calistate = array[i][2];
        }
      }
    }
    //moving in H
    if(calistate==array[i][2])
    {
      std::cout<<"Horizon "<< i << "   Estimate time: "<< time_h <<std::endl;
      if(array[i][2]==CALI_UP) pos_z = CALI_HOLD_HEIGHT;
      if(array[i][2]==CALI_DOWN) pos_z = CALI_WRITING_HEIGHT;
      pos_x = cur_x+(sec-tmp_time)*speed_x;
      pos_y = cur_y+(sec-tmp_time)*speed_y;

      if((sec-tmp_time) >= time_h)
      {
        pos_x = target_x;
        pos_y = target_y;
        pos_z = target_z;
        if((sec-tmp_time) >= time_h+1.0){
          std::cout<<"Finish horizon movement"<<i<<std::endl;
          tmp_time = sec;
          calistate = array[i][2];
          flag = 0;
          i++;
        }
      }
    }
    return 1;
  }
  return 0;
}
