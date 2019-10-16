#include "generalmove.h"


generalMove::generalMove()
{
    vv = DEFALUT_VV;
    hv = DEFAULT_HV;
    est_flag = 0;
}

void generalMove::setVerticalVelocityLimit(double v)
{
    vv = v;
}

void generalMove::setHorizonVelocityLimit(double v)
{
    hv = v;
}

void generalMove::setDestPos(double x, double y, double z)
{
    endx = x;
    endy = y;
    endz = z;
}

void generalMove::setStartTimePos(double time, double x, double y, double z)
{
    start_time = time;
    startx = x;
    starty = y;
    startz = z;
}

void generalMove::getsetpoint(double time, double &x, double &y, double &z)
{
    curr_time = time;
    if(est_flag == 0)
    {
        double t1 =  (sqrt(pow((endx-startx),2)+pow((endy-starty),2)))/hv;
        double t2 =  (sqrt(pow((endz-startz),2)))/vv;
        est_t = MAX(t1,t2);
        vx = (endx-startx)/est_t;
        vy = (endy-starty)/est_t;
        vz = (endz-startz)/est_t;
    }
    double dt=curr_time-start_time;
    x=startx+dt*vx;
    y=starty+dt*vy;
    z=startz+dt*vz;
}

int generalMove::finished()
{
    if((curr_time-start_time)>=est_t)
    {return 1;}
    else
    {return 0;}
}
