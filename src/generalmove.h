#ifndef GENERALMOVE_H
#define GENERALMOVE_H

#include <math.h>

#define DEFALUT_VV (0.05)
#define DEFAULT_HV (0.05)
#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

class generalMove
{
public:
    generalMove();
    void setStartTimePos(double time, double x, double y, double z);
    void setDestPos(double x, double y, double z);
    void setVerticalVelocityLimit(double v);
    void setHorizonVelocityLimit(double v);
    void getsetpoint(double time, double &x, double &y, double &z);
    int finished(void);
private:
    double endx, endy, endz;
    double startx, starty, startz;
    double hv,vv;
    double start_time;
    double curr_time;
    double vx,vy,vz;
    double est_t;

    int est_flag;
};



#endif // GENERALMOVE_H
