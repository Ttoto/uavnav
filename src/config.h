#ifndef CONFIG_H



#define TAKE_OFF_SPEED  (0.1)
#define LANDING_SPEED  (0.1)
#define PLATFORM_HEIGHT (0.7)
#define TAKE_OFF_LANDING_HEIGHT (1.6)
#define HORIZON_SPEED (0.4)
#define VERTICAL_SPEED (0.15)
#define CALI_WRITING_HEIGHT (0.60)
#define CALI_HOLD_HEIGHT (0.8)
#define CALI_WRITING_SPEED_V (0.8)
#define CALI_WRITING_SPEED_H (0.4)

#define CROSS_BOARD_HEIGHT (1.2)
#define CONFIG_H



int writing1(double &pos_x, double &pos_y, double &pos_z,
             double init_x, double init_y, double init_z,
             double sec, double array[][3], double LENGTH);

int writing2(double &pos_x, double &pos_y, double &pos_z,
             double init_x, double init_y, double init_z,
             double sec, double array[][3], double LENGTH);

#endif // CONFIG_H
