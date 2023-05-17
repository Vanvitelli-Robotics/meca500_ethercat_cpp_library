#ifndef ROBOT_H
#define ROBOT_H

#include <cstdint>
#include <iostream>
#include <iostream>
#include "Master.h"
// #include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <cmath>

class Robot
{
private:
    sun::Meca500 meca500;
    sun::Master master;
    sun::Controller controller;
    bool as, hs, sm, es, pm, eob, eom;
    float joint_angles[6];
    float joints[6] = {0, 0, 0, 0, 90, 0};
    float omega[6] = {0, 0, 0, 0, 0, -30};
    int activateRob, deactivateRob, homeRob;
    const uint32_t TARGET_CYCLE_TIME_MICROSECONDS;
    const double POS_LIMIT;
    static void update_data();
    bool block_ended();

public:
    Robot(double pos_limit,uint32_t target_cycle_time_microseconds);
    int main();
    ~Robot();
    void activate();
    void home();
    void deactivate();
    void reset_error();
    void print_number(double number);
    double get_velocity();
    double get_position();
    double get_target_velocity();
    void move_lin_vel_trf(double velocity);
    void set_conf(short c1, short c2, short c3);
    void move_pose(double x, double y, double z, double alpha, double beta, double gamma);
    void set_monitoring_interval(uint32_t monitoring_interval_microseconds);
    double get_position_timestamp();
    double get_speed_timestamp();
    double get_target_speed_timestamp();
};

#endif