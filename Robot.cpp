#include "Robot.hpp"
#include <iostream>
#include "Master.h"
// #include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <cmath>

Robot::Robot(double pos_limit, uint32_t target_cycle_time_microseconds,
             char *network_interface_in,
             float blending_percentage,
             float cart_accel_limit) : POS_LIMIT(pos_limit),
                                       master(network_interface_in, FALSE, EC_TIMEOUT_TO_SAFE_OP),
                                       meca500(1, &master),
                                       controller(&meca500, 0.5),
                                       TARGET_CYCLE_TIME_MICROSECONDS(target_cycle_time_microseconds)

{
    using namespace sun;
    using namespace std;

    sprintf(network_interface, network_interface_in);

    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    uint16 state_check;

    long time_new_packet[500];

    master.setupSlave(meca500.getPosition(), Meca500::setup_static);

    master.configDC();
    master.configMap();

    meca500.assign_pointer_struct();

    master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
    master.createThread(TARGET_CYCLE_TIME_MICROSECONDS * 1e+3);

    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);

    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    printf("\nActivate: %d\n", as);
    printf("Homed: %d\n", hs);
    printf("Sim: %d\n", sm);
    printf("Error: %d\n", es);

    if (meca500.setBlending(blending_percentage) != 0)
    {
        cout << "Error set blending\n";
    }

    if (meca500.setCartAcc(cart_accel_limit) != 0)
    {
        cout << "error set cart acceleration\n";
    }

    meca500.activateRobot();
    meca500.home();

    meca500.setPoint(1);
}

bool Robot::block_ended()
{
    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    std::cout << "eob:" << eob << std::endl;
    return eob;
}

bool Robot::movement_ended()
{
    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    std::cout << "eom:" << eom << std::endl;
    return eom;
}

Robot::~Robot()
{
    master.close_master();
    master.stampa();
    master.waitThread();
}

void Robot::deactivate()
{
    meca500.deactivateRobot();
}

void Robot::reset_error()
{
    meca500.resetError();
}

double Robot::get_position() // Returning Horizontal position of the Robot
{
    float pose[6];
    meca500.getPose(pose);
    return pose[0]; // Checking if is the right return
}

void Robot::print_pose()
{
    float pose[6];
    meca500.getPose(pose);
    for (int i = 0; i < 6; i++)
    {
        std::cout << i << ": " << pose[i] << std::endl;
    }
}

void Robot::move_lin_vel_trf(double velocity) // From -1000 mm/s to 1000 mm/s
{
    float vel[6] = {0, 0, 0, 0, 0, 0};
    vel[0] = (float)velocity;
    meca500.moveLinVelTRF(vel);
}

void Robot::set_conf(short c1, short c2, short c3)
{
    float conf[] = {(float)c1, (float)c2, (float)c3};
    int n = meca500.setConf(conf);
    std::cout << n << std::endl;
}
void Robot::move_pose(double x, double y, double z, double alpha, double beta, double gamma)
{
    float pose[] = {(float)x, (float)y, (float)z, (float)alpha, (float)beta, (float)gamma};
    meca500.movePose(pose);
    usleep(0.2e+6);
    while (!movement_ended())
    {
        printf("waiting for robot to finish moving\n");
        usleep(1e+6);
    }
}