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

Robot::Robot(double pos_limit , uint32_t target_cycle_time_microseconds) : POS_LIMIT(pos_limit),
                                 master("eno1", FALSE, EC_TIMEOUT_TO_SAFE_OP), meca500(1, &master), controller(&meca500, 0.5) ,
                                 TARGET_CYCLE_TIME_MICROSECONDS(target_cycle_time_microseconds)

{
    using namespace sun;
    using namespace std;

    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    uint16 state_check;

    long time_new_packet[500];

    master.setupSlave(meca500.getPosition(), Meca500::setup_static);

    master.configDC();
    master.configMap();

    meca500.assign_pointer_struct();

    master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
    master.createThread(TARGET_CYCLE_TIME_MICROSECONDS*1e+3);

    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);

    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    printf("\nActivate: %d\n", as);
    printf("Homed: %d\n", hs);
    printf("Sim: %d\n", sm);
    printf("Error: %d\n", es);
}

bool Robot::block_ended() {
    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    return eob;
}

Robot::~Robot()
{
    master.close_master();
    master.stampa();
    master.waitThread();
}

void Robot::activate()
{
    meca500.activateRobot();
}
void Robot::home()
{
    meca500.home();
}
void Robot::deactivate()
{
    meca500.deactivateRobot();
}

int Robot::main()
{
    meca500.resetError();

    activateRob = meca500.activateRobot();
    if (activateRob == 0 || activateRob == 1)
    {
        if (activateRob == 1)
            std::cout << "Motors already activated.\n";
        else
            std::cout << "Motors activated.\n";

        homeRob = meca500.home();
        if (homeRob == 0 || homeRob == 1)
        {
            if (activateRob == 1)
                std::cout << "Home already done.\n";
            else
                std::cout << "Home done.\n";

            meca500.getJoints(joint_angles);

            for (int i = 0; i < 6; i++)
            {
                std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
            }
            printf("\n\n");

            // theta_0 = joint_angles[5];

            // for (int i = 1; i < dim; i++)
            // {
            //     time_array[i] = time_array[i - 1] + Tc;
            //     tau = time_array[i] / tf;
            //     theta_d[i] = theta_0 + (theta_f - theta_0) * (b[0] * pow(tau, 5) + b[1] * pow(tau, 4) + b[2] * pow(tau, 3));
            //     //std::cout << theta_d[i]<<"\n";
            //     //printf("%f\n", theta_d[i]);
            // }

            if (meca500.setPoint(1) == 0)
            {

                meca500.moveJoints(joints);

                controller.startThread();

                controller.waitLoop(); // the user thread waits the end of thread controller.
                meca500.setPoint(0);

                meca500.getJoints(joint_angles);

                for (int i = 0; i < 6; i++)
                {
                    std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                }
                printf("\n\n");
            }

            else
                std::cout << "Invalid input!\n";
        }

        else
        {
            if (homeRob == -1)
                std::cout << "Motors must be activated to do home";
            else
                std::cout << "ERROR_Homing.\n";
        }
    }
    else
    {
        std::cout << "ERROR_Activate.\n";
    }

    // meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    // printf("\nActivate: %d\n", as);
    // printf("Homed: %d\n", hs);
    // printf("Sim: %d\n", sm);
    // printf("Error: %d\n", es);
}