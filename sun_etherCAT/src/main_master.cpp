
#include <iostream>
#include "Master.h"
#include "Meca500.h"
#include "ATINano43.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>

using namespace sun;
typedef int (Meca500::*meca_fun_ptr)(uint16);
using namespace std;
using namespace EtherCAT;
int main(int argc, char *argv[])
{
    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    if (argc > 1)
    {
        char *ifname = argv[1];
        uint16 state_check;

        try
        {
            long time_new_packet[500];
            Master master(ifname, FALSE, EC_TIMEOUT_TO_SAFE_OP);
            //ATINano43 forceSensor(1, &master, 1000000);
            Meca500 meca500(1, &master);

            master.setupSlave(meca500.getPosition(), Meca500::setup_static);
            //master.config_ec_sync0(1,TRUE,1000000,500000);
            master.configDC();
            master.configMap();

            meca500.assign_pointer_struct();
            //forceSensor.assign_pointer_struct();
            try
            {
                //master.movetoState(forceSensor.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.createThread(1000000);

                try
                {
                    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    //master.movetoState(forceSensor.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    try
                    {
                        //MECA500_CODE

                        bool as, hs, sm, es, pm, eob, eom;
                        float joint_angles[6];
                        float joints[6] = {0, 0, 0, 0, 90, 0};
                        float omega[6] = {0, 0, 0, 0, 0, -30};
                        int activateRob, deactivateRob, homeRob;

                        sleep(5);

                        meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        printf("\nActivate: %d\n", as);
                        printf("Homed: %d\n", hs);
                        printf("Sim: %d\n", sm);
                        printf("Error: %d\n", es);

                        activateRob = meca500.activateRobot();
                        if (activateRob == 0 || activateRob == 1)
                        {
                            if (activateRob == 1)
                                std::cout << "Motors already activated.\n";
                            else
                                std::cout << "Motors activated.\n";

                            sleep(2);

                            homeRob = meca500.home();
                            if (homeRob == 0 || homeRob == 1)
                            {
                                if (activateRob == 1)
                                    std::cout << "Home already done.\n";
                                else
                                    std::cout << "Home done.\n";
                                sleep(2);

                                meca500.getJoints(joint_angles);

                                for (int i = 0; i < 6; i++)
                                {
                                    std::cout << "Joint_" << i + 1 << ": " << joint_angles[i] << "\n";
                                }
                                printf("\n\n");

                                sleep(2);
                                if (meca500.setPoint(1) == 0)
                                {
                                    sleep(2);

                                    //meca500.SetVelTimeout(0.005);

                                    //sleep(2);

                                    //meca500.moveJointsVel(omega);
                                    meca500.moveJoints(joints, 1);

                                    sleep(2);

                                    meca500.setPoint(0);

                                    sleep(2);

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

                            sleep(2);

                            deactivateRob = meca500.deactivateRobot();
                            if (deactivateRob == 0)
                                std::cout << "Motors deactivated";
                        }
                        else
                        {
                            std::cout << "ERROR_Activate.\n";
                        }

                        master.stampa();

                        // meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        // printf("\nActivate: %d\n", as);
                        // printf("Homed: %d\n", hs);
                        // printf("Sim: %d\n", sm);
                        // printf("Error: %d\n", es);

                        //ATINANO43_CODE
                        /*forceSensor.start_realtime();
                        double array_forces[3];
                        double array_torques[3];
                        uint32 array_status[3];

                        int array_iteration = 0;
                        uint32 last_packet = -1;
                        int y = 0;

                        for (int i = 0; i < 500; i++)
                            time_new_packet[i] = 0;

                        struct timespec ts, ht;
                        //auto start = std::chrono::system_clock::now();
                        clock_gettime(CLOCK_MONOTONIC, &ts);
                        while (y < 5000)
                        {
                            forceSensor.getStatus(array_status);
                            if (array_status[0] != last_packet)
                            {
                                last_packet = array_status[0];
                                forceSensor.getForces(array_forces);
                                forceSensor.getTorques(array_torques);
                                printf("Value: \n");
                                for (int i = 0; i < 3; i++)
                                {
                                    printf("F_%d: %f\t", i, array_forces[i]);
                                    printf("T_%d: %f", i, array_torques[i]);
                                    printf("\n");
                                }
                                printf("\n");

                                //auto end = std::chrono::system_clock::now();
                                clock_gettime(CLOCK_MONOTONIC, &ht);
                                if (array_iteration < 500)
                                {
                                    time_new_packet[array_iteration++] = ht.tv_nsec - ts.tv_nsec;
                                    //time_new_packet[array_iteration++]=end-start;
                                    //time_new_packet[array_iteration++] = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count(); //us
                                }
                                //auto start = std::chrono::system_clock::now();
                                clock_gettime(CLOCK_MONOTONIC, &ts);
                                //std::cout << "\n\nY: " << y;
                                y++;
                            }
                        }

                        forceSensor.stop();*/
                    }
                    catch (const std::runtime_error &e)
                    {
                        cerr << e.what();
                    }

                    usleep(5000000);
                    try
                    {
                        master.close_master();
                        
                    }
                    catch (const std::runtime_error &e)
                    {
                        std::cout << "Error close_master\n";
                    }
                    master.waitThread();

                    //scrittura su file
                    /*std::ofstream oFile("Time_new_packet.txt", std::ios_base::out | std::ios_base::trunc);
                    if (oFile.is_open())
                    {
                        for (int i = 0; i < 500; i++)
                            oFile << time_new_packet[i] << "\n";
                        oFile.close();
                    }*/
                }
                catch (const std::runtime_error &e)
                {
                    std::cout << "Error state_transition SAFE_OP-> OP\n";
                }
            }
            catch (const std::runtime_error &e)
            {
                std::cout << "Error state_transition PRE_OP-> SAFE_OP\n";
            }
        }
        catch (const std::runtime_error &e)
        {
            cerr << e.what();
            return -1;
        }
    }
}