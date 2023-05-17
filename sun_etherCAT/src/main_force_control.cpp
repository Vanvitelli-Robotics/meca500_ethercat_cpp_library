#include <iostream>
#include "Master.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include "Controller_force.h"

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
            ATINano43 forceSensor(1, &master, 1000000);
            Meca500 meca500(1, &master);
            Controller_force controller_force(&meca500, 0.5);

            master.setupSlave(meca500.getPosition(), Meca500::setup_static);
            //master.config_ec_sync0(1,TRUE,1000000,500000);
            master.configDC();
            master.configMap();

            meca500.assign_pointer_struct();
            forceSensor.assign_pointer_struct();
            try
            {
                master.movetoState(forceSensor.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
                master.createThread(1000000);

                try
                {
                    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    master.movetoState(forceSensor.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);
                    try
                    {
                        bool as, hs, sm, es, pm, eob, eom;
                        int activateRob, deactivateRob, homeRob;
                        float pose_default[6] = {}; //campi da inserire
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

                                forceSensor.start_realtime();

                                if (meca500.setPoint(1) == 0)
                                {
                                    sleep(2);
                                    meca500.moveLin(pose_default);

                                    controller_force.start_force_control();
                                    controller_force.waitLoop();
                                    meca500.moveLin(pose_default);

                                    meca500.setPoint(0);
                                    forceSensor.stop();

                                    sleep(2);
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