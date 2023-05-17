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
            ATINano43 forceSensor(1, &master, 1000000);
            Meca500 meca500(2, &master);

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
                        float pose_measure[6] = {}; //campi da inserire
                        float k = 0;                //elastic constant

                        double array_forces[3];
                        double force_z_bias[5000];
                        double bias;
                        uint32 array_status[3];
                        int array_iteration = 0;
                        uint32 last_packet = -1;
                        int y = 0;

                        sleep(5);

                        meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
                        printf("\nActivate: %d\n", as);
                        printf("Homed: %d\n", hs);
                        printf("Sim: %d\n", sm);
                        printf("Error: %d\n", es);

                        //bias
                        //ATINANO43_CODE
                        forceSensor.start_realtime();

                        for (int i = 0; i < 500; i++)
                            time_new_packet[i] = 0;

                        while (y < 5000)
                        {
                            forceSensor.getStatus(array_status);
                            if (array_status[0] != last_packet)
                            {
                                last_packet = array_status[0];
                                forceSensor.getForces(array_forces);
                                force_z_bias[y] = array_forces[2];
                                y++;
                            }
                        }
                        for (int i = 0; i < 5000; i++)
                        {
                            bias = bias + force_z_bias[i];
                        }
                        bias = bias / 5000;
                        std::cout << "bias_z: " << bias << "\n";

                        forceSensor.stop();

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

                                if (meca500.setPoint(1) == 0)
                                {
                                    sleep(2);
                                    meca500.moveLin(pose_default); //move robot into the default pose

                                    //ATINANO43_start
                                    forceSensor.start_realtime();

                                    meca500.moveLin(pose_measure);
                                    sleep(2);

                                    for (int i = 0; i < 5; i++)
                                    {
                                        forceSensor.getStatus(array_status);
                                        if (array_status[0] != last_packet)
                                        {
                                            last_packet = array_status[0];
                                            forceSensor.getForces(array_forces);

                                            printf("Value: \n");
                                            for (int i = 0; i < 3; i++)
                                            {
                                                printf("F_%d: %f\t", i, array_forces[i]-bias);
                                                printf("\n");
                                            }
                                            printf("\n");
                                        }
                                    }
                                    meca500.moveLin(pose_default);
                                    forceSensor.stop();

                                    meca500.setPoint(0);

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

                        //master.stampa();
                        k = array_forces[2] / pose_measure[2];
                        std::cout << "fz_applicata: " << array_forces[0] - bias << "\n";
                        std::cout << "spostamento_z: " << pose_measure[2] << "\n";
                        std::cout << "k_misurata: " << k << "\n";
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