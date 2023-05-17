//IP MECA 192.168.2.103
//tempo di ciclo garantito 2ms, minimo 1ms

#include "Master.h"
#include <pthread.h>
#include <stdexcept>
#include <cstring>
#include <fstream>


#define SET_BIT(prev, bit) (prev | (0x0ff & bit))
#define CLEAR_BIT(prev, bit) (prev & (0x0ff & (~bit)))
#define GET_BIT(mask, value) (mask & value)

#define NSEC_PER_SEC 1000000000 //nanosecondi ogni secondo

//struct of Object Dictionary's entry
typedef struct
{
    int index;
    int sub_index;
    int size;
    int value;

} OBentry;

namespace sun
{
    //Constructor
    Master::Master(char *ifname, uint8 usetable, int timeout)
    {
        initialize(ifname);
        config_init(usetable);
        movetoState_broadcast(EC_STATE_PRE_OP, timeout);
    }

    //destructor
    Master::~Master() {}

    //initialize peripheral ifname
    void Master::initialize(char *ifname)
    {
        //initialize SOEM and match to ifname
        if (ec_init(ifname))
        {
            std::cout << "ec_init on " << ifname << " succeeded.\n";
        }

        else
            throw std::runtime_error("Error initialization!\n\nWARNING: You need 'sudo' to launch the program\n");
    }

    //Configuration and initialization of slave
    //return: number of slaves (0 if not slave)
    int Master::config_init(uint8 usetable)
    {
        int slave_number = ec_config_init(usetable);
        if (slave_number > 0)
        {
            std::cout << ec_slavecount << " slaves found and configured.\n";
            return slave_number;
        }
        else
            throw std::runtime_error("");
    }

    //Change slave's state
    //Return: written state
    uint16 Master::movetoState(uint16 slave, int stato, int timeout)
    {
        //std::cout << "\n\nSTATO_REQUIRED: " << stato << "\n";
        //std::cout << "TIMEOUT: " << timeout << "\n";

        ec_slave[slave].state = stato;
        ec_writestate(slave);

        uint16 state_check = ec_statecheck(slave, stato, timeout);
        //std::cout << "STATE_CHECK: " << state_check<<"\n";

        if (state_check != stato)
            throw std::runtime_error("");

        return state_check;
    }

    //Change slave's state broadcast (at all)
    //Return: written state
    uint16 Master::movetoState_broadcast(int stato, int timeout)
    {
        //std::cout << "\n\nSTATO_REQUIRED: " << stato << "\n";
        //std::cout << "TIMEOUT: " << timeout << "\n";

        ec_slave[0].state = stato;
        ec_writestate(0);
        uint16 state_check = ec_statecheck(0, stato, timeout);

        //std::cout << "STATE_CHECK: " << state_check << "\n";

        if (state_check != stato)
            throw std::runtime_error("");

        return state_check;
    }

    //Setup slave
    void Master::setupSlave(int slave, int (*setup)(uint16 position))
    {
        //when the slave move from PRE_OP state to SAFE_OP state, it calls slave.setup to set parameters and map PDO
        ec_slave[slave].PO2SOconfig = setup;
    }

    //configure DC mechanism
    bool Master::configDC()
    {
        if (!ec_configdc())
            throw std::runtime_error("Error config_DC\n");
        return true;
    }

    //Print slave's state
    void Master::printState()
    {
        int cnt;
        //read and put the state in ec_slave[]
        ec_readstate();
        for (cnt = 1; cnt <= ec_slavecount; cnt++)
        {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d, StatusCode=0x%4.4x : %s \n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc,
                   ec_slave[cnt].ALstatuscode, ec_ALstatuscode2string(ec_slave[cnt].ALstatuscode));
        }
    }

    void Master::setCycle(int64 cycletime)
    {
        this->cycletime = cycletime;
    }

    //funzione che aggiorna quando ecathread dovrà svegliarsi
    void Master::add_timespec(struct timespec *ts, int64 addtime)
    {
        int64 sec, nsec;

        nsec = addtime % NSEC_PER_SEC;
        sec = (addtime - nsec) / NSEC_PER_SEC;
        ts->tv_sec += sec;
        ts->tv_nsec += nsec;
        if (ts->tv_nsec > NSEC_PER_SEC)
        {
            nsec = ts->tv_nsec % NSEC_PER_SEC;
            ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
            ts->tv_nsec = nsec;
        }
    }

    //sincronizzazione del clock del master e della rete
    void Master::ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
    {
        static int64 integral = 0;
        int64 delta;
        delta = (reftime) % cycletime;
        if (delta > (cycletime / 2))
        {
            delta = delta - cycletime;
        }
        if (delta > 0)
        {
            integral++;
        }
        if (delta < 0)
        {
            integral--;
        }
        *offsettime = -(delta / 100) - (integral / 20);
        //gldelta = delta;
    }

    void Master::config_ec_sync0(uint16 position, bool activate, uint32 cycletime, int cycleshift)
    {
        ec_dcsync0(position, activate, cycletime, cycleshift);
    }

    void Master::ecatthread()
    {
        struct timespec ts, tleft;
        int ht;
        struct sched_attr attr;
        attr.size = sizeof(attr);
        sched_rr(&attr, 40, 0);

        std::cout << "Start real time thread\n";
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ht = (ts.tv_nsec / 1000000) + 1; // round to nearest ms
        ts.tv_nsec = ht * 1000000;
        toff = 0;
        // eseguo il pinning della pagine attuali e future occupate dal thread per garantire
        //   prevedibilità nelle prestazioni real-time
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
            std::cout << "mlockall failed\n";
            pthread_cancel(pthread_self());
        }

        //for (int i = 0; i < 5000; i++)
        i = 0;
        while (shutdown)
        {
            time1 = ec_DCtime;
            this->add_timespec(&ts, cycletime + toff);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

            mtx.lock();
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            mtx.unlock();

            this->ec_sync(ec_DCtime, cycletime, &toff);
            time2 = ec_DCtime;
            cycle = time2 - time1;
            if (i < 50000)
                timecycle[i++] = cycle;
        }
    }

    void Master::createThread(int64 cycleTime)
    {
        if (!thread)
        {
            //create real-time thread to exchange data
            setCycle(cycleTime);
            shutdown = true;
            thread_master = std::thread(&Master::ecatthread, this);
            thread = true;
        }
        else
            std::cout << "The thread already exists\n";
    }

    void Master::waitThread()
    {
        thread_master.join();
    }

    void Master::close_master()
    {
        //movetoState_broadcast(EC_STATE_SAFE_OP, 200000);
        //movetoState_broadcast(EC_STATE_PRE_OP, 9000000);
        shutdown = false;
        thread = false;
        ec_close();
    }

    void Master::configMap()
    {
        if (!ec_config_map(&IOmap))
            throw std::runtime_error("Error config_map\n");
    }

    void Master::stampa()
    {
        // for (int i = 0; i < 5000; i++)
        // {
        //     printf("PDO: i=%d, calc_cycle=%ld[ns],cycle_read=%ld[ns] \n",
        //            i, cycle, ec_slave[1].DCcycle);
        //     fflush(stdout);
        //     osal_usleep(1000);
        // }

        std::ofstream oFile("Cycle_time.txt", std::ios_base::out | std::ios_base::trunc);
        if (oFile.is_open())
        {
            for (int y = 0; y < i; y++)
                oFile << timecycle[y] << "\n";
            oFile.close();
        }
        
        /*for (int i = 0; i < 50000; i++)
        {
            printf("Cycle_time_%d: %d \n", i + 1, timecycle[i]);
            fflush(stdout);
            osal_usleep(1000);
        }*/
    }

    void Master::deactivate()
    {
        printf("Before: %c\n", IOmap[0]);
        IOmap[0] = CLEAR_BIT(0x02, 0x02);
        usleep(1000);
        IOmap[0] = CLEAR_BIT(0x04, 0x04);
        usleep(1000);
        IOmap[0] = SET_BIT(0x00, 0x01);
        printf("After1: %c\n", IOmap[0]);
        usleep(1000);
        printf("After2: %c\n", IOmap[0]);
    }

    uint8 *Master::getOutput_slave(uint16 position)
    {
        //std::cout<<"outputs_master: "<<ec_slave[position].outputs<<"\n";
        return ec_slave[position].outputs;
    }

    uint8 *Master::getInput_slave(uint16 position)
    {
        //std::cout<<"inputs_master: "<<ec_slave[position].inputs<<"\n";
        return ec_slave[position].inputs;
    }

    void Master::mutex_down()
    {
        mtx.lock();
    }

    void Master::mutex_up()
    {
        mtx.unlock();
    }
} // namespace sun
