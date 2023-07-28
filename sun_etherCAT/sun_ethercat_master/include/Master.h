#ifndef SUN_MASTER
#define SUN_MASTER

#define stack8k (8 * 1024)

#include <sys/mman.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <mutex>
#include <thread>
#include "ethercat.h"
#include <array>

extern "C"
{
#include "scheduling.h" //a C header, so wrap it in extern "C"
}

namespace sun
{
    /**
     * This class abstracts the role of the master in an ethercat network.
     * It implements the effective communication between master and slaves.
    */
    class Master
    {
    private:
        char IOmap[4096]; /**< Buffer used by all slaves to write and read data */
        pthread_t tidm;
        bool thread = false;
        bool shutdown = true;
        int64 toff;
        int i=0;
        long int time1;
        long int time2;
        std::array<int,50000> timecycle;
        long int cycle;
        volatile int wkc;
        int64 cycletime;
        void ecatthread();
        void add_timespec(struct timespec *ts, int64 addtime);
        void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
        std::mutex mtx;
        std::thread thread_master;

    public:
        void deactivate();

        /**
         * Constructor of Master
         * @param char* ifname this is the port for ethercat comunication
         * @param uint8 usetabele 
         * @param int timeout 
         */
        Master(char *ifname, uint8 usetable, int timeout);

        /**
         * Distructor
        */
        ~Master();

        /**
         * Initialize the peripheral for the communication.
         * @param char* ifname this is the port for ethercat comunication
         * @throw runtime_error 
        */
        void initialize(char *ifname);

        /**
         * Configuration and initialization of slave.
         * @param uint8 usetabele 
         * @return int number_slave returns the number of slaves found
         * @throw runtime_error 
        */
        int config_init(uint8 usetable);

        /**
         * Changes the status of a slave.
         * @param int slave this is the position of the slave in the network.
         * @param uint16 stato this is the state in which you want to move the slave.
         * @param int timeout 
         * @return uint16 written state returns the state written by the master
         * @throw runtime_error 
        */
        uint16 movetoState(uint16 slave, int stato, int timeout);

        /**
         * Changes the status of all the slaves.
         * @param uint16 stato this is the state in which you want to move the slaves.
         * @param int timeout 
         * @return uint16 written state returns the state written by the master
         * @throw runtime_error 
        */
        uint16 movetoState_broadcast(int stato, int timeout);

        /**
         * When the slave move from PRE_OP state to SAFE_OP state, it calls slave.setup to set parameters and map PDO.
         * @param int slave this is the position of the slave in the network.
         * @param (int)(*setup)(uint16 x) this is a function pointer.
        */
        void setupSlave(int slave, int (*setup)(uint16 position));

        /**
         * Configure DC mechanism
         * @return bool true if the DC mechanism is setted.
         * @throw runtime_error 
        */
        bool configDC();

        void config_ec_sync0(uint16 position, bool activate, uint32 cycletime, int cycleshift);

        /**
         * Print the state of all slaves.
        */
        void printState();

        /**
         * Creates the real-time thread for the cyclical exchange of ethercat package.
         * It sends the buffer IOmap every 'cycletime'
         * @param int timeout 
         * @param int64 cycletime this is the time of cycle to send ethercat package.
        */
        void createThread(int64 cycleTime);

        /**
         * It moves all slaves to the PRE_OP state and closes the ethercat connection.
         * @param int timeout 
        */
        void close_master();

        /**
         * It configures the IOmap.
         * @throw runtime_error 
        */
        void configMap();

        // /**
        //  * Changes the status of all the slaves.
        //  * @param uint8* array this is the buffer used to exchange data written as sequence of bytes.
        //  * @param int slave this is the position of slave thaht has to comunication in the network
        //  * @param int dim this is the dimension of data to write
        //  * @param int offset the byte to start writing the data
        // */
        // void write_slave(uint8 *array, int slave, int dim, int offset);

        // /**
        //  * Changes the status of all the slaves.
        //  * @param uint8* array this is the buffer used to exchange data written as sequence of bytes.
        //  * @param int slave this is the position of slave thaht has to comunication in the network
        //  * @param int dim this is the dimension of data to read.
        //  * @param int offset the byte to start writing the data
        // */
        // void read_slave(uint8 *array, int slave, int dim, int offset);

        /**
         * It sets the time of cycle used by a real-time thread to exchange ethercat package thorow IOmap buffer.
         * @param uint64 cycletime this is the time of cycle to send ethercat package.
        */
        void setCycle(int64 cycletime);

        void stampa();

        void waitThread();

        uint8 *getOutput_slave(uint16 position);

        uint8 *getInput_slave(uint16 position);

        /**
         * Each slave has to call this method to access the IObuffer.
        */
        void mutex_down();

        /**
         * Each slave has to call this method to release the IObuffer.
        */
        void mutex_up();
    };

} // namespace sun

#endif