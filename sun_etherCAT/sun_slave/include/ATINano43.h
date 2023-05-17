#ifndef ATINANO43_H
#define ATINANO43_H

//IP Address: 192.168.2.120
#define STOP_STREAMING 0x0000
#define START_REALTIME 0x0002
#define START_BUFFERED 0x0003
#define RDT_RECORD_SIZE 36
#define STATUS_OK 0x00000000

#include "Master.h"

namespace sun
{
    namespace EtherCAT
    {

        class ATINano43
        {
        private:
            //uint16 START_REALTIME=0x0002;
            typedef struct PACKED
            {
                uint32 rdt_sequence; /** RDT sequence number of this packet*/
                uint32 ft_sequence;  /** The record's internal sequence number*/
                uint32 status;       /** System status code*/
                int32 fx;            /** X-axis force */
                int32 fy;            /** y-axis force */
                int32 fz;            /** z-axis force */
                int32 tx;            /** X-axis torque */
                int32 ty;            /** y-axis torque */
                int32 tz;            /** z-axis torque */
            } RDTRecord_t;

            typedef struct PACKED
            {
                uint32 sample_count = 0; /** Samples to output (0=infinite)*/
                uint16 command;          /** Command to execute*/

            } RDTRequest_t;

            RDTRecord_t *RDTRecord;
            RDTRequest_t *RDTRequest;
            Master *master;
            uint16 position;
            int64 cycletime;
            //! Scaling factor for converting raw force values from device into Newtons
            double force_scale;
            //! Scaling factor for converting raw torque values into Newton*meters
            double torque_scale;

            // TODO : Get/Set Force/Torque scale for device
            // Force/Sclae is based on counts per force/torque value from device
            // these value are manually read from device webserver, but in future they
            // may be collected using http get requests
            double counts_per_force = 1000000;
            double counts_per_torque = 1000000000;
            
            

        public:
            /** Constructor*/
            ATINano43(uint16 position, Master *m, int64 cycletime);

            /**
             * Start high-speed real-time streaming. 
             * Real-time response applications.
             *  
            */
            void start_realtime();

            /**
             * Start high-speed buffered streaming: collecting data at high speed, but not 
             * responding to it in real-time.
             *  
            */
            void start_buffered();

            /**
             * Stop streaming
            */
            void stop();

            /**
             * Receive forces array.
             * @param double *forces: array contains measured forces.
            */
            void getForces(double *forces);

            /**
             * Receive torques array.
             * @param double *torques: array contains measured torques.
            */
            void getTorques(double *torques);

            /**
             * Receive status array
             * @param uint32 *status: array contains info about the status.
            */
            void getStatus(uint32 *status);

            /**
             * This method must be used to assign a pointer to a struct. It must be called before the first command.
            */
            void assign_pointer_struct();

            /**
             * This methods returns the position of the slave in the network.
             * @return uint16 position of slave in the network.
            */ 
            uint16 getPosition();
        };

    } // namespace EtherCAT

} //namespace sun

#endif