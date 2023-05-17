#include "ATINano43.h"
#define TIMEOUT 1000000 //us
#define SLEEP_TIME 100  //us

namespace sun
{
    namespace EtherCAT
    {
        ATINano43::ATINano43(uint16 position, Master *m, int64 cycletime)
        {
            this->position = position;
            this->master = m;
            this->cycletime = cycletime;
        }

        void ATINano43::start_realtime()
        {
            master->mutex_down();
            RDTRequest->command = START_REALTIME;
            
            int wait = 0;
            while (RDTRecord->status != STATUS_OK && wait < (TIMEOUT / SLEEP_TIME))
            {
                master->mutex_up();
                wait++;
                osal_usleep(SLEEP_TIME);
                master->mutex_down();
            }
            if (RDTRecord->status == STATUS_OK)
            {
                master->mutex_up();
                std::cout << "Starting streaming real-time...\n";
            }

            else
            {
                master->mutex_up();
                throw std::runtime_error("Error start real-time streamig\nTIMEOUT expired!\n");
            }
        }

        void ATINano43::start_buffered()
        {
            RDTRequest->command = START_BUFFERED;
            int wait = 0;
            while (RDTRecord->status != STATUS_OK && wait < (TIMEOUT / SLEEP_TIME))
            {
                wait++;
                osal_usleep(SLEEP_TIME);
            }
            if (RDTRecord->status == STATUS_OK)
                std::cout << "Starting streaming buffered...\n";
            else
                throw std::runtime_error("Error start buffered streamig\nTIMEOUT expired!\n");
        }

        void ATINano43::stop()
        {
            master->mutex_down();
            RDTRequest->command = STOP_STREAMING;
            int wait = 0;
            while (RDTRecord->status != STATUS_OK && wait < (TIMEOUT / SLEEP_TIME))
            {
                master->mutex_up();
                wait++;
                osal_usleep(SLEEP_TIME);
                master->mutex_down();
            }
            if (RDTRecord->status == STATUS_OK)
            {
                master->mutex_up();
                std::cout << "Stopping streaming...\n";
            }
            else
            {
                master->mutex_up();
                throw std::runtime_error("Error stop streamig\nTIMEOUT expired!\n");
            }
        }

        void ATINano43::getForces(double *forces)
        {
            force_scale = (double)1.0 / counts_per_force;
            master->mutex_down();
            forces[0] = double(RDTRecord->fx) * force_scale;
            forces[1] = double(RDTRecord->fy) * force_scale;
            forces[2] = double(RDTRecord->fz) * force_scale;
            master->mutex_up();
        }

        void ATINano43::getTorques(double *torques)
        {
            torque_scale = 1.0 / counts_per_torque;
            master->mutex_down();
            torques[0] = double(RDTRecord->tx) * torque_scale;
            torques[1] = double(RDTRecord->ty) * torque_scale;
            torques[2] = double(RDTRecord->tz) * torque_scale;
            master->mutex_up();
        }

        void ATINano43::getStatus(uint32 *status)
        {
            master->mutex_down();
            status[0] = RDTRecord->rdt_sequence;
            status[1] = RDTRecord->ft_sequence;
            status[2] = RDTRecord->status;
            //printf("rdt_sequence: %d\n", RDTRecord->rdt_sequence);
            master->mutex_up();
        }

        void ATINano43::assign_pointer_struct()
        {
            RDTRecord = (RDTRecord_t *)master->getInput_slave(position);
            RDTRequest = (RDTRequest_t *)master->getOutput_slave(position);
        }

        uint16 ATINano43::getPosition()
        {
            return position;
        }

    } // namespace EtherCAT
} // namespace sun