#include "Meca500.h"
#include <cstring>
#include <cstddef>
#include <stdexcept>

#define TIMEOUT_RQ 200000

#define SET_BIT(prev, bit) (prev | (0x0ff & bit))
#define CLEAR_BIT(prev, bit) (prev & (0x0ff & (~bit)))
#define GET_BIT(mask, value) (mask & value)

namespace sun
{
    std::vector<Meca500 *> Meca500::meca_vector;

    Meca500::Meca500(uint16 p, Master *m, uint32 cycletime)
    {
        master = m;
        position = p;
        meca_vector.push_back(this);
        this->cycletime = cycletime;
    }

    Meca500::~Meca500()
    {
        //distruttore
        int i = 0;
        while (this->position != meca_vector[i]->position && i < meca_vector.size())
        {
            i++;
        }
        if (i >= meca_vector.size())
            throw std::runtime_error("");
        meca_vector.erase(meca_vector.begin() + i);
    }

    uint16 Meca500::getPosition()
    {
        return position;
    }

    uint32 Meca500::getCycletime()
    {
        return this->cycletime;
    }

    void Meca500::setCycletime(uint32 cycletime)
    {
        this->cycletime = cycletime;
    }

    int Meca500::setup(uint16 slave)
    {
        int retval;
        //disattivo i PDO in inviati allo slave
        OBentry TXPDO_number = {0x1c13, 0x00, sizeof(uint8), 0};
        OBentry RXPDO_number = {0x1c12, 0x00, sizeof(uint8), 0};

        printf("\n---DATI DA SCRIVERE---\n");
        retval = ec_SDOwrite(slave, TXPDO_number.index, TXPDO_number.sub_index,
                             FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
        printf("sub_index=%d,value=%x,esito=%d\n\n", TXPDO_number.sub_index, TXPDO_number.value,
               retval);

        retval = ec_SDOwrite(slave, RXPDO_number.index, RXPDO_number.sub_index,
                             FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
        printf("sub_index=%d,value=%x,esito=%d\n\n", RXPDO_number.sub_index, RXPDO_number.value,
               retval);
        //mappo tutti i PDO
        OBentry TXPDO = {0x1c13, 0x01, sizeof(uint16), 0x1a00};
        OBentry RXPDO = {0x1c12, 0x01, sizeof(uint16), 0x1600};

        //posso assegnarli così perchè gli oggetti in cui è descritta la mappatura
        //sono consecutivi
        while (TXPDO.value <= 0x1a08)
        {
            if (TXPDO.value != 0x1a07)
            {
                retval = ec_SDOwrite(slave, TXPDO.index, TXPDO.sub_index, FALSE, TXPDO.size,
                                     &(TXPDO.value), EC_TIMEOUTSAFE);
                printf("sub_index=%d,value=%x,esito=%d\n", TXPDO.sub_index, TXPDO.value, retval);
                TXPDO_number.value++;
                TXPDO.value++;
                TXPDO.sub_index++;
            }

            else
                TXPDO.value++;
        }

        //abilito i PDO

        retval = ec_SDOwrite(slave, TXPDO_number.index, TXPDO_number.sub_index,
                             FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
        printf("esito_numero=%d\n", retval);

        //controllo che siano memorizzati i dati corretti
        printf("\n---DATI LETTI---\n");
        for (int i = 0x1; i <= TXPDO_number.value; i++)
        {
            retval = ec_SDOread(slave, TXPDO.index, i, FALSE, &(TXPDO.size),
                                &(TXPDO.value), EC_TIMEOUTSAFE);
            printf("sub_index=%d,value=%x,esito=%d\n", i, TXPDO.value, retval);
        }

        while (RXPDO.value <= 0x1602)
        {
            retval = ec_SDOwrite(slave, RXPDO.index, RXPDO.sub_index, FALSE, RXPDO.size,
                                 &(RXPDO.value), EC_TIMEOUTSAFE);
            printf("sub_index=%d,value=%x,esito=%d\n", RXPDO.sub_index, RXPDO.value, retval);
            if (retval > 0)
            {
                RXPDO_number.value++;
                RXPDO.value++;
                RXPDO.sub_index++;
            }
        }

        //abilito i PDO
        retval = ec_SDOwrite(slave, RXPDO_number.index, RXPDO_number.sub_index,
                             FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
        printf("esito_numero=%d\n", retval);
        //ec_dcsync0(slave, TRUE, cycletime, cycletime/2);

        return 0;
    }

    int Meca500::setup_static(uint16 position)
    {
        int i = 0;
        while (position != meca_vector[i]->getPosition() && i < meca_vector.size())
        {
            i++;
        }
        if (i >= meca_vector.size())
            throw std::runtime_error("Error setup_Slave\n");
        return meca_vector[i]->setup(position);
    }

    void Meca500::assign_pointer_struct()
    {
        in_MECA500 = (in_MECA500t *)master->getOutput_slave(position);
        out_MECA500 = (out_MECA500t *)master->getInput_slave(position);
    }

    void Meca500::getJointsVelocities(float *joint_velocities)
    {
        master->mutex_down();
        joint_velocities[0] = out_MECA500->angular_velocities.joint_speed_1;
        joint_velocities[1] = out_MECA500->angular_velocities.joint_speed_2;
        joint_velocities[2] = out_MECA500->angular_velocities.joint_speed_3;
        joint_velocities[3] = out_MECA500->angular_velocities.joint_speed_4;
        joint_velocities[4] = out_MECA500->angular_velocities.joint_speed_5;
        joint_velocities[5] = out_MECA500->angular_velocities.joint_speed_6;
        master->mutex_up();
    }
    

    /*
    *                               Request commands
    */

    int Meca500::activateRobot()
    {
        int time = 0;
        master->mutex_down();
        if (GET_BIT(0x02, out_MECA500->status_bits == 2))
        {
            master->mutex_up();
            return 1; //Motors already activated
        }
        else
        {
            time = 0;
            in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x02);

            while (GET_BIT(0x02, out_MECA500->status_bits) != 2 && time < TIMEOUT_RQ)
            {
                master->mutex_up();
                usleep(10);
                time++;
                master->mutex_down();
            }
            if (GET_BIT(0x02, out_MECA500->status_bits) != 2 && time == TIMEOUT_RQ)
            {
                master->mutex_up();
                return getError(); //timeout exceeded
            }
            else
            {
                master->mutex_up();
                return 0; //Motors activated
            }
        }
    }

    int Meca500::activateSim()
    {
        //To enable the simulation mode, the robot must be deactivated first.
        int time = 0;
        if (deactivateRobot() != 0)
            return -1;

        master->mutex_down();
        in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x10);
        while (GET_BIT(0x08, out_MECA500->status_bits) != 8 && time < TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //Simulation mode is enabled.
    }

    int Meca500::deactivateSim()
    {
        int time = 0;
        master->mutex_down();
        in_MECA500->robot_control_data = CLEAR_BIT(in_MECA500->robot_control_data, 0x10);
        while (GET_BIT(0x08, out_MECA500->status_bits) != 8 && time < TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //Simulation mode is disabled
    }

    int Meca500::deactivateRobot()
    {
        int time = 0;

        master->mutex_down();
        //set activate_in to 0
        in_MECA500->robot_control_data = CLEAR_BIT(in_MECA500->robot_control_data, 0x02);
        //set home_in to 0
        in_MECA500->robot_control_data = CLEAR_BIT(in_MECA500->robot_control_data, 0x04);
        //set deactivate to 1
        in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x01);

        while (GET_BIT(out_MECA500->status_bits, 0x02) == 2 && time < TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //Motors are disabled.
    }

    int Meca500::home()
    {
        int time = 0;
        //Verify homing already done
        master->mutex_down();
        if (GET_BIT(0x04, out_MECA500->status_bits) == 4)
        {
            master->mutex_up();
            return 1; //Homing already done
        }
        else
        {
            //Verify if the robot is activated
            if (GET_BIT(0x02, out_MECA500->status_bits) == 2)
            {
                //enable homing
                in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x04);
                while (GET_BIT(0x04, out_MECA500->status_bits) != 4 && time < TIMEOUT_RQ)
                {
                    master->mutex_up();
                    usleep(10);
                    time++;
                    master->mutex_down();
                }
                master->mutex_up();
                if (time == TIMEOUT_RQ)
                {
                    return getError(); //Timeout exceeded!
                }
                else
                    return 0; //Homing done
            }
            else
            {
                return -1; //Motors must be activated to do home.
            }
        }
    }

    int Meca500::resetError()
    {
        int time = 0;
        master->mutex_down();
        if (out_MECA500->error == 0)
        {
            master->mutex_up();
            return 1; //There was no error to reset
        }
        else
        {
            in_MECA500->robot_control_data = SET_BIT(in_MECA500->robot_control_data, 0x08);
            while (out_MECA500->error != 0 && time < TIMEOUT_RQ)
            {
                master->mutex_up();
                time++;
                master->mutex_down();
            }
            master->mutex_up();

            if (time == TIMEOUT_RQ)
                return getError(); //Timeout exceeded!
            else
                return 0; //The error was reset
        }
    }

    int Meca500::clearMotion()
    {
        int time = 0;
        master->mutex_down();
        in_MECA500->motion_control.motion_control_data = SET_BIT(in_MECA500->motion_control.motion_control_data, 0x04);
        while (GET_BIT(0x08, out_MECA500->motion_status.motion_bits) != 8 && time <= TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //The motion was cleared
    }

    void Meca500::getConf(int8 *array_c)
    {
        master->mutex_down();
        array_c[0] = out_MECA500->configuration.c1;
        array_c[1] = out_MECA500->configuration.c3;
        array_c[2] = out_MECA500->configuration.c5;
        master->mutex_up();
    }

    void Meca500::getJoints(float *joint_angles)
    {
        master->mutex_down();
        joint_angles[0] = out_MECA500->angular_position.joint_angle_1;
        joint_angles[1] = out_MECA500->angular_position.joint_angle_2;
        joint_angles[2] = out_MECA500->angular_position.joint_angle_3;
        joint_angles[3] = out_MECA500->angular_position.joint_angle_4;
        joint_angles[4] = out_MECA500->angular_position.joint_angle_5;
        joint_angles[5] = out_MECA500->angular_position.joint_angle_6;
        master->mutex_up();
    }

    void Meca500::getPose(float *pose)
    {
        master->mutex_down();
        pose[0] = out_MECA500->cartesian_position.x;
        pose[1] = out_MECA500->cartesian_position.y;
        pose[2] = out_MECA500->cartesian_position.z;
        pose[3] = out_MECA500->cartesian_position.alpha;
        pose[4] = out_MECA500->cartesian_position.beta;
        pose[5] = out_MECA500->cartesian_position.gamma;
        master->mutex_up();
    }

    void Meca500::getStatusRobot(bool &as, bool &hs, bool &sm, bool &es, bool &pm, bool &eob, bool &eom)
    {
        master->mutex_down();
        if (GET_BIT(0x02, out_MECA500->status_bits) == 2)
            as = true;
        else
            as = false;

        if (GET_BIT(0x04, out_MECA500->status_bits) == 4)
            hs = true;
        else
            hs = false;

        if (GET_BIT(0x08, out_MECA500->status_bits) == 8)
            sm = true;
        else
            sm = false;

        if (out_MECA500->error != 0)
            es = true;
        else
            es = false;

        if (GET_BIT(0x01, out_MECA500->motion_status.motion_bits) == 1)
            pm = true;
        else
            pm = false;

        if (GET_BIT(0x02, out_MECA500->motion_status.motion_bits) == 2)
            eob = true;
        else
            eob = false;

        if (GET_BIT(0x04, out_MECA500->motion_status.motion_bits) == 4)
            eom = true;
        else
            eom = false;
        master->mutex_up();
    }

    int Meca500::pauseMotion()
    {
        int time = 0;
        master->mutex_down();
        in_MECA500->motion_control.motion_control_data = SET_BIT(in_MECA500->motion_control.motion_control_data, 0x02);
        while (GET_BIT(0x01, out_MECA500->motion_status.motion_bits) != 1 && time < TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //Motion paused.
    }

    //Must be called after clear motion and reset error commands.
    int Meca500::resumeMotion()
    {
        int time = 0;
        master->mutex_down();
        in_MECA500->motion_control.motion_control_data = CLEAR_BIT(in_MECA500->motion_control.motion_control_data, 0x02);
        in_MECA500->motion_control.motion_control_data = CLEAR_BIT(in_MECA500->motion_control.motion_control_data, 0x04);
        while ((GET_BIT(0x01, out_MECA500->motion_status.motion_bits) != 1 || GET_BIT(0x08, out_MECA500->motion_status.motion_bits) != 8) && time < TIMEOUT_RQ)
        {
            master->mutex_up();
            usleep(10);
            time++;
            master->mutex_down();
        }
        master->mutex_up();
        if (time == TIMEOUT_RQ)
        {
            return getError(); //Timeout exceeded!
        }
        else
            return 0; //Motion resumed.
    }

    // void Meca500::switchToEthernet()
    // {
    //      retval = ec_SDOwrite(this->position, TXPDO_number.index, TXPDO_number.sub_index,
    //                          FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
    // }

    /*
    *                               Motion commands
    * WARNING: call getError() after each motion command to verify if there is an error
    */

    int Meca500::setPoint(int x)
    {
        if (x == 1)
        {
            master->mutex_down();
            in_MECA500->motion_control.motion_control_data = SET_BIT(in_MECA500->motion_control.motion_control_data, 0x01);
            master->mutex_up();
        }
        else if (x == 0)
        {
            master->mutex_down();
            in_MECA500->motion_control.motion_control_data = CLEAR_BIT(in_MECA500->motion_control.motion_control_data, 0x01);
            master->mutex_up();
        }
        else
        {
            return -1;
        }
        return 0;
    }

    void Meca500::setMoveID(uint16 moveID)
    {
        master->mutex_down();
        in_MECA500->motion_control.moveID = moveID;
        master->mutex_up();
    }

    void Meca500::resetMotion(uint16 moveID)
    {
        setMoveID(moveID);
        master->mutex_down();
        in_MECA500->movement.motion_command = 0;
        in_MECA500->movement.variables.varf[0] = 0;
        in_MECA500->movement.variables.varf[1] = 0;
        in_MECA500->movement.variables.varf[2] = 0;
        in_MECA500->movement.variables.varf[3] = 0;
        in_MECA500->movement.variables.varf[4] = 0;
        in_MECA500->movement.variables.varf[5] = 0;
        master->mutex_up();
    }

    void Meca500::moveJoints(float *theta, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 1;
        in_MECA500->movement.variables.varf[0] = theta[0];
        in_MECA500->movement.variables.varf[1] = theta[1];
        in_MECA500->movement.variables.varf[2] = theta[2];
        in_MECA500->movement.variables.varf[3] = theta[3];
        in_MECA500->movement.variables.varf[4] = theta[4];
        in_MECA500->movement.variables.varf[5] = theta[5];
        master->mutex_up();
    }

    void Meca500::movePose(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 2;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::moveLin(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 3;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::moveLinRelTRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 4;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::moveLinRelWRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 5;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    

    int Meca500::setBlending(float p, uint16 moveID)
    {
        setMoveID(moveID);
        if (p > 100 || p < 0)
            return -1; //Error value of percentage! The value must be 0<p<100
        else
        {
            //0<p<100
            master->mutex_down();
            in_MECA500->movement.motion_command = 7;
            in_MECA500->movement.variables.varf[0] = p;
            master->mutex_up();
            return 0;
        }
    }

    int Meca500::setJoinVel(float p, uint16 moveID)
    {
        setMoveID(moveID);
        if (p > 100 || p < 0)
            return -1; //Error value of percentage! The value must be 0<p<100
        else
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 8;
            in_MECA500->movement.variables.varf[0] = p;
            master->mutex_up();
            return 0;
        }
    }

    int Meca500::setJoinAcc(float p, uint16 moveID)
    {
        //NEL MANUALE RISULTA P<600 ??????
        setMoveID(moveID);
        if (p > 150 || p < 0)
            return -1; //Error value of percentage! The value must be 0<p<100
        else
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 9;
            in_MECA500->movement.variables.varf[0] = p;
            master->mutex_up();
            return 0;
        }
    }

    int Meca500::setCartAngVel(float omega, uint16 moveID)
    {
        setMoveID(moveID);
        if (omega > 300 || omega < 0.001)
            return -1; //Error value of percentage! The value must be 0.001<p<300.
        else
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 10;
            in_MECA500->movement.variables.varf[0] = omega;
            master->mutex_up();
            return 0;
        }
    }

    int Meca500::setCartLinVel(float v, uint16 moveID)
    {
        setMoveID(moveID);
        if (v > 1000 || v < 0.001)
            return -1; //Error value of percentage! The value must be 0.001<p<1000.
        else
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 11;
            in_MECA500->movement.variables.varf[0] = v;
            master->mutex_up();
            return 0;
        }
    }

    int Meca500::setCartAcc(float p, uint16 moveID)
    {
        setMoveID(moveID);
        if (p > 600 || p < 0.001)
            return -1; //Error value of percentage! The value must be 0.001<p<100.
        else
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 12;
            in_MECA500->movement.variables.varf[0] = p;
            master->mutex_up();
            return 0;
        }
    }

    void Meca500::setTRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);
        master->mutex_down();
        in_MECA500->movement.motion_command = 13;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::setWRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);
        master->mutex_down();
        in_MECA500->movement.motion_command = 14;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    int Meca500::setConf(float *c, uint16 moveID)
    {
        setMoveID(moveID);
        int i = 0;
        master->mutex_down();
        in_MECA500->movement.motion_command = 15;
        for (i = 0; i < 3; i++)
        {
            if (c[i] == 1 || c[i] == -1)
                in_MECA500->movement.variables.varf[i] = c[i];
            else
                i = 4;
        }
        master->mutex_up();
        if (i == 4)
            return -1; //Error values of parameter c! They must be 1 or -1.
        return 0;
    }

    int Meca500::setAutoConf(int e, uint16 moveID)
    {
        setMoveID(moveID);
        if (e == 1 || e == -1)
        {
            master->mutex_down();
            in_MECA500->movement.motion_command = 16;
            in_MECA500->movement.variables.varf[0] = e;
            master->mutex_up();
            return 0;
        }
        else
            return -1; //Error: values of parameter e! They must be 1 or -1.
    }

    void Meca500::moveJointsVel(float *omega, uint16 moveID)
    {
        setMoveID(moveID);
        master->mutex_down();
        in_MECA500->movement.motion_command = 21;
        in_MECA500->movement.variables.varf[0] = omega[0];
        in_MECA500->movement.variables.varf[1] = omega[1];
        in_MECA500->movement.variables.varf[2] = omega[2];
        in_MECA500->movement.variables.varf[3] = omega[3];
        in_MECA500->movement.variables.varf[4] = omega[4];
        in_MECA500->movement.variables.varf[5] = omega[5];
        master->mutex_up();
    }

    void Meca500::moveLinVelWRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 22;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::moveLinVelTRF(float *pose, uint16 moveID)
    {
        setMoveID(moveID);

        master->mutex_down();
        in_MECA500->movement.motion_command = 23;
        in_MECA500->movement.variables.varf[0] = pose[0];
        in_MECA500->movement.variables.varf[1] = pose[1];
        in_MECA500->movement.variables.varf[2] = pose[2];
        in_MECA500->movement.variables.varf[3] = pose[3];
        in_MECA500->movement.variables.varf[4] = pose[4];
        in_MECA500->movement.variables.varf[5] = pose[5];
        master->mutex_up();
    }

    void Meca500::SetVelTimeout(float timeout, uint16 moveID)
    {
        setMoveID(moveID);
        master->mutex_down();
        in_MECA500->movement.motion_command = 24;
        in_MECA500->movement.variables.varf[0] = timeout;
        master->mutex_up();
    }

    int Meca500::getError()
    {
        master->mutex_down();
        switch (out_MECA500->error)
        {
        case (0):
            return 2;
            break;
        case (1000):
            throw std::runtime_error("1000: Command buffer is full.\n");
            break;
        case (1001):
            throw std::runtime_error("1001: Unknown or empty command.\n");
            break;
        case (1002):
            throw std::runtime_error("1002: A parenthesis or a comma has been omitted.\n");
            break;
        case (1003):
            throw std::runtime_error("1003: Wrong number of arguments or invalid input (e.g. the argument is out of range).\n");
            break;
        case (1005):
            throw std::runtime_error("1005: The robot must be activated.\n");
            break;
        case (1006):
            throw std::runtime_error("1006: Homing must be done.\n");
            break;
        case (1007):
            throw std::runtime_error("1007: Joint over limit: the robot cannot reach the joint set or pose requested because of its joint limits.\n");
            break;
        case (1011):
            throw std::runtime_error("1011: The robot is in error mode and cannot process other commands until s resetError command is send.\n");
            break;
        case (1012):
            throw std::runtime_error("1012: Singularity detected.\n");
            break;
        case (1013):
            throw std::runtime_error("1013: Activation failed. Try again\n");
            break;
        case (1014):
            throw std::runtime_error("1014: Homing procedure failed.Try again.\n");
            break;
        case (1016):
            throw std::runtime_error("1016: Pose out of reach.\n");
            break;
        case (1017):
            throw std::runtime_error("1017: Communication failed.\n");
            break;
        case (1021):
            throw std::runtime_error("1021: Deactivation failed. Something is wrong. Try again.\n");
            break;
        case (1025):
            throw std::runtime_error("1025: Impossible to reset the error.\n");
            break;
        case (1027):
            throw std::runtime_error("1027: Simulation mode can only be enabled/disabled while the robot is deactivated.\n");
            break;
        default:
            throw std::runtime_error("Default error\n");
            break;
        }
        master->mutex_up();
    }

} // namespace sun