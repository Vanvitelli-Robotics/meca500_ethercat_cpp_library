#ifndef MECA500_H
#define MECA500_H

#include "Master.h"
#include <vector>
#include <cstring>

#define EC_TIMEOUT_INIT_TO_PRE_OP 2000000 /** TIMEOUT_STATE INIT-> PRE_OP */
#define EC_TIMEOUT_TO_SAFE_OP 9000000     /** TIMEOUT_STATE PRE_OP -> SAFE_OP ; SAFE_OP -> OP */
#define EC_TIMEOUT_PRE_OP_TO_INIT 5000000 /** TIMEOUT_STATE PRE_OP -> INIT */
#define EC_TIMEOUT_OP_TO_SAFE_OP 200000   /** TIMEOUT_STATE OP -> SAFE_OP */

namespace sun
{

    class Meca500
    {
    private:
        /** Struct of entries of Object Dictionary */
        typedef struct
        {
            int index;
            int sub_index;
            int size;
            int value;
        } OBentry;

        typedef struct PACKED
        {
            uint16 status_bits; //bit1=busy,2=activated,3=homed,4=simactivated, altri a 0
            uint16 error;
        } robot_statust;

        typedef struct PACKED
        {
            uint32 checkpoint;
            uint16 move_id;
            uint16 FIFO_space;
            uint32 motion_bits; //bit=paused,2=EOB,3=EOM,4=FIFO_cleared,5=PStop
        } motion_statust;

        typedef struct PACKED
        {
            float joint_angle_1;
            float joint_angle_2;
            float joint_angle_3;
            float joint_angle_4;
            float joint_angle_5;
            float joint_angle_6;
        } angular_positiont;

        typedef struct PACKED
        {
            float x;     //[mm]
            float y;     //[mm]
            float z;     //[mm]
            float alpha; //[degrees]
            float beta;  //[degrees]
            float gamma; //[degrees]
        } cartesian_positiont;

        typedef struct PACKED
        {
            float joint_speed_1;
            float joint_speed_2;
            float joint_speed_3;
            float joint_speed_4;
            float joint_speed_5;
            float joint_speed_6;
        } angular_velocitiest;

        typedef struct PACKED
        {
            float joint_torque_1;
            float joint_torque_2;
            float joint_torque_3;
            float joint_torque_4;
            float joint_torque_5;
            float joint_torque_6;
        } torque_ratiost;

        typedef struct PACKED
        {
            int X;
            int Y;
            int Z;
        } accelerometert;

        typedef struct PACKED
        {
            int8 c1;
            int8 c3;
            int8 c5;
        } configurationt;

        /**struct of outputs of MECA500 (Tx PDO)*/
        typedef struct PACKED
        {
            //robot_statust robot_status;
            uint16 status_bits; //bit1=busy,2=activated,3=homed,4=simactivated, altri a 0
            uint16 error;
            motion_statust motion_status;
            angular_positiont angular_position;
            cartesian_positiont cartesian_position;
            angular_velocitiest angular_velocities;
            torque_ratiost torque_ratio;
            accelerometert accelerometer;
            configurationt configuration;

        } out_MECA500t;

        typedef union PACKED
        {
            float varf[6];
            uint8 varb;
        } variablest;

        typedef struct PACKED
        {
            uint32 motion_command;
            variablest variables;
        } movementt;

        typedef struct PACKED
        {
            uint32 robot_control_data;
        } robot_control_t;

        typedef struct PACKED
        {
            uint16 moveID;
            uint16 motion_control_data;

        } motion_control_t;

        /**struct of inputs of MECA500 (Rx PDO)*/
        typedef struct PACKED
        {
            uint32 robot_control_data;
            //robot_control_t robot_control;
            motion_control_t motion_control;
            movementt movement;
        } in_MECA500t;

        in_MECA500t *in_MECA500;
        out_MECA500t *out_MECA500;
        Master *master;
        uint16 position;
        uint32 cycletime;

    public:
        /**
         * meca_vector is a vector of all meca500 in the network
        */
        static std::vector<Meca500 *> meca_vector;

        /**
         * Constructor
         * @param Master* m pointer to master
         * @param uint16 position this is the position of the slave in the network
         * @param uint32 cycletime this is the specific time of cycle of each slave. It's set to 1000000 as default value. 
        */
        Meca500(uint16 position, Master *m, uint32 cycletime = 1000000);

        /**
         * Destructor: delete a slave from vector
        */
        ~Meca500();

        /**
        * This method returns the position of the slave in the network.
        * @return uint16 position
        */
        uint16 getPosition();

        /**
         * This method returns the cycletime of the slave.
         * @return uint32 cycletime
        */
        uint32 getCycletime();

        /**
         * This methods returns the measured velocities of joints throw a float array.
         * @param float *joint_velocities: array contains the measured velocities.
        */
       void getJointsVelocities(float *joint_velocities);
     

        /**
         * This method sets the cycletime of the slave.
         * @param uint32 cycletime
        */
        void setCycletime(uint32 cycletime);

        /**
         * This method sets up the slave.
         * @param uint16 position this is the position of the slave in the network
         * @return int 0 as positive  
        */
        int setup(uint16 position);

        /**
         * It is used to call the setup method.
         * @param uint16 position this is the position of the slave in the network
         * @return a pointer to setup method of the slave.
        */
        static int setup_static(uint16 position);

        /**
         * Allows to slave to send and reiceve data. It has to be called only once after the mapping of the slave.
        */
        void assign_pointer_struct();

        /*
        #############################################################################
         #
         #                          Request commands
         #
        ###############################################################################
        */

        /**
         * This command activates all motors and disables the brakes on joints 1,2 and 3.
         * It must be sent before homing is started.
         * This command only works if the robot is idle.
         * @return int 0 = Motors activated; 
         * @return int 1 = Motors already activated; 
         * @return int 2 = Timeout exceeded;
        */
        int activateRobot();

        /**
         * Activate simulation mode.
         * @return int 0 = Simulation mode is enabled. 
         * @return int 2 = Timeout exceeded.
         * @return int -1 = Motors must be deactivated.
        */
        int activateSim();

        /**
         * Deactivate simulation mode.
         * @return int 0 = Simulation mode is deactivate.
         * @return int 2 = Timeout exceeded.
        */
        int deactivateSim();

        /**
        * This command disables all motors and enages the breaks on joints 1, 2 and 3.
        * It must not be sent while the robot is moving.
        * This command should be run before powering down the robot.
        * When this command is executed, the robot loses its homing. The homing process must be repeated after reactivating the robot.
        * @return int 0 = Motors are disabled; 
        * @return int 2 = Timeout exceeded;
        */
        int deactivateRobot();

        /**
        * This command stops the robot movement.
        * If the robot is stopped in the middle of a trajectory, the resto of the trajectory is deleted.
        * You need to send the commande 'ResumeMotion' to make the robot ready to execute new motion commands.
        * @return int 0 = The motion was cleared;
        * @return int 2 = Timeout exceeded.
        */
        int clearMotion();

        /**
         * Thic ommand returns the robot current inverse kinematica configuration, not the desired one, if any, set by the command SetConf.
         * @param int8* array_c this is the vector that contains the values requested.
        */
        void getConf(int8 *array_c);

        /**
        * This command returns the robot joint angles in degrees.
        * @param float* joint_angles: this is the array will contain the joint angles requested.
        */
        void getJoints(float *joint_angles);

        /**
        * This command returns the current pose of the robot TRF with respect to the WRF.
        * @param float* pose: this is the array will contain the values requested.
        */
        void getPose(float *pose);

        /**
        * This command returns the status of the robot.
        * @param bool& as: activation state
        * @param bool& hs: homing state
        * @param bool& sm: simulation mode
        * @param bool& es: error status
        * @param bool& pm: pause motion status
        * @param bool& eob: end of block status
        * @param bool& eom: end of movement status
        */
        void getStatusRobot(bool &as, bool &hs, bool &sm, bool &es, bool &pm, bool &eob, bool &eom);

        /**
        * This command starts the robot and gripper homing process. 
        * This command takes about four seconds to execute.
        * @return int 0 = Homing done;
        * @return int 1 = Homing already done; 
        * @return int 2 = Timeout exceeded; 
        * @return int -1 = Motors must be activated to do home.
        */
        int home();

        /**
        * This command stops the robot movement.
        * The robot stops by decelerating, and not by applying the breakes.
        * @return int 0 = Motion paused.
        * @return int 2 = Timeout exceeded.
        */
        int pauseMotion();

        /**
        * This command resets the robot error status. 
        * @return int 0 = Error was reset. 
        * @return int 1 = There was no error to reset.
        * @return int 2 = Timeout exceeded.
        */
        int resetError();

        /**
        * This command resumes the robot movement, if it was previously paused with the command PauseMotion.
        * This command will not work if the robot was deactivate after the last time the command PauseMotion was used, if the robot is in an error mode, 
        * or if the rest of the trajectory was deleted usign the ClearMotion command.
        * @return int 0 = Motion resumed.
        * @return int 2 = Timeout exceeded.
        */
        int resumeMotion();

        /**
        * The user could enable or disable the messages wich a command sends.
        * @param int e it is 1=enabled or 0=disable. The default value is 1.
        */
        //void setEOB(int e = 1);

        /**
        * The user could enable or disable the messages wich a command sends.
        * @param int e it is 1=enabled or 0=disable. The default value is 1.
        */
        //void setEOM(int e = 1);

        /**
        * This commmand will disable the EtherCAT protocol and enable Ethernet.
        */
        void switchToEthernet();

        /*
         #######################################################################
         #
         #                      Motion commands
         # 
         #######################################################################
        */

        /** 
         * This command must be called before calling the first motion command. It must be the first operation to move robot.
         * Has to be set to 1 for information to be send to the robot.
         * @param int x = 1 -> enable to send command; x = 0 -> disable to send command.
         * @return int -1 = Invalid input;
         * @return int 0 = Operation succeeded.
        */
        int setPoint(int x);

        /**
         * This command sets the move ID.
         * @param uint16 moveID: a distinct ID number associated with each motion command. 
        */
        void setMoveID(uint16 moveID);

        /**
         * This command cleans the motion command.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode. 
        */
        void resetMotion(uint16 moveID = 0);

        /**
        * This command makes the robot move simultaneously its joints to the specified joint set.
        * All joint rotations start and stop simultaneously.
        * @param float theta_i the angle of joint i (i=1,...,6) in degrees
        * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void moveJoints(float *theta, uint16 moveID = 0);

        /**
         * This command makes the robot rotate simultaneously its joints with the specified joint velocities.
         * @param float omega_i: the velocity of joint i (i=1,...,6) in degrees/sec.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void moveJointsVel(float *omega, uint16 moveID = 0);

        /**
        * This command makes the robot move its TRF to a specific pose with respect to the WRF
        * @param float pose[0], pose[1], pose[2] are respectively x,y,z coordinates of the origin of the TRF w.r.t. the WRF in mm
        * @param float pose[3], pose[4], pose[5] are respectively alpha, beta, gamma the Euler angles representing the orientation 
        * of the TRF w.r.t. the WRF in degrees.
        * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void moveLin(float *pose, uint16 moveID = 0);

        /**
        * This command is similar to the MoveLin command but allows a desired pose to be specified relative to the current pose of the TRF.
        * @param float pose[0], pose[1], pose[2] are respectively x,y,z the position coordinates, in mm.
        * @param float pose[3], pose[4], pose[5] are respectively alpha, beta, gamma the Euler angles, in degrees.
        * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void moveLinRelTRF(float *pose, uint16 moveID = 0);

        /**
         * This command is similar to the MoveLinRelTRF command, but it is defined with respect to a preference frame that has the same orientation as the WRF.
         * @param float pose[0], pose[1], pose[2] are respectively x,y,z the position coordinates, in mm.
         * @param float pose[3], pose[4], pose[5] are respectively alpha, beta, gamma the Euler angles, in degrees.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void moveLinRelWRF(float *pose, uint16 moveID = 0);

        /**
         * This command makes the robot move its TRF with the specified instantaneous Cartesian velocity, defined with respect to the TRF.
         * @param float pose[0], pose[1], pose[2] are respectively x_p,y_p,z_p, the components of the instantaneous linear velocity of the TCP w.r.t. the TRF, in mm/s, ranging
         * from -1000 mm/s to 1000 mm/s.
         * @param float pose[3], pose[4], pose[5] are respectively w_x, w_y, w_z, the components of the instantaneous angular velocity of the TRF w.r.t. the TRF, in °/s,
         * ranging from -300°/s to300°/s.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
       void moveLinVelTRF(float *pose, uint16 moveID = 0); 

       /**
         * This command makes the robot move its TRF with the specified instantaneous Cartesian velocity, defined with respect to the WRF.
         * @param float pose[0], pose[1], pose[2] are respectively x_p,y_p,z_p, the components of the instantaneous linear velocity of the TCP w.r.t. the WRF, in mm/s, ranging
         * from -1000 mm/s to 1000 mm/s.
         * @param float pose[3], pose[4], pose[5] are respectively w_x, w_y, w_z, the components of the instantaneous angular velocity of the TRF w.r.t. the WRF, in °/s,
         * ranging from -300°/s to300°/s.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
       void moveLinVelWRF(float *pose, uint16 moveID = 0);

        /**
         * This command makes the robot move its TRF to a specific pose with respect to the WRF.
         * @param float pose[0], pose[1], pose[2] are respectively x,y,z the coordinates of the origin of the TRF w.r.t. the WRF, in mm
         * @param float pose[3], pose[4], pose[5] are respectively alpha, beta, gamma the Euler angles representing the orientation of the TRF w.r.t. the WRF, in degrees.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void movePose(float *pose, uint16 moveID = 0);

        /**
         * This command enables or disables the automatic robot configuration selection and has effect only on the MovePose command. 
         * @param int e e=0 means disable, e=1 means enable. SetAutoConf is enabled by default.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
         * @return int 0 = operation succeeded.
         * @return int -1 = invalid input.
        */
        int setAutoConf(int e, uint16 moveID = 0);

        /**
         * This command enables/disables the robot's blending feature.
         * @param float p percentage of blending, ranging from 0 (blending disabled) to 100%. Blending is enabled at 100% by default. 
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
         * @return int 0 = blending set.
         * @return int -1 = invalid input.
        */
        int setBlending(float p, uint16 moveID = 0);

        /**
         *This command limits the Cartesian acceleration (both the linear and the angular) of the end-effector.
         *@param float p precentage of maximum acceleration of the end-effector, ranging from 1% to 100%. The default end-effector acceleration limit is 100%.
         *@param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode. 
         * @return int 0 = operation succeeded.
         * @return int -1 = invalid input.
        */
        int setCartAcc(float p, uint16 moveID = 0);

        /**
         * This command limits the angular velocity of the robot TRF with respect to its WRF. 
         * It only affects the movements generated by the MoveLin, MoveLinRelTRF and MoveLineRelWRF commands.
         * @param float omega: TRF angular velocity limit, ranging from 0.001°/s to 180°/s. The default end-effector angular velocity limit is 45°/s.
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
         * @return int 0 = operation succeeded.
         * @return int -1 = invalid input.
        */
        int setCartAngVel(float omega, uint16 moveID = 0);

        /**
         * This command limits the Cartesian linear velocity of the robot's TRF with respect to its WRF. 
         * It only affect the movement generated by the MoveLin, MoveLinRelTRF and MoveLinRelWRF commands.
         * @param float v; velocity limit
         * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
         * @return int 0 = operation succeeded.
         * @return int -1 = invalid input.
        */
        int setCartLinVel(float v, uint16 moveID = 0);

        /**
         *This command sets the desired robot inverse kinematic configuration to be observed in the MovePose command.
         *The robot inverse kinematic configuration can be automatically selected byusing the SetAutoConf command. 
         *Using SetConf automatically disables the automatic configuration selection.
         *@param float c[0]=c1: first inverse kinematics configuration parameter, either −1 or 1
         *@param float c[1]=c3: second inverse kinematics configuration parameter, either −1 or 1
         *@param float c[2]=c5: third inverse kinematics configuration parameter, either −1 or 1
         *@param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
         *@return int 0 = operation succeeded.
         *@return int -1 = invalid input.
        */
        int setConf(float *c, uint16 moveID = 0);

        /**
        * This command limits the acceleration of the joints. Note that this command makes the robot stop, even if blending is enabled.
        * @param float p: percentage of maximum acceleration of the joints, ranging from 1% to 100%. The default joint acceleration limit is 100%
        * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        * @return int 0 = joint acceleration set.
        * @return int -1 = invalid input.         
        */
        int setJoinAcc(float p, uint16 moveID = 0);

        /**
        * This command limits the angular velocities of the robot joints. It affects the movements generated by the MovePose and MoveJoints commands.
        * @param float p: percentage of maximum joint velocities, ranging from 1 to 100%. By default, the limit is set to 25%.
        * @param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        * @return int 0 = joint velocieties set.
        * @return int -1 = invalid input.
        */
        int setJoinVel(float p, uint16 moveID = 0);

        /**
        *This command de nes the pose of the TRF with respect to the FRF. Note that this command
        *makes the robot come to a complete stop, even if blending is enabled.
        *@param float pose[0], pose[1], pose[2] are respectively x , y , and z: the coordinates of the origin of the TRF w.r.t. the FRF, in mm
        *@param float pose[3], pose[4], pose[5] are respectively α , β , and γ: the Euler angles representing the orientation of the TRF w.r.t. the FRF, in degrees.
        *By default, the TRF coincides with the FRF.
        *@param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void setTRF(float *pose, uint16 moveID = 0);

        /**
         *This command defines the pose of the WRF with respect to the BRF. Note that this com-
         *mand makes the robot come to a complete stop, even if blending is enabled.
         *@param float  pose[0], pose[1], pose[2] are respectively x , y , and z : the coordinates of the origin of the WRF w.r.t. the BRF, in mm
         *@param float  pose[3], pose[4], pose[5] are respectively α , β , and γ : the Euler angles representing the orientation of the WRF w.r.t. the BRF,in degrees.
         *@param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void setWRF(float *pose, uint16 moveID = 0);

        /**
         *This command sets the timeout after a velocity-mode motion command (MoveJointsVel,
         *MoveLinVelTRF, or MoveLinVelWRF), after which all joint speeds will be set to zero unless
         *another velocity-mode motion command is received. The SetVelTimeout command should
         *be regarded simply as a safety precaution.
         *@param float timeout: desired time interval, in seconds, ranging from 0.001 s to 1 s.
         *By default, the velocity-mode timeout is 0.05 s.
         *@param uint16 moveID = 0 (default) -> the robot works in a cyclic mode; moveID !=0 -> the robot works in one-off mode.
        */
        void SetVelTimeout(float timeout, uint16 moveID = 0);

        /**
         * This command returns the error.
         * @return int 2= Timeout exceeded;
        */
        int getError();
    };

} // namespace sun

#endif
