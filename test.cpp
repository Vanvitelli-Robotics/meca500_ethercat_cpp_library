#include "Robot.hpp"

int main(int argc, char *argv[]) {
    Robot robot(0,5000,"eth0",0.0,100);
    robot.reset_error();
    //robot.main();
    robot.set_conf(1,1,-1);
    robot.move_pose(0,-240,190,90,0,0);
    robot.print_pose();
    while(true) {
        static float speed = 500;
        speed *= -1;
        robot.move_lin_vel_trf(speed);
        usleep(0.1e+6);
    }
}
