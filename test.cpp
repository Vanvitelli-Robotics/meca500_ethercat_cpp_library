#include "Robot.hpp"

int main(int argc, char *argv[]) {
    Robot robot(0,1000,"eno1");
    robot.reset_error();
    robot.activate();
    robot.home();
    robot.set_conf(1,1,-1);
    robot.move_pose(0,-240,190,90,0,0);
    robot.print_pose();
    while(true) {
        static float speed = 20;
        speed *= -1;
        robot.move_lin_vel_trf(speed);
        usleep(2e+6);
    }
}