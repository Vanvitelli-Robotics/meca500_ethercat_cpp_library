#include "Robot.hpp"

int main(int argc, char *argv[]) {
    Robot robot(0,1000);
    robot.activate();
    robot.home();
    robot.deactivate();
}