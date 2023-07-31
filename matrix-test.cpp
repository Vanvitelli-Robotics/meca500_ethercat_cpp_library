#include "joints_vel.h"
#include "math.h"
#include "time.h"
#include <iostream>

#define DEG_TO_RAD(x) (M_PI * x / 180.0)

int main(int argc, char *argv[])
{
    float joints[6] = {DEG_TO_RAD(-94), DEG_TO_RAD(33), DEG_TO_RAD(20), DEG_TO_RAD(3), DEG_TO_RAD(-48), DEG_TO_RAD(90)};
    float joints_vel[6];
    float pose[6] = {0, -0.25, 0.18, M_PI_2-0.1, 0.1, 0.15};
    float speed = 0.4;
    clock_t begin = clock();
    get_joints_vel_with_jacobian(speed, joints, joints_vel, pose);
    clock_t end = clock();
    double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    std::cout << time_spent << " secondi" << std::endl;
}
