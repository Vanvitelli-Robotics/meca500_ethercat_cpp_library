#ifndef JOINTS_VEL_H
#define JOINTS_VEL_H

void get_joints_vel_with_jacobian(double velocity,float* joints,float* joints_vel);
void print_matrix_colmajor( char* desc, int m, int n, double* mat, int ldm );

//test//TEST
#endif