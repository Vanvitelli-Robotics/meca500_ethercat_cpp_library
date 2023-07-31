#ifndef JOINTS_VEL_H
#define JOINTS_VEL_H

void get_joints_vel_with_jacobian(double velocity,float* joints,float* joints_vel,float* pose);
void print_matrix_rowmajor( const char* desc, int rows, int cols, const double* mat );
void print_matrix_rowmajor_f( const char* desc, int rows, int cols, const float* mat );
void construct_T_phi(const double* phi,double* T_phi);

//test//TEST
#endif