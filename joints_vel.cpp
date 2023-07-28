#include <iostream>
#include <string.h>
#include "jacobian_meca.h" //TEST
#include "joints_vel.h"    //TEST
#include "invertij_1colonna.h"

void get_joints_vel_with_jacobian(double velocity, float *joints, float *joints_vel)
{
    double jacobian_v[36];
    double prima_colonna_inversa[6];
    jacobian_meca(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], jacobian_v);
    invertiJ_1colonna(jacobian_v, prima_colonna_inversa);
    for (int i = 0; i < 6; i++)
    {
        joints_vel[i] = prima_colonna_inversa[i] * velocity;
    }
}

void print_matrix_colmajor( char* desc, int m, int n, double* mat, int ldm ) {
        int i, j;
        printf( "\n %s\n", desc );
 
        for( i = 0; i < m; i++ ) {
                for( j = 0; j < n; j++ ) printf( " %6.2f", mat[i+j*ldm] );
                printf( "\n" );
        }
}