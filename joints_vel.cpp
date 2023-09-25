#include <iostream>
#include <string.h>
#include "matrix_tools.h" //TEST
#include "joints_vel.h"    //TEST
#include "jacobian_meca.h"    //TEST
#include "math.h"
#include "CsvLoggerFeedback.hpp"
#include "UnitQuaternion.h"

#define DEG_TO_RAD(x) (M_PI * x / 180.0)

const double initial_pose[6] = {0.0, -0.240, 0.190,M_PI_2, 0.0, 0.0};
const double joints_saturation_speed[6] = {DEG_TO_RAD(150.0),DEG_TO_RAD(150.0),DEG_TO_RAD(180.0),DEG_TO_RAD(300.0),
DEG_TO_RAD(300.0),DEG_TO_RAD(500.0)};
const double pose_tolerance[6] = {0.2,0.02,0.02,5,5,5};
double T = 4e-3;
const double gamma_coeff = (0.1)/T;
CsvLoggerFeedback csvLoggerFeedback("feedback.csv");
vanvitelli::UnitQuaternion<double> qd; //test
vanvitelli::UnitQuaternion<double> q_current;
vanvitelli::UnitQuaternion<double> deltaQ;


void get_joints_vel_with_jacobian(double velocity_x, float *joints, float *joints_vel,float* pose)
{
    qd.euler_xyz(initial_pose+3);
    double jacobian[36];
    double buffer[36];
    double velocity[6];
    double joints_vel_d[6];
    double j_jv[6];
    double theta[3];
    // print_matrix_rowmajor("initial pose",6,1,initial_pose);
    // print_matrix_rowmajor_f("joints",6,1,joints);
    // print_matrix_rowmajor_f("pose",6,1,pose);
    double pos_err[3] = {0,0,0};
    double phi_err[3];
    double error_v[6];

    for(int i=0;i<6;i++) {
        csvLoggerFeedback << pose[i];
    }

    for(int i=1;i<3;i++) {
        pos_err[i] = -pose[i]+initial_pose[i];
    }

    q_current.euler_xyz(pose+3);
    deltaQ = qd.prod(vanvitelli::inv(q_current));
    phi_err[0] = deltaQ.v().x();
    phi_err[1] = deltaQ.v().y();
    phi_err[2] = deltaQ.v().z();

    // std::cout << "qd\n";
    // qd.print();
    // std::cout << "\nq curr\n";
    // q_current.print();
    // std::cout << "\ndelta q\n";
    // deltaQ.print();
    for(int i=0;i<3;i++) {
        theta[i] = pose[i+3];
    }

    // print_matrix_rowmajor("pos error",3,1,pos_err);
    // print_matrix_rowmajor("theta",3,1,theta);
    // print_matrix_rowmajor("phi error",3,1,phi_err);
    // print_matrix_rowmajor("T(phi)",3,3,T_phi);
    // print_matrix_rowmajor("T(phi)*phi",3,1,T_phi_m_phi_err);
    concat_vertically(pos_err,phi_err,1,3,3,error_v);
    // print_matrix_rowmajor("error vector",6,1,error_v);
    velocity[0] = velocity_x;
    for(int i=1;i<6;i++) {
        velocity[i] = gamma_coeff*error_v[i];
    }

    // print_matrix_rowmajor("desired cartesian vel",6,1,velocity);
    for(int i=0;i<6;i++) {
        csvLoggerFeedback << velocity[i];
    }
    jacobian_meca(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], buffer);
    transpose(buffer,6,6,jacobian);
    // print_matrix_rowmajor("jacobiano",6,6,jacobian);
    gaussian_elimination_6(jacobian,velocity,joints_vel_d);
    /*
        Going back to robot convention for velocities
    */
    joints_vel_d[1] *= -1;
    joints_vel_d[2] *= -1;
    joints_vel_d[4] *= -1;
    for(int i=0;i<6;i++) {
        joints_vel[i] = joints_vel_d[i];
    }
    for(int i=0;i<6;i++) {
        if(fabs(joints_vel[i])>joints_saturation_speed[i]) {
            double k = joints_saturation_speed[i]/joints_vel[i];
            for(int j=0;j<6;j++) {
                joints_vel[j] *= k;
            }
        }
    }
    for(int i=0;i<6;i++) {
        if(fabs(pose[i]-initial_pose[i])>pose_tolerance[i]) {
            for(int i=0;i<6;i++) {
                joints_vel[i] = 0.0;
            }
        }
    }
    for(int i=0;i<6;i++) {
        csvLoggerFeedback << joints_vel[i];
    }
    for(int i=0;i<6;i++) {
        csvLoggerFeedback << joints[i];
    }
    csvLoggerFeedback.end_row();
    // print_matrix_rowmajor("joints_v",6,1,joints_vel_d);
    // print_matrix_rowmajor_f("joints_v saturata",6,1,joints_vel);
    multiply_matrix(jacobian,6,6,joints_vel_d,6,1,j_jv);
    // print_matrix_rowmajor("jacobian*joints_v",6,1,j_jv);
    // std::cout << "\n"; 
}

void construct_T_phi(const double* phi, double* T_phi) {
    double sinTheta = sin(phi[1]);
    double cosTheta = cos(phi[1]);
    double sinPhi = sin(phi[2]);
    double cosPhi = cos(phi[2]);

    T_phi[0] = sinTheta;
    T_phi[1] = 0.0;
    T_phi[2] = 1.0;
    T_phi[3] = -cosTheta*sinPhi;
    T_phi[4] = cosPhi;
    T_phi[5] = 0.0;
    T_phi[6] = cosTheta * cosPhi;
    T_phi[7] = sinPhi;
    T_phi[8] = 0.0;

    // double cosPhi1 = cos(phi[0]);
    // double sinPhi1 = sin(phi[0]);
    // double cosPhi2 = cos(phi[1]);
    // double sinPhi2 = sin(phi[1]);
    // double cosPhi3 = cos(phi[2]);
    // double sinPhi3 = sin(phi[2]);

    // T_phi[0] = cosPhi2 * cosPhi3;
    // T_phi[1] = sinPhi3;
    // T_phi[2] = sinPhi2 * cosPhi3;
    // T_phi[3] = cosPhi2 * sinPhi3 * sinPhi1 - cosPhi1 * sinPhi2;
    // T_phi[4] = cosPhi3 * cosPhi1;
    // T_phi[5] = cosPhi1 * cosPhi2 + sinPhi1 * sinPhi2 * sinPhi3;
    // T_phi[6] = sinPhi1 * sinPhi3 * cosPhi2 + cosPhi1 * sinPhi2;
    // T_phi[7] = -cosPhi1 * cosPhi3;
    // T_phi[8] = cosPhi1 * sinPhi2 - sinPhi1 * cosPhi2 * sinPhi3;
}

void print_matrix_rowmajor( const char* desc, int rows, int cols, const double* mat ) {
        int i, j;
        printf( "\n %s\n", desc );
 
        for( j = 0; j < rows; j++ ) {
                for( i = 0; i < cols; i++ ) printf( " %6.5f", mat[j*cols+i] );
                printf( "\n" );
        }
}

void print_matrix_rowmajor_f( const char* desc, int rows, int cols, const float* mat ) {
        int i, j;
        printf( "\n %s\n", desc );
 
        for( j = 0; j < rows; j++ ) {
                for( i = 0; i < cols; i++ ) printf( " %6.5f", mat[j*cols+i] );
                printf( "\n" );
        }
}