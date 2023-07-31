#ifndef MATRIX_TOOLS_H
#define MATRIX_TOOLS_H

extern const double alpha;
extern const int gradient_descent_iterations;

void multiply_matrix(double* A, int rows_a, int cols_a,double* B, int rows_b,int cols_b,double* result);

void subtract_matrix(double* A, double* B,int rows,int cols, double* result);

void multiply_matrix_scalar(double* A,int rows, int cols, double k, double* result);

void transpose(double* M,int rows,int cols,double* Mt);

int forward_elim(double mat[6][7]);
 
// function to calculate the values of the unknowns
void back_sub(double mat[6][7],double* b);
 
void solve_linear_system_6(double* A, double* c, double* b);

void gradient_descent_6(double* A, double* c, double* b);

void gradient_descent_6_iteration(double* qk,double* Jt_J,double* Jt_v,double* qk_1);

// function to get matrix content
void gaussian_elimination_6(double* A, double* c, double* b);

void concat_vertically(double* A,double* B,int cols,int rows_A,int rows_B, double* result);

#endif
