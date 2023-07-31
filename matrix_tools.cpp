#include "matrix_tools.h"
#include <math.h>

const double alpha = 0.4;
const int gradient_descent_iterations = 1000;
double qk[6] = {0,0,0,0,0,0};

void multiply_matrix(double *A, int rows_a, int cols_a, double *B, int rows_b, int cols_b, double *result)
{
    // performs A*B
    if (cols_a != rows_b)
    {
        throw "impossible multiplication";
    }

    for (int i = 0; i < rows_a; i++)
    {
        for (int j = 0; j < cols_b; j++)
        {
            double sum = 0;
            for (int k = 0; k < cols_a; k++)
            {
                sum += A[i * cols_a + k] * B[k * cols_b + j];
            }
            result[i * cols_b + j] = sum;
        }
    }
}

void subtract_matrix(double* A, double* B,int rows,int cols, double* result) {
    for(int i=0;i<rows;i++) {
        for(int j=0;j<cols;j++) {
            result[i*cols+j] = A[i*cols+j] - B[i*cols+j];
        }
    }
}

void multiply_matrix_scalar(double* A,int rows, int cols, double k, double* result) {
    for(int i=0;i<rows;i++) {
        for(int j=0;j<cols;j++) {
            result[i*cols+j] = A[i*cols+j]*k;
        }
    }
}

void concat_vertically(double* A,double* B,int cols,int rows_A,int rows_B, double* result) {
    for(int i=0;i<rows_A;i++) {
        for(int j=0;j<cols;j++) {
            result[i*cols+j] = A[i*cols+j];
        }
    }
    for(int i=0;i<rows_B;i++) {
        for(int j=0;j<cols;j++) {
            result[(i+rows_A)*cols+j] = B[i*cols+j];
        }
    }
}

void transpose(double *M, int rows, int cols, double *Mt)
{
    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            Mt[i * rows + j] = M[j * cols + i];
        }
    }
}

void solve_linear_system_6(double* A, double* c, double* b) {
    //gaussian_elimination_6(A,c,b);
    gradient_descent_6(A,c,b);
}

void gradient_descent_6(double* J, double* c, double* b) {
    double Jt[36];
    double Jt_J[36];
    double Jt_v[6];
    double q_k1[6];
    transpose(J,6,6,Jt);
    multiply_matrix(Jt,6,6,J,6,6,Jt_J);
    multiply_matrix(Jt,6,6,c,6,1,Jt_v);
    for(int i=0;i<gradient_descent_iterations;i++) {
        gradient_descent_6_iteration(qk,Jt_J,Jt_v,q_k1);
        for(int j=0;j<6;j++) {
            qk[j] = q_k1[j];
        }
    }
    for(int j=0;j<6;j++) {
            b[j] = qk[j];
        }
}

void gradient_descent_6_iteration(double* qk,double* Jt_J,double* Jt_v,double* qk_1) {
    double Jt_J_qk[6];
    double gradient[6];
    double multiplied_gradient[6];
    multiply_matrix(Jt_J,6,6,qk,6,1,Jt_J_qk);
    subtract_matrix(Jt_J_qk,Jt_v,6,1,gradient);
    multiply_matrix_scalar(gradient,6,1,alpha,multiplied_gradient);
    subtract_matrix(qk,multiplied_gradient,6,1,qk_1);
}

// function to get matrix content
void gaussian_elimination_6(double *A, double *c, double *b)
{
    /*

    Solves the linear system Ab=c , with A being a 6x6 matrix

    */

    double mat[6][7];
    for (int i = 0; i < 6; i++)
    {
        for(int j=0;j<6;j++) {
            mat[i][j] = A[i*6+j];
        }
        mat[i][6] = c[i];
    }

    int singular_flag = forward_elim(mat);

    /* if matrix is singular */
    if (singular_flag != -1)
    {
        throw "Singular matrix";
    }

    /* get solution to system and print it using
       backward substitution */
    back_sub(mat,b);
}

// function for elementary operation of swapping two rows
void swap_row(double mat[6][7], int i, int j)
{
    // printf("Swapped rows %d and %d\n", i, j);

    for (int k = 0; k <= 6; k++)
    {
        double temp = mat[i][k];
        mat[i][k] = mat[j][k];
        mat[j][k] = temp;
    }
}

// function to reduce matrix to r.e.f.
int forward_elim(double mat[6][7])
{
    for (int k = 0; k < 6; k++)
    {
        // Partial pivoting: find the row with the maximum pivot element
        int i_max = k;
        double v_max = fabs(mat[i_max][k]);

        for (int i = k + 1; i < 6; i++)
        {
            double v = fabs(mat[i][k]);
            if (v > v_max)
            {
                v_max = v;
                i_max = i;
            }
        }

        // Check if the pivot element is too close to zero (near singularity)
        if (fabs(mat[i_max][k]) < 1e-10)
            return k; // Matrix is singular

        // Swap the greatest value row with the current row
        if (i_max != k)
            swap_row(mat, k, i_max);

        for (int i = k + 1; i < 6; i++)
        {
            double f = mat[i][k] / mat[k][k];

            for (int j = k + 1; j <= 6; j++)
                mat[i][j] -= mat[k][j] * f; //test

            mat[i][k] = 0;
        }
    }

    return -1;
}

// function to calculate the values of the unknowns
void back_sub(double mat[6][7],double* b)
{

    /* Start calculating from last equation up to the
       first */
    for (int i = 5; i >= 0; i--)
    {
        /* start with the RHS of the equation */
        b[i] = mat[i][6];

        /* Initialize j to i+1 since matrix is upper
           triangular*/
        for (int j = i + 1; j < 6; j++)
        {
            /* subtract all the lhs values
             * except the coefficient of the variable
             * whose value is being calculated */
            b[i] -= mat[i][j] * b[j];
        }

        /* divide the RHS by the coefficient of the
           unknown being calculated */
        b[i] = b[i] / mat[i][i];
    }
}
