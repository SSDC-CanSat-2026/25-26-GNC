#ifndef LINALG_H
#define LINALG_H

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum GNC_Axis
{
    linalg_x_axis = 1,
    linalg_y_axis,
    linalg_z_axis,
} GNC_Axis;

// Function Prototypes
//I believe undefined or variable array size (anything just not known at compile time) for parameters causes a compilation error in C; 
//if anything matrices have to be passed in as pointers
void printArray(float arr[], int len);
float dot(float arr1[], float arr2[], int size);
void transposeMat(float A[][3]);
void matmult(int n, int m, float mat1[n][n], float mat2[n][m], float AB[n][m]);
void computeDCM(float th, char ax, float R[3][3]);
void rpyDCM(float r,float p,float y, float R_py[3][3]);
void geodetic2ecef(float latpt, float longpt, float h,float r[3][1]);
void geodetic2ned(float lat_t,float long_t,float h_t,float lat_p,float long_p,float h_p, float r_tp[3][1]);
#endif