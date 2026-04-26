#include <stdio.h>
#include <math.h>
#include "linalg.h"

int main() {
    //Just some print testing to understand the syntax
    float gyrob[3] = {1,2,3};
    float accelb[3] = {1,2,3};
    int len = sizeof(gyrob)/sizeof(gyrob[0]);
    //Dot product function test
    float dotga = dot(gyrob,accelb,len);
    
    //Testing 3x3 matrix multiplication 
    //Arbitrary matrix
    float I[3][3] = {{1,0,0},
                     {0,1,0},
                     {0,0,1}};
    //Arbitrary Matrix
    float R[3][3] = {{1,2,3},
                     {3,4,5},
                     {6,7,8}};
    //Matrix Multiplication 
    //Start by pre-allocating memory for resultant matrix
    //May be better just to preallocate matrices then modify in void functions 
    //as oppossed to using pointers, everything is a fixed 3x3 size anyway.
    float Res[3][3] = {{0,0,0},
                       {0,0,0},
                       {0,0,0}};

    //Specify the number of rows (n) and number of columns(m) of the resultant matrix
    float Rc[3][1] = {{1},{2},{3}};
    
    float cRes[3][1] = {{0},{0},{0}};

    matmult(3,1,I,Rc,cRes);
    rpyDCM(30.0,30.0,30.0,Res);

    //Testing geodetic2NED
    float r_tp[3][1] = {{0},{0},{0}};
    
    
    float trgt[3][1] = {{38.3760167}, {-79.6078722},{200}}; //lat, long target coordinates (at launch pad)
    float pgldr[3][1] = {{38.3758388}, {-79.6076027},{500}}; //lat, long of paraglider, right now this is just a position on the target pad
    
    //Compute the NED vector and the North East distance
    geodetic2ned(trgt[0][0],trgt[1][0],trgt[2][0],pgldr[0][0],pgldr[1][0],pgldr[2][0],r_tp);
    float rnorm = sqrt((r_tp[0][0]*r_tp[0][0])+(r_tp[1][0]*r_tp[1][0]));
    
    
    printf("NE Distance (m): %0.4f\n",rnorm);
    printf("PRINTING NED VECTOR: \n");
    for (int i=0;i<3;i++){
        printArray(r_tp[i],1); 
        printf("\n");  
    }
    printf("===========\n"); 
    return 0;
}