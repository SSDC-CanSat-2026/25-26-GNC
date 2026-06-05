#include <stdio.h>
#include <math.h>
#include "linalg.h"
#include "GNC.h"

//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
const float D2R = M_PI*(1.0/180.0);

int main() {
    //TESTING (CAN IGNORE UP TO "MAIN INITIALIZATION") -----------------------
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
    
    Nav nav = {0};
    Guidance guid = {0};
    AutoPilot ap = {0};
    nav.trgt[0][0] = 38.3760167f;
    nav.trgt[1][0] = -79.6078722f;
    nav.trgt[2][0] = 1038.0f;

    nav.rpy[0][0] = 10.0*D2R;
    nav.rpy[1][0] = 0.0;
    nav.rpy[2][0] = 0.0;

    float rtp[3][1] = {{-41.63},{50.50},{0.00}};
    float vel_L[3][1] = {{3.32},{1.21},{-3.54}};

    nav.vel_L[0][0] = 3.32;
    nav.vel_L[1][0] = 1.21;
    nav.vel_L[2][0] = -3.54;


    nav.HE = calculateHE(rtp,vel_L);
    nav.HE *= D2R;
    printf("----------------------\n");
    printf("Heading Error: %0.3f\n",nav.HE);

    printf("NE Distance (m): %0.4f\n",rnorm);
    printf("PRINTING NED VECTOR: \n");
    for (int i=0;i<3;i++){
        printArray(r_tp[i],1); 
        printf("\n");  
    }
    printf("===========\n"); 

    float latg   = 38.3756417f;
    float longg = -79.6072944f;
    float mhoe  = 1038.0f;
    ap.phi_cmd = 18.0;
    nav.slack = 0;
    findTarget(&nav,latg,longg,mhoe);
    uint32_t cmd = computeCommand(nav,ap);

    printf("Command computed: %d us\n",cmd);

    float HE = 1.44;
    float Vg =  0.93;
    float Rng = 83.39;

    float tgo = calculateTgo(HE,Rng,Vg);
    printf("Time to go estimate: %0.2f\n",tgo);

    nav.mode = 1;

    //Update Autopilot
    nav.pos_G[0][0] = r_tp[0][0];
    nav.pos_G[1][0] = r_tp[1][0];
    nav.pos_G[2][0] = r_tp[2][0];
    // END OF TESTING -----------------------

    //Replace with whatever initialization is being done 
    ICM42688P_AccelData data = {0};
    GGA_Data_t gga = {0};
    PSTMPV_Data_t PSTMPV = {0};
    int GPS_ready = 1; // check the GPS readines

    //MAIN INITIALIZATION - run once to initialize Nav struct 
        //1) initialize gyro,accel,mag offsets at launch pad
        //2) read from sensors and run init_Navigation()
            //EXAMPLE: Nav nav = init_Navigation(data, gga, PSTMPV, GPS_ready);
            //Guidance guid = {0}; //can make guidance blank until we enter control loop
        //3) need something that will start the main loop when glider outside container, can we use descent acceleration?
            /*
            The accelerometer, though noisy, should measure nearly +1g when the glider reaches terminal velocity. 
            So we can use a conditional that checks ICM42688P accel_z (or whichever axis is pointing vertical) and 
            see when it's close to zero it may be best to do this over an average to avoid fast spikes in accelerometer 
            values affecting when the main loop is started. We will have to coordinate transform this into the NED frame
            and I can work on getting this done ASAP unless we already have something we can use for this.
            */

    //MAIN LOOP - Process in the main loop will always perform the following actions
        //1) Read data from sensors (assuming this is already being done)
        //2) Update_Navigation(structs from all sensors)
        //3) Update_Guidance(Nav nav, Guidance *guid)
        //4) Update_AutoPilot()
        //5) uint16_t cmd = computeCommand(Nav nav, AutoPilot ap)
        //6) SERVO_MoveTo(cmd)

    //CONDITIONAL CHECKS (kinda looks like a logic table)
        //1) nav.slack = 1 AND nav.DROPNOW = 1 (deploy egg if glider has intercepted target and is at req. drop altitude)
        //2) nav.slack = 1 AND nav.DROPNOW = 0 (run targetScan if nav.pursue = 0 and run loop as normal)
        //3) nav.slack = 0 AND nav.DROPNOW = 1 (deploy egg at target altitude)
        //4) nav.slack = 0 AND nav.DROPNOW = 0 (run targetScan if nav.pursue = 0 and run loop as normal)

    //Update_Navigation()
    Update_Guidance(&nav,&guid);
    Update_Autopilot(&guid,nav);
    printf("Autopilot activated, phi_cmd: %0.2f\n",ap.phi_cmd);
    return 0;

}