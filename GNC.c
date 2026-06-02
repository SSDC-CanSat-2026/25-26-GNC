#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "linalg.h"
#include "GNC.h"
//#include "ICM42688PSPI.h" (needed for calculating navigation states)

//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
const float DEG2RAD = M_PI*(1.0/180.0);
//GLOBAL CONSTANTS - these are useful constant variables that the GNC algorithm will need
const float eps = 30.0;


#define trgthae 38.376f
#define sens_yaw 0.0f //sensor mounting yaw
#define sens_pitch 0.0f //sensor mounting pitch
#define sens_roll 0.0f
#define deploy (trgthae+2.0f) //ellipsoidal height to drop egg at altitude requirement (change to mean sea level if desired)

float trgtnodes[5][3] = {{38.37606944444445,-79.60810277777777,deploy}, //5 points on the drop zone (2 up, 1 center, 2 down)
            {38.376127777777775,-79.60805277777777,deploy},
            {38.376016666666665, -79.60787222222221,deploy},
            {38.375905555555555, -79.607675,deploy},
            {38.375952777777776, -79.60763055555555,deploy}};


float calculateTgo(float phi, float R, float Vg) {
    /*----Description----
        calculateTgo() - Computes the time to go (t_go) using
        an analytical expression derived by Dhananjay and Ghose.
        Credits: 
        Inputs: 
        %1) Heading Error (phi)
        %2) Range (R) 
        %3) Navigation Gain (N=3 for analytical solution)
        %4) Glider Velocity (Vg)
    */
    float N = 3.0;
    phi *= DEG2RAD;
    phi = fabsf(phi); //prevent negative sqrt
    float denom = powf(fabsf(sin(phi)),(1.0f/(N-1.0f)));
    float K = R/denom;
    //Expression in the paper for N=3 is used: 
    float sqrtphi = powf(phi,0.5f);
    float tw14 = powf(12.0f,0.25f);
    float num = sqrtphi+tw14;
    denom = sqrtphi-tw14;
    //Calculate time to go using analytical equation
    float t_go = (K/(2.0f*Vg))*(tw14)*((0.5f*log(fabsf(num/denom)))+atan(sqrtphi/tw14));
    
    return t_go;
}

void findTarget(Nav *nav, float latg, float longg, float altg){
    /*---- Description -----
    findTarget() - selects a target point within the target drop zone
    to navigate towards based on the following critieria
    /*Notes: 
        1) Because most linear algebra functions in the linalg.c library use void functions,
        arrays are is pre-allocated internally within this function. 
        2) trgt is maintained by the navigation function 
    */
    float trgtprev = nav->trgt[0][0]; //keeping track of current trgt
    float rtp[3][1] = {{0},{0},{0}}; //target to glider NED vector preallocated.
    geodetic2ned(trgtnodes[0][0],trgtnodes[0][1],trgtnodes[0][2],latg,longg,altg,rtp); //get target to glider NED
    float arr1[2] = {rtp[0][0],rtp[1][0]}; //just want NE distance 
    float dist = dot(arr1,arr1,2); //compute north east distance to center target position
    float distprev = dist;
    
    //We iterate on each target, and check which one has the closest altitude time to intercept time
    if (nav->slack == 0) { //no slack time, searching for target during coast
        for (int i=1;i<5;i++) {
            //Check the distance from each target
            geodetic2ned(trgtnodes[i][0],trgtnodes[i][1],trgtnodes[i][2],latg,longg,altg,rtp); //get target to glider NED
            float arr1[2] = {rtp[0][0],rtp[1][0]};
            dist = dot(arr1,arr1,2);
            if (dist<distprev && dist > eps) { //if a closer target is found that satisfies the curvature limit 
                //UPDATE - NEED TO HAVE STRUCT POINTER PASSED TO FUNCTION
                nav->trgt[0][0] = trgtnodes[i][0];
                nav->trgt[1][0] = trgtnodes[i][1];
                nav->trgt[2][0] = trgtnodes[i][2]; 
                printf("FOUND TARGET\n");
            }
        }
    }   
    else { //target intercepted with time remaining until altitude, now we wait for target
        //Check the time to go of each target and look for the one with the smallest POSITIVE time difference
        float arr1[2] = {rtp[0][0],rtp[1][0]};
        
        float vel_L[3][1] = {{0.0},{0.0},{0.0}}; //assign nav arrays to avoid passing more pointers
        vel_L[0][0] = nav->vel_L[0][0];
        vel_L[1][0] = nav->vel_L[1][0];
        vel_L[2][0] = nav->vel_L[2][0];

        float pos_G[3][1] = {{0.0},{0.0},{0.0}};
        pos_G[0][0] = nav->pos_G[0][0];
        pos_G[1][0] = nav->pos_G[1][0];
        pos_G[2][0] = nav->pos_G[2][0];

        nav->HE = calculateHE(pos_G,vel_L);
        float talt = nav->pos_G[2][0]/nav->vel_L[2][0];
        float Vgarr[2] = {nav->vel_L[0][0],nav->vel_L[1][0]};
        float Vg = dot(Vgarr,Vgarr,2);
        dist = sqrt(dot(arr1,arr1,2));
        float tgo = calculateTgo(nav->HE,dist,Vg);
        float tgo_prev = tgo;
        float tdiff = fabsf(tgo-talt);
        float tdiffprev = tdiff;
        for (int i=1;i<5;i++) {
            //Check the time to go from each target
            geodetic2ned(trgtnodes[i][0],trgtnodes[i][1],trgtnodes[i][2],latg,longg,altg,rtp); //get target to glider NED
            float arr1[2] = {rtp[0][0],rtp[1][0]}; //getting NE distance
            nav->HE = calculateHE(rtp,nav->vel_L);

            float Vgarr[2] = {nav->vel_L[0][0],nav->vel_L[1][0]}; //getting velocity in North-East plane 
            Vg = dot(Vgarr,Vgarr,2);
            dist = dot(arr1,arr1,2);
            tgo = calculateTgo(nav->HE,dist,Vg); //use calculate tgo function
            tdiff = tgo-talt; //how close is target intercept time to reaching the altitude?
            
            if (tdiff<tdiffprev && tdiff > 0.0) { 
                //Found a target that has a closer intercept time but is still positive in time difference
                //this ensures that the 
                //UPDATE - NEED TO HAVE STRUCT POINTER PASSED TO FUNCTION
                nav->trgt[0][0] = trgtnodes[i][0];
                nav->trgt[1][0] = trgtnodes[i][1];
                nav->trgt[2][0] = trgtnodes[i][2];
                printf("Target found!\n");
            }
            else if (fabsf(tdiff)<fabsf(tdiffprev)){ // otherwise fing the target with a smaller time regardless of sign
                nav->trgt[0][0] = trgtnodes[i][0];
                nav->trgt[1][0] = trgtnodes[i][1];
                nav->trgt[2][0] = trgtnodes[i][2];   
                printf("FOUND TARGET\n");
            }
            //otherwise, don't assign a new target
            tdiffprev = tdiff;
        }
        //Lastly, check if the trgt changed, if it did, we found a target to pursue. 
        if (trgtprev != nav->trgt[0][0]) {
            printf("FOUND TARGET\n");
            nav->pursue = 1;
            //nav->timeFound = HAL_GetTick();
        }  
        else {
            printf("Did not find a new target\n");
        }
    }
}
    
float calculateHE(float pos_G[3][1], float vel_L[3][1]) {
    printf("TEST PRINT\n");
    /*Description: calculateHE() returns the heading error between the line of sight
    of the target with respect to the glider and the North-East velocity vector of the glider.
    Notes: 
    1) It is assummed that r_tp and vel_L are global navigation states. Both are 
    3x1 arrays. 
    */
    float gamma = atan2f(vel_L[1][0],vel_L[0][0]); //compute the flight path angle of the glider
    float los = atan2f(-pos_G[1][0],-pos_G[0][0]); //compute the line of sight angle of the glider
    float HE = (gamma-los)/DEG2RAD;
    return HE;
}

uint32_t computeCommand(Nav *nav, uint32_t channel, float roll_cmd) {
    /*
    Writes a PWM signal to the motor based on given roll command and current roll
    The current method just uses a proportional controller to bank the glider in 
    the direction of desired bank angle.
    */
    float Kp = 2.0; //proportional gain term
    float DEL2US = 58.97365; //this directly converts a deflection command to microseconds
    float roll = nav->rpy[0][0]*1.0/DEG2RAD;
    roll_cmd *= 1.0/DEG2RAD;
    float err = roll_cmd-roll;
    
    printf("Error: %0.3f\n",err);
    float cmddeg2us = DEL2US*(Kp*err);

    uint32_t cmd = 1500+(uint32_t)roundf(cmddeg2us); //add servo command to servo home at 1500 us
    if (cmd > 2500) {
        cmd = 2500; //clip to max rotation 
    }
    if (cmd < 500) {
        cmd = 500; //clip to min rotation
    }
    return cmd;
}

float Update_Autopilot(Guidance guid, Nav nav){
    float phi_max = 10.0; //Maximum bank angle 
    float phi_cmd = 0.0; //Commanded bank angle 
    if (nav.mode == 0) { //Coasting phase - glider moves in a straight line
        printf("Coasting Phase\n");
        phi_cmd = 0.0;
    }
    else {
        phi_cmd = atan2f(guid.accel_cmd_B[1][0], -guid.accel_cmd_B[2][0]+0.001f); //coordinated turn bank equation
        phi_cmd *= 1/DEG2RAD;
        if (fabsf(phi_cmd) > phi_max) {
            phi_cmd = (phi_cmd/phi_cmd)*phi_max; // clip to max bank and  keep the sign
        }
        
    }
    return phi_cmd;
}

void Update_Guidance(Nav nav, Guidance *guid) { //computes Guidance commands depending on mode
    printf("px: %0.2f\n",nav.pos_G[0][0]);
    float R_BL[3][3] = {{0,0,0},  //form rotation matrix from local NED frame to body frame
                        {0,0,0},
                        {0,0,0}};
    float R_LB[3][3] = {{0,0,0},  //transposed matrix (so we don't change the original one)
                        {0,0,0},
                        {0,0,0}};
    rpyDCM(nav.rpy[0][0],nav.rpy[1][0],nav.rpy[2][0],R_BL);
    rpyDCM(nav.rpy[0][0],nav.rpy[1][0],nav.rpy[2][0],R_LB);
    transposeMat(R_LB);
    if (nav.mode == 0) {
        printf("Coasting Phase\n");
        //Don't do anything, just coasting (using gravity in to avoid accidentally entering a conditional where we divide by 0.0)
        guid->accel_cmd_L[0][0] = 0.0;
        guid->accel_cmd_L[0][1] = 0.0;
        guid->accel_cmd_L[0][2] = -9.80556;
        matmult(3,1,R_LB,guid->accel_cmd_L,guid->accel_cmd_B);
    }
    else { //nav->mode == 1 (Pro-Nav)
        //---------- PROPORTIONAL NAVIGATION ----------
        //Parameters: 
        float N = 3.0; //proportional gain (2-5 is typical)
        matmult(3,1,R_LB,nav.vel_L,nav.vel_B);
        //Relative positions and velocities
        float RtgN = -nav.pos_G[0][0]; //target is origin so just negation of glider pos.
        float RtgE = -nav.pos_G[1][0];
        float R[2] = {RtgN,RtgE};


        float VtgN = -nav.vel_L[0][0];
        float VtgE = -nav.vel_L[1][0];
        float RTG = sqrtf(dot(R,R,2)); //Get planar North-East distance from target to glider
        //Line of sight / line of sight rate
        float los = atan2f(RtgE,RtgN);
        float los_dot = (RtgN*VtgE-RtgE*VtgN)/(powf(RTG,2.0));

        //Control Law (Pure Proportional Navigation)
        //Since using pure pro-nav, we do not use the closing velocity
        //instead, we use the velocity along the x axis of the body frame.
        guid->accel_cmd_B[1][0] = N*nav.vel_B[1][0]*los_dot;
        guid->accel_cmd_B[2][0] = -9.79774; //gravity at CANSAT location
        float ay = N*(nav.vel_B[0][0])*los_dot;
        float az = -9.79774;
        printf("los_dot: %0.5f\n",powf(RTG,2.0));
        printf("Accel CMD, az: %0.5f\n",az);
    }
}


/*
void Update_Navigation(Nav *nav, ICM42688P_AccelData data, GGA_Data_t gga, PSTMPV_Data_t PSTMPV) {
    //------------------- ATTITUDE (roll,pitch,yaw in degrees) -------------------
    //NOTE: CONDITIONALS MUST BE IMPLEMENTED TO CHECK GPS STATUS

    //Integrate gyro and combine with accel (no mag yet, this is bare minimum)
    float w1 = 0.9; //complementary filter weights 
    float w2 = 0.1;
    //Conversion from sensor frame to body frame due to mounting misalignment
    float R_SB[3][3] = {{0.0,0.0,0.0},
                        {0.0,0.0,0.0},
                        {0.0,0.0,0.0}}; //convert sensor frame to body frame (to account for sensor orientation)
    rpyDCM(sens_roll,sens_pitch,sens_yaw,R_SB); //compute the rotation matrix from sensor frame to body frame

    //ICM42688P_read_data() - I'd assume this would already be called in main loop we just pass the data struct?
    //GYRO ANGLES - integrate previous gyro angles over time step times the current gyro rate
    uint32_t currtime = HAL_GetTick();
    uint32_t delta_t = currtime-nav->time; //get time difference
    float gyro_rpy[3][1] = {{0.0},{0.0},{0.0}};
    gyro_rpy[0][0] = (data.gyro_old_r+data.gyro_r*delta_t)*DEG2RAD; //gyro roll, pitch, yaw angles
    gyro_rpy[1][0] = (data.gyro_old_p+data.gyro_p*delta_t)*DEG2RAD;
    gyro_rpy[2][0] = (data.gyro_old_y+data.gyro_y*delta_t)*DEG2RAD;
    //ACCELEROMETER TRIGONOMETRY - Using trigonometry to determine roll and pitch with respect to gravity
    //Note we cannot get yaw from an accelerometer, there is no reference acceleration we can base measurements off of.
    float accel_xyz[3][1] = {{0.0},{0.0},{0.0}};
    accel_xyz[0][0] = data.accel_x;
    accel_xyz[1][0] = data.accel_y;
    accel_xyz[2][0] = data.accel_z;

    matmult(3,1,R_SB,gyro_rpy,gyro_rpy);
    matmult(3,1,R_SB,accel_xyz,accel_xyz);

    float ayz[2] = {data.accel_y,data.accel_z}; 
    float ayz_mag = sqrtf(dot(ayz,ayz,2));
    float accel_rang = atan2f(-data.accel_x,ayz_mag); //accelerometer roll and pitch angles
    float accel_pang = atan2f(data.accel_y,data.accel_z);
    
    //Yaw Angles - If the magnetometer is available can add ecompass() algorithm with accel+mag later
    //If we can get GPS velocity in North, East directions, we can calculate GPS heading and add to gyro.
    float GPS_yang = atan2f(PSTMPV.vel_N,PSTMPV.vel_E);

    //Final Attitude estimation - combine using a complementary filter
    
    nav->rpy[0][0] = (w1*gyro_rpy[0][0])+(w2*accel_rang);
    nav->rpy[1][0] = (w1*gyro_rpy[1][0])+(w2*accel_pang);
    nav->rpy[2][0] = (w1*gyro_rpy[2][0])+(w2*GPS_yang);
    
    //------------------- VELOCITY (North, East, Down in m/s) -------------------
    //VELOCITY 
    w1 = 0.96; //more reliant on GPS properties in this phase
    w2 = 0.04;
    //Note, GPS may not be ready, add conditional that integrates accelerometer completely if this is the case
    float vel_GPS[3][1] = {{PSTMPV.velN},{PSTMPV.velE},{PSTMPV.velV}};
    float vel_accelx = nav->vel_L[0][0]+data.accel_x*delta_t;
    float vel_accely = nav->vel_L[1][0]+data.accel_y*delta_t;
    float vel_accelz = nav->vel_L[2][0]+data.accel_z*delta_t;

    nav->vel_L[0][0] = (w1*vel_GPS[0][0])+(w2*vel_accelx);
    nav->vel_L[1][0] = (w1*vel_GPS[1][0])+(w2*vel_accely);
    nav->vel_L[2][0] = (w1*vel_GPS[2][0])+(w2*vel_accelz);

    //------------------- POSITION (North, East, Down in m) -------------------
    float r_tp[3][1] = {{0.0},{0.0},{0.0}}; //current NED position vector from target to the paraglider
    float pos_GPS[3][1] = {{0.0},{0.0},{0.0}};
    geodetic2ned(nav->trgt[0][0],nav->trgt[1][0],nav->trgt[2][0],gga.latitude,gga.longitude,gga.altitude, pos_GPS);
    //integrate the navigation velocity to get new position estimate
    //Combine with the pure GPS lat,long estimate using a complementary filter
    float pos_vel_x = nav->pos_G[0][0]+nav->vel_L[0][0]*delta_t;
    float pos_vel_y = nav->pos_G[1][0]+nav->vel_L[1][0]*delta_t;
    float pos_vel_z = nav->pos_G[2][0]+nav->vel_L[2][0]*delta_t; 
    nav->pos_G[0][0] = (w1*pos_GPS[0][0])+(w2*pos_vel_x);
    nav->pos_G[1][0] = (w1*pos_GPS[1][0])+(w2*pos_vel_y);
    nav->pos_G[2][0] = (w1*pos_GPS[2][0])+(w2*pos_vel_z);

    nav->time = currtime; //update navigation time to current time 
}
*/

