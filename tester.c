#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "linalg.h"
#include "GNC.h"

//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
const float D2R = M_PI*(1.0/180.0);

int main() {
//USE THE FOLLOWING BELOW AS A REFERENCE FOR CONSTRUCTION OF THE GNC ALGORITHM

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

    //EXAMPLE: MAIN INITIALIZATION - REPLACE NECESSARY ELEMENTS WITH SENSOR STRUCT FIELDS
    //So for example, gps[3] takes in gga struct latitude, longitude, and altitude (or global struct)
    float gps[3] = {38.3756417, -79.6072944, 1038}; //arbitrary (this is just the center of the drop zone)
    float accel[3][1] = {{0},{0},{1.0}}; //acceleration from the accelerometer (g)
    float gyro[3][1] = {{2.0},{1.0},{4.0}};

    Nav nav = init_Navigation(gps,gyro,accel); //initialize the navigation states
    Guidance guid = {0}; //no guidance initially, can set this to zero
    AutoPilot ap = {0}; //same story with the autopilot, can set this to zero 

    printf("Nav struct properties:\n");
    printf("Initial Position (N,E,D): %0.5f (m),%0.5f (m),%0.5f (m) \n",nav.pos_G[0][0],nav.pos_G[1][0],nav.pos_G[2][0]);
    printf("Attitude (r,p,y): %0.2f (rad), %0.2f (rad), %0.2f (rad) \n",nav.rpy[0][0],nav.rpy[1][0],nav.rpy[2][0]);
    printf("---------------------------------------------------\n");

//This is the target directly in the center of the drop zone
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
    
    //EXAMPLE TEMPLATE: MAIN INITIALIZATION
    int totalTime = 10;
    for (int i=1;i<totalTime;i++) { //simualte GPS change in coordinates (replace with sensor readings)
        gps[0] = gps[0]-((rand()%1)/1000.0);
        gps[1] = (gps[1]+(rand()%1)/1000.0);
        gps[2] = (gps[2]-(rand()%2)); 
        
        gyro[0][0] = (rand()%100); //random change gyro and accel readings (angular rate)
        gyro[1][0] = (rand()%100); //pitch (rate)
        gyro[2][0] = (rand()%100); //yaw (rate)

        accel[0][0] = (rand()%16); //random change in accel readings (g's), ax
        accel[1][0] = (rand()%16); //ay
        accel[2][0] = (rand()%16); //az

        Update_Navigation(&nav,gps,gyro,accel); //update the navigation states
        //printf("Position (x,y,z): %0.2f (m), %0.2f (m), %0.2f (m)\n",nav.pos_G[0][0],nav.pos_G[1][0],nav.pos_G[2][0]);
        printf("Attitude (r,p,y): %0.2f (rad), %0.2f (rad), %0.2f (rad) \n",nav.rpy[0][0],nav.rpy[1][0],nav.rpy[2][0]);
        Update_Guidance(&nav,&guid); //with the new navigation states, update guidance commands 
        Update_Autopilot(&guid,&nav,&ap); //determine autopilot commands which convert guidance commands into rotations
        uint16_t cmd = computeCommand(&nav,&ap); //compute the rotations necessary to turn the motors in us
        
        //ADD SERVO WRITE CODE HERE // (the servos should be run at a 50Hz frequency)
        //Ex: SERVO_RawMove(instance,cmd)
        
        //CONDITIONAL LOGIC (this mainly just checks whether the paraglider needs to search for a target)
        if (nav.slack == 1 && nav.DROPNOW == 1) {
            printf("Condition 1 Entered. Deploying Egg....\n");
            //ADD DEPLOYMENT CODE HERE (write to release servo)
        }
        else if (nav.slack == 1 && nav.DROPNOW == 0) {
            printf("Condition 2 Entered. Searching for nearest target...\n");
            findTarget(&nav,gps);
        }
        else if (nav.slack == 0 && nav.DROPNOW == 1) {
            printf("Condition 3. Deploying Egg....\n");
            //ADD DEPLOYMENT CODE HERE (write to release servo)
        }
        else if (nav.slack == 0 && nav.DROPNOW == 0) {
            printf("Condition 4 Entered, Glider in Coast Phase\n");
            findTarget(&nav,gps);
        }
    }
}