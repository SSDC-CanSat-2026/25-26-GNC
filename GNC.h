#ifndef GNC
#define GNC
#include <stdint.h>
//Define pi macro
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//Define Structs
/*Navigation struct (the navigation states of the glider used in the guidance laws to steer the paraglider)
  Most likely will need to use the global mission struct and pull from relevant sensors ready 
  for data acquisition (e.g. GPS and accel can be used for velocity computations, but accel may only be available at 
  certain times, (this functionality must be programmed in the updateNavigation() function))*/
typedef struct {
    float pos_G[3][1]; //position of the glider in NED coordinates
    float vel_L[3][1]; //this is the local velocity in NED coordinates
    float vel_B[3][1]; //velocity in the body frame (obtained by doing coordinate transform from local NED to body)
    float rpy[3][1]; //attitude of the paraglider to do body to NED (or vice versa) conversion
    float HE; //Heading error of the paraglider in (deg)
    float trgt[3][1]; //the target that the glider is actively pursuing
    int slack; //this is a boolean that determines if the glider has reached a target too early  
    int mode; 
    int pursue;
    uint32_t time;
    uint32_t timeFound; // time a target was found 
    uint32_t timeIntercept; //time a target has been intercepted
} Nav;
/* 
Guidance struct contains the acceleration commands determined by a specified guidance law depending on the mode
in the navigation struct. This is then converted to a roll angle command in the autopilot function.
*/
typedef struct {
    float accel_cmd_L[3][1]; //position of the glider in NED coordinates
    float accel_cmd_B[3][1]; //this is the local velocity in NED coordinates
} Guidance;

//Function Definitions
float calculateTgo(float phi, float R, float Vg);
void findTarget(Nav *nav, float latg, float longg, float altg);
float calculateHE(float pos_G[3][1], float vel_L[3][1]);
uint32_t computeCommand(Nav *nav, uint32_t channel, float roll_cmd);
float Update_Autopilot(Guidance guid, Nav nav);
void Update_Guidance(Nav nav, Guidance *guid);
//void Update_Navigation(Nav *nav, ICM42688P_AccelData data, GGA_Data_t gga, PSTMPV_Data_t PSTMPV)
#endif

