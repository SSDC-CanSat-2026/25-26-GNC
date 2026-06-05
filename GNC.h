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
    float trgtnodes[3][3]; //collection of target positions on the launch pad
    float deployHeight; //this is the altitude difference where the glider drops 
    int slack; //this is a boolean that determines if the glider has reached a target too early  
    int mode; //integer that is 0 during (coast) and 1 during (pro-nav)
    int pursue; //boolean that determines if a target can be pursued
    int DROPNOW; //boolean that determines when the egg gets dropped
    int activateGNC; //boolean that determines when GNC main loop turns on
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

typedef struct {
    float phi_cmd; 
} AutoPilot; 

//Can remove these, I just included this because VSCode is complaining about undefined structs
//looks like sensors each have their own struct in flight software, so these won't be needed.
typedef struct 
{
    float something;
} ICM42688P_AccelData;
typedef struct 
{
    float something;
} GGA_Data_t;
typedef struct 
{
    float something;
} PSTMPV_Data_t;

typedef struct {
    float x;
    float y; 
    float z;
} BMM350;

//Function Definitions
float calculateTgo(float phi, float R, float Vg);
void findTarget(Nav *nav, GGA_Data_t *gga);
float calculateHE(float pos_G[3][1], float vel_L[3][1]);
uint16_t computeCommand(Nav *nav, AutoPilot *ap);
Nav init_Navigation(ICM42688P_AccelData data, GGA_Data_t gga, int GPS_ready);
float Update_Autopilot(Guidance guid, Nav nav);
void Update_Guidance(Nav *nav, Guidance *guid);
void Update_Navigation(Nav *nav, ICM42688P_AccelData data, GGA_Data_t gga, int GPS_ready);
#endif

