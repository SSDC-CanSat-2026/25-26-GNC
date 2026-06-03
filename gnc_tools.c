#include "linalg.h"

#define r2d (180.0f / M_PI)
#define g 9.79281f
#define DEL2US 58.97365f
#define delta0 1500

//I am not sure about the parameters here: they seem to be vecctors so I used arrays to accomodate them
float proNav(float r_p[3], float vNED[3], float v_p, float los_i, float roll_i, float dt)
{
// #Calculates the proportional navigation acceleration command to be
// #used during Phase 1 of flight. 
// #Inputs: 
// #1) r_p (pursuer(glider) NED position vector of pursuer with respect to target 
// #2) vNED (pursuer velocity in the NED frame) -> GPS,accel
// #3) v_p (pursuer velocity in body frame) -> GPS,accel

// #Proportional navigation requires the following three
// #quantites to be determined to compute an acceleration command. These are as follows:
// #1) Proportional constant (N) - (user defined) affects how fast the glider enters a collision trajectory with the target  
// #2) Pursuer velcity (v_p) - velocity of the glider (measured from GPS/accelerometer)
// #3) Line of sight rate (dlos) - the line of sight rate must be proportional to the commanded acceleration for target intercept

    const float N = 2.0f; //usually around 2-5
    const float Kp = 2.0f; //Proportional gain for roll controller

    //pro-nav requires vector from pursuer to target, not the other way around
    float rp[3];
    rp[0] = -r_p[0];
    rp[1] = -r_p[1];
    rp[2] = -r_p[2];

    
    float los_j = atan2f(rp[1], rp[0]);

    float dlos = (los_j - los_i) / dt;

    float a_p = N * v_p * dlos;

    float roll_cmd = atan2f(a_p, g) * r2d;

    float delta = (Kp * (roll_cmd - roll_i));

    // Servo command: The original recommends this as an int but for integrity purposes I'll return it as a float
    float mtr_cmd = delta0 + delta * DEL2US;

    return mtr_cmd;
}

void Rmatrix(float ang, int axis, float R[3][3])
{
    switch(axis)
    {
        //Extremely minor detail: I kept the ordering for the axis parameter just in case (x_axis=1)
        case linalg_x_axis:
            R[0][0] = 1.0f;      R[0][1] = 0.0f;       R[0][2] = 0.0f;
            R[1][0] = 0.0f;      R[1][1] = cosf(ang); R[1][2] = sinf(ang);
            R[2][0] = 0.0f;      R[2][1] = -sinf(ang);R[2][2] = cosf(ang);
            break;

        case linalg_y_axis:
            R[0][0] = cosf(ang); R[0][1] = 0.0f;       R[0][2] = -sinf(ang);
            R[1][0] = 0.0f;      R[1][1] = 1.0f;       R[1][2] = 0.0f;
            R[2][0] = sinf(ang); R[2][1] = 0.0f;       R[2][2] = cosf(ang);
            break;

        case linalg_z_axis:
            R[0][0] = cosf(ang);  R[0][1] = sinf(ang);  R[0][2] = 0.0f;
            R[1][0] = -sinf(ang); R[1][1] = cosf(ang);  R[1][2] = 0.0f;
            R[2][0] = 0.0f;       R[2][1] = 0.0f;       R[2][2] = 1.0f;
            break;

        default:
            break;
    }
}