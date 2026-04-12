import numpy as np
import math as m
from scipy.integrate import ode
import matplotlib.pyplot as plt
#--------------------- GNC Subroutines---------#
# The objective of this python code is to provide 
# the Electronics/Firmware team with the necessary GNC
# subroutine logic that will be used on the Paraglider
# for autonomous navigation. Description of each subroutine
# are present and are intended for instructive purposes and 
# will not be actually implemented in this exact manner in the 
# flight code.
d2r = m.pi/180
r2d = 1/d2r
g = 9.79281 #local gravity

def proNav(r_p,vNED,v_p,los_i,roll_i):
#Calculates the proportional navigation acceleration command to be
#used during Phase 1 of flight. 
#Inputs: 
#1) r_p (pursuer(glider) position with respect to target) GPS
#2) vNED (pursuer velocity in the NED frame) -> GPS,accel
#3) v_p (pursuer velocity in body frame) -> GPS,accel

#Proportional navigation requires the following three
#quantites to be determined to compute an acceleration command. These are as follows:
#1) Proportional constant (N) - (user defined) affects how fast the glider enters a collision trajectory with the target  
#2) Pursuer velcity (v_p) - velocity of the glider (measured from GPS/accelerometer)
#3) Line of sight rate (dlos) - the line of sight rate must be proportional to the commanded acceleration for target intercept
    
    N = 2 #(usually around 2-5)    
    Kp = 5 #Proportional gain for roll controller 
    
    dlos = ((vNED[1]*r_p[0])-(r_p[1]*v_NED[0]))/m.sqrt(r_p[0]**2+r_p[1]**2) #rate of change of the line of sight
    #Note: it may be acceptable to instead compute the line of sight over two time steps and compute the derivative 
    #using los_2-los_1/(dt). The above is from kinematic derivations. If a quicker approximation is desired. 
    #then the following approximation can also be done (los_i is line of sight at previous time step i) 
    los_j = m.atan2(r_p[1],r_p[0]) #line of sight at next time step j 
    dlos = (los_j-los_i)/dt  

    #Compute the proportional navigation command in the body frame: 
    a_p = N*v_p*dlos
    
    #To relate the acceleration command to the required roll angle we use this equation. 
    #This comes from equations of equillibrium of an aircraft at some roll angle in steady flight
    #Constant altitude, coordinate turn is assummed 
    roll_cmd = m.atan2(a_p/g)*r2d
    #Compute the control command to the motors this is correlated to a line deflection.
    #This deflection (delta) can then be mapped to the motors using a degree/deflection coefficinet (c).
    #Of course we also must convert this to a difference in position in microseconds, define a us/degree coefficient (d)
    c = 1.0
    d = 1.0 #these values are bs for now
    delta = Kp*(roll_cmd-roll_i)
    mtr_cmd = c*d*delta
    
    return mtr_cmd

def Rmatrix(ang,axis):
#Description: Computes a 3x3 rotation matrix about a specific axis
#inputs required are an angle and the corrosponding axis of rotation 
#to select the appropriate matrix. It forms 3 rows of the matrix and combines them into a 
#3x3 array

#The following convention is used in this function: 
# (1) - x axis
# (2) - y axis
# (3) - z axis
    if axis == 1: #compute x rotation matrix (R_x)
        r1 = [1,0,0] 
        r2 = [0,m.cos(ang),m.sin(ang)]
        r3 = [0,-m.sin(ang),m.cos(ang)]
        R = np.array([r1,r2,r3])
    elif axis == 2: #compute y rotation matrix (R_y)
        r1 = [m.cos(ang),0,-m.sin(ang)]
        r2 = [0,1,0]
        r3 = [m.sin(ang),0,m.cos(ang)]
        R = np.array([r1,r2,r3]) 
    elif axis == 3: #compute z rotation matrix (R_z)
        r1 = [m.cos(ang),m.sin(ang),0]
        r2 = [-m.sin(ang),m.cos(ang),0]
        r3 = [0,0,1]
        R = np.array([r1,r2,r3])
    return R

def geodetic2ned(geot,geop):
#Description: Finds the relative position vector between the target geot = (lat_t,long_t,h_s)
#and the paraglider geop = (lat_p,long_p,h_p) in a North, East, Down coordinates. 
#Uses the WGS-84 ellipsoid model of earth

#Variables: 
#1) lat_t,long_t: latitude and longitude of the target (fixed)
#2) long_p,long_p: latitude and longitude of the paraglider
#3) h_t,h_p: mean height above ellipsoid (if just looking at planar motion can make heights the same...
#...and we can deal with altitude in a separate routine). 
    
    #Convert deg. to rad.
    lat_t = geot[0]*d2r
    long_t = geot[1]*d2r
    h_t = geot[2]
    lat_p = geop[0]*d2r
    long_p = geop[1]*d2r
    h_p = geop[2]
    
    #Now we have to compute the geodetic coordinates to GPS
    r_t = geodetic2ecef(lat_t,long_t,h_t) #target ECEF vector
    r_p = geodetic2ecef(lat_p,long_p,h_p) #paraglider ECEF vector
    
    #Compute the difference between the two vectors (target to position vector)
    dr = r_t-r_p
    
    #Compute the ECEF->NED rotation matrix (sorry but it's time for array math) 
    r1 = [-m.sin(lat_t)*m.cos(long_t),-m.sin(lat_t)*m.sin(long_t),m.cos(lat_t)]
    r2 = [-m.sin(long_t),m.cos(long_t),0.0]
    r3 = [-m.cos(lat_t)*m.cos(long_t),-m.cos(lat_t)*m.sin(long_t),-m.sin(lat_t)]
    R = np.array([r1,r2,r3])
    
    #Finally, compute the NED vector
    r = R@dr
    
    return r 
    
    
def geodetic2ecef(lat,long,h):
#Description: Finds the position vector of a specific point in on Earth using Earth-Centered-Earth-Fixed
#(ECEF) coordinates. This uses equations that can be found on ESA's website, but it mainly comes from the 
# local geometry of the WGS-84 ellisoid. 

#Variables: 
#1) lat,long: latitude and longitude of the point
#2) h: mean height above ellipsoid
    a = 6378137.0 #equatorial radius of the Erf
    f = 1/298.257223563
    e2 = f*(2-f)
    
    N = a/m.sqrt(1-e2*m.sin(lat**2))#local curvature of the ellipsoid
    x = (N+h)*m.cos(lat)*m.cos(long) #Coordinates of point in ECEF
    y = (N+h)*m.cos(lat)*m.sin(long)
    z = (N*(1-e2)+h)*m.sin(lat)
    
    r = np.array([x,y,z]).T #ECEF vector
    return r


def main(): #main is used as a tester function
    #---------------------- PHASE 0) INITIALIZE -----------------------
    #BIAS variables (obtain from taking an average of a couple samples when ground station command calibration) 
    h_b = 100 #altitude bias (m) (have to calibrate to zero, but store to calculate altitude difference)
    gyro_bx,gyro_by,gyro_bz = 0.5,0.75,0.1 #gyro bias (deg/s) 
    gb = np.array([gyro_bx,gyro_by,gyro_bz])
    accel_bx,accel_by,accel_bz = 0.1,0.05,0.2 #accelerometer bias (m/s^2)
    ab = np.array([accel_bx,accel_by,accel_bz])
    
    #Note: via CANSAT rules accelerometer must be in deg/s^2 in the Ground Station, but GNC should use m/s^2
    #Convert Sensor Frame coordinates to Body frame coordinate
    tz = 180*d2r #sensor is rotated 180 about x (not actually sure, will check how sensor is mounted)
    gb = Rmatrix(tz,1)
    
    trgt = np.array([38.3760167, -79.6078722,500]).T #target coordinates (at launch pad)
    pgldr = np.array([38.3758388, -79.6076027,500]).T #right now this is just a position on the target pad
    
    rtp = geodetic2ned(trgt,pgldr)
    dist = m.sqrt(rtp[0]**2+rtp[1]**2)
    
    #Print results: 
    print(f'Rotation Matrix (IMU to Body): {gb}')
    print(f'Position Vector to Paraglider: {rtp}')
    print(f'Distance (North-East Plane): {dist} (m)')
    
    #----------------------PHASE 1) COAST PHASE -----------------------
    #This phase will be executed when the glider is outside of the container and will initiate 
    #based on the descent rate of the glider (once this reaches nearly a constant).
    #once the descent rate reaches a constant, the porportional navigation phase is executed
    #so that the glider steers to an intercept trajectory.
    
    
if __name__ == "__main__":
    main()