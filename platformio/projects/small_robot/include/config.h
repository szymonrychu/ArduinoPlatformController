#ifndef CONFIG_H
#define CONFIG_H

#define PI               3.1415926535897932384626433832795
#define HALF_PI          1.5707963267948966192313216916398
#define TWO_PI           6.283185307179586476925286766559
#define DEG_TO_RAD       0.017453292519943295769236907684886
#define RAD_TO_DEG       57.295779513082320876798154814105
#define DEG10_IN_RADIANS 0.174532925199433

#define ROBOT_LENGTH 0.350
#define ROBOT_WIDTH  0.248

#define MIN_TURN_COEF 0.0

// +X - to right
// +Y - to up

#define M1_X -(ROBOT_WIDTH/2) 
#define M1_Y -(ROBOT_LENGTH/2)
#define M1_IN_PLACE_TURN -1.0
#define M1_ENCA 30
#define M1_ENCB 1
#define M1_HA   24
#define M1_HB   25
#define M1_SERV 8

#define M2_X (ROBOT_WIDTH/2) 
#define M2_Y -(ROBOT_LENGTH/2)
#define M2_IN_PLACE_TURN -1.0
#define M2_ENCA 3
#define M2_ENCB 2
#define M2_HA   29
#define M2_HB   28
#define M2_SERV 9

#define M3_X (ROBOT_WIDTH/2) 
#define M3_Y (ROBOT_LENGTH/2)
#define M3_IN_PLACE_TURN -1.0
#define M3_ENCA 5
#define M3_ENCB 4
#define M3_HA   22
#define M3_HB   23
#define M3_SERV 10

#define M4_X -(ROBOT_WIDTH/2) 
#define M4_Y (ROBOT_LENGTH/2)
#define M4_IN_PLACE_TURN -1.0
#define M4_ENCA 31
#define M4_ENCB 7
#define M4_HA   37
#define M4_HB   36
#define M4_SERV 33

#define SERV_PAN  15
#define SERV_TILT 14

#define BATT_MEAS_AIN A17
#define USER_BUTTON_1 38
#define USER_BUTTON_2 35
#define USER_LED_1    34
#define USER_LED_2    32

#endif