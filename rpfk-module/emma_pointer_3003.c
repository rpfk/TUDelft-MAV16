/*
 * Copyright (C) Robert Koster
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Robert Koster
 * 30-03-2016 Version of emma_pointer
 */

#include "modules/emma/emma_pointer_3003.h"
#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>

uint8_t safeToGoForwards=FALSE;
int tresholdColorCount = 200;
int32_t incrementForAvoidance;

void orange_avoider_init() {
    // Initialise the variables of the colorfilter to accept orange
    color_lum_min=0;
    color_lum_max=131;
    color_cb_min=93;
    color_cb_max=255;
    color_cr_min=134;
    color_cr_max=255;
    // Initialise random values
    srand(time(NULL));
    chooseRandomIncrementAvoidance();
}
void orange_avoider_periodic() {
    // Check the amount of orange. If this is above a threshold
    // you want to turn a certain amount of degrees
    safeToGoForwards = color_count < tresholdColorCount;
    printf("Checking if this funciton is called %d treshold: %d now: %d \n", color_count, tresholdColorCount, safeToGoForwards);
}

/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, int32_t increment)
{
    *heading = *heading + increment;
    // Check if your turn made it go out of bounds...
    INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
    return FALSE;
}
uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters){
    struct EnuCoor_i new_coor;
    struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

    // Calculate the sine and cosine of the heading the drone is keeping
    float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
    float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

    // Now determine where to place the waypoint you want to go to
    new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
    new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
    new_coor.z = pos->z; // Keep the height the same

    // Set the waypoint to the calculated position
    waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

    return FALSE;
}

uint8_t chooseRandomIncrementAvoidance(){

    int r = rand() % 2;
    if(r==0){
        incrementForAvoidance=350;
    }
    else{
        incrementForAvoidance=-350;
    }
    return FALSE;
}

/**
 * moveWaypointToEdge(WP_GOAL, WP_CZ1, WP_CZ2, WP_CZ3, WP_CZ4)
 *
 * Set waypoint to furthest target possible point in cyberzoo from direction in body frame (pitch, yaw)
 */
uint8_t moveWaypointToEdge(uint8_t waypoint, uint8_t waypoint_oz1, uint8_t waypoint_oz2, uint8_t waypoint_oz3, uint8_t waypoint_oz4){
    struct EnuCoor_i new_coor;
    struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
    struct FloatEulers *ang = stateGetNedToBodyEulers_f(); // Get your current attitude

    // Initial target heading in dx, dy
    float Heading[2] = { sinf(ang->psi), cosf(ang->psi) };

    // Get target heading from image coordinates x,y
    // To do: get target heading h from image coordinates x,y
    float x_target = xtarget;
    float y_target = 0;
    target_angle_body(x_target, y_target, Heading);

    // List corner points of the ObstacleZone
    float oz[4][2] = {
            {WaypointX(waypoint_oz1), WaypointY(waypoint_oz1)},
            {WaypointX(waypoint_oz2), WaypointY(waypoint_oz2)},
            {WaypointX(waypoint_oz3), WaypointY(waypoint_oz3)},
            {WaypointX(waypoint_oz4), WaypointY(waypoint_oz4)}
    };

    int i,j;
    float n, n_min = 100;
    for(i = 0; i < 4; i++) {
        j = ( i + 1 ) % 4;

        if (oz[j][0] - oz[i][0] != 0) {

            float A[2];
            A[0] = ( oz[j][1] - oz[i][1] ) / ( oz[j][0] - oz[i][0] );
            A[1] = Heading[1] / Heading[0];

            if (A[0] - A[1] != 0) {
                float B[2];
                B[0] = oz[i][1] - A[0] * oz[i][0];
                B[1] = pos->y - A[1] * pos->x;
                n = ( (B[1] - B[0]) / (A[0] - A[1]) - pos->x ) / Heading[0];
            }

        } else {
            float m[3];
            m[0] = ( oz[i][1] - pos->y ) / Heading[1];
            m[1] = ( oz[j][1] - pos->y ) / Heading[1];
            m[2] = ( oz[i][0] - pos->x ) / Heading[0];

            for(ii = 0; ii < 3; ii++) {
                n = 100;
                if ( m[ii] > 0 & m[ii] < n ) {
                    n = m[ii];
                }
            }
        }

        if ( n > 0 & n < n_min ) {
            n_min = n;
        }
    }

    // Now determine where to place the waypoint you want to go to
    new_coor.x = pos->x + n_min * Heading[0];
    new_coor.y = pos->y + n_min * Heading[1];
    new_coor.z = pos->z; // Keep the height the same

    // Set the waypoint to the calculated position
    waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

    return FALSE;
}
/**
 * target_yaw_body(x_target,y_target)
 *
 * get the target direction in the body reference frame from a target location in the image
 */
void target_angle_body(float x_target, float y_target, float heading[2]) {

    float pitch_t, yaw_t, F_cam, x_t, y_t;

    //// Constants
    // Image size (x_m,y_m) in pixels
    int x_max = 272, y_max = 272;
    // Camera distance to the viewing plane (f_c) and to the center of body reference frame (d_c)
    float f_cam = 1.0, d_cam = 0.0;
    // Camera viewing angle (horizontal a_c, vertical b_c) in degrees
    float a_cam = 90.0, b_cam = 90.0;

    // correction for camera offset
    F_cam = f_cam / (f_cam + d_cam);

    // location of target in image from -1 to 1
    x_t = 2 * (float) x_target / x_max - 1;
    y_t = 2 * (float) y_target / y_max - 1;

    // get pitch and yaw from camera offset correction factor, target location in image and viewing angle (in radians)
    pitch_t = atanf( F_cam * y_t * tan( b_cam * M_PI / 360 ) );
    yaw_t   = atanf( F_cam * x_t * tan( a_cam * M_PI / 360 ) );

    float target_body[3];

    target_direction_body(pitch_t, yaw_t, target_body);
    
    float target[3];
    
    target_direction_cyberzoo(target, target_body);
    
    heading[0] = target[0] / sqrtf( powf(target[0],2) + powf(target[1],2) );
    heading[1] = target[1] / sqrtf( powf(target[0],2) + powf(target[1],2) );

}

void target_direction_body(float pitch_target, float yaw_target, float target[3]) {

    // Direction of target in body frame
    target[0] = cosf(-pitch_target) * cosf(-yaw_target);
    target[1] = -cosf(-pitch_target) * sinf(-yaw_target);
    target[2] = sinf(-pitch_target);

}

void target_direction_cyberzoo(float target_earth[3], float target_body[3]) {
    
    // Get your current attitude
    struct FloatEulers *att = stateGetNedToBodyEulers_f(); 

    float roll  = att->phi;
    float pitch = att->theta;
    float yaw   = att->psi;
    
    // Transformation matrix of body frame to Earth fixed frame
    static float T[3][3];
    T[0][0] = cosf(pitch) * cosf(yaw);
    T[0][1] = sinf(pitch) * cosf(yaw) * sinf(roll) + sinf(yaw) * cosf(roll);
    T[0][2] = -sinf(pitch) * cosf(yaw) * cosf(roll) + sinf(yaw) * sinf(roll);
    T[1][0] = -cosf(pitch) * sinf(yaw);
    T[1][1] = -sinf(pitch) * sinf(yaw) * sinf(roll) + cosf(yaw) * cosf(roll);
    T[1][2] = sinf(pitch) * sinf(yaw) * cosf(roll) + cosf(yaw) * sinf(roll);
    T[2][0] = sinf(pitch);
    T[2][1] = -cosf(pitch) * sinf(roll);
    T[2][2] = cosf(pitch) * cosf(roll);

    // Direction of target in Earth fixed frame
    target_earth[0] = T[0][0] * target_body[0] + T[0][1] * target_body[1] + T[0][2] * target_body[2];
    target_earth[1] = T[1][0] * target_body[0] + T[1][1] * target_body[1] + T[1][2] * target_body[2];
    target_earth[2] = T[2][0] * target_body[0] + T[2][1] * target_body[1] + T[2][2] * target_body[2];
    
}