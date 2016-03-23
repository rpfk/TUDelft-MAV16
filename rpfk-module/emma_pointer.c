//
// Created by robert on 20-3-16.
//

#include "modules/emma/emma_pointer.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

/**
 * moveWaypointInDirection(waypoint, pitch, yaw)
 *
 * Set waypoint to furthest target possible point in cyberzoo from direction in body frame (pitch, yaw)
 */
uint8_t moveWaypointToEdge(uint8_t waypoint, uint8_t waypoint_oz1, uint8_t waypoint_oz2, uint8_t waypoint_oz3, uint8_t waypoint_oz4){
    struct EnuCoor_i new_coor;
    struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
    struct FloatEulers *ang = stateGetNedToBodyEulers_f(); // Get your current attitude
    
    // Initial direction
    struct FloatEulers h = ang;
    
    // To do: get target heading h from image coordinates x,y
    
    // Target heading in dx, dy
    float Heading[2] = { sin(h->psi), cos(h->psi) };
    
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
                n = ( (B[1] - B[0]) / (A[0] - A[1]) - pos->x ) / heading[0];
            }
                        
        } else {
            float m[3];
            m[0] = ( Y(1) - C_b_cg(2) ) / C_b_h(2);
            m[1] = ( Y(2) - C_b_cg(2) ) / C_b_h(2);
            m[2] = ( X(1) - C_b_cg(1) ) / C_b_h(1);
            
            for(ii = 0; ii < 3; ii++) {
                n = 100;
                if ( m[ii] > 0 & m[ii] < n ) {
                    n = m[ii]
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
void target_angle_body(float x_target, float y_target) {

    float pitch_t, yaw_t, F_cam, x_t, y_t;

    // correction for camera offset
    F_cam = f_cam / (f_cam + d_cam);

    // location of target in image from -1 to 1
    x_t = 2 * (float) x_target / x_max - 1;
    y_t = 2 * (float) y_target / y_max - 1;

    // get pitch and yaw from camera offset correction factor, target location in image and viewing angle (in radians)
    pitch_t = atan( F_cam * y_t * tan( b_cam * M_PI / 360 ) );
    yaw_t   = atan( F_cam * x_t * tan( a_cam * M_PI / 360 ) );
}

void target_direction_body(float pitch_target, float yaw_target) {

    // Direction of target in body frame
    static float PB[3][1];
    PB[0][0] = cos(-pitch_target) * cos(-yaw_target);
    PB[0][1] = -cos(-pitch_target) * sin(-yaw_target);
    PB[0][2] = sin(-pitch_target);
}

void target_direction_cyberzoo(float yaw, float pitch, float roll) {

    // Transformation matrix of body frame to Earth fixed frame
    static float T[3][3];
    T[0][0] = cos(pitch) * cos(yaw);
    T[0][1] = sin(pitch) * cos(yaw) * sin(roll) + sin(yaw) * cos(roll);
    T[0][2] = -sin(pitch) * cos(yaw) * cos(roll) + sin(yaw) * sin(roll);
    T[1][0] = -cos(pitch) * sin(yaw);
    T[1][1] = -sin(pitch) * sin(yaw) * sin(roll) + cos(yaw) * cos(roll);
    T[1][2] = sin(pitch) * sin(yaw) * cos(roll) + cos(yaw) * sin(roll);
    T[2][0] = sin(pitch);
    T[2][1] = -cos(pitch) * sin(roll);
    T[2][2] = cos(pitch) * cos(roll);

    // The following should also give the body attitude rotation matrix:
    // T = FloatRMat;

    // Direction of target in Earth fixed frame
    static float PE[3][1];
    PE[0][0] = T[0][0] * PB[0][0] + T[0][1] * PB[1][0] + T[0][2] * PB[2][0];
    PE[0][1] = T[1][0] * PB[0][0] + T[1][1] * PB[1][0] + T[1][2] * PB[2][0];
    PE[0][2] = T[2][0] * PB[0][0] + T[2][1] * PB[1][0] + T[2][2] * PB[2][0];
}
