//
// Created by robert on 20-3-16.
//

/**
 * @file "modules/emma/emma_pointer.h"
 * @author Robert Koster
 * Move around in the cyberzoo
 */

#ifndef EMMA_POINTER_H
#define EMMA_POINTER_H
#include <inttypes.h>

/**
 * Constants
 */

// Image size (x_m,y_m) in pixels
#define x_max 1280
#define y_max 720
// Target position in image (x_t,y_t)
#define x_target 1180
#define y_target 360
// Camera distance to the viewing plane (f_c) and to the center of body reference frame (d_c)
#define f_cam 1.0
#define d_cam 0.5
// Camera viewing angle (horizontal a_c, vertical b_c) in degrees
#define a_cam 170.0
#define b_cam 2.0

//extern uint8_t safeToGoForwards;
//extern int32_t incrementForAvoidance;
//extern void orange_avoider_init(void);
//extern void orange_avoider_periodic(void);
extern uint8_t moveWaypointToEdge(uint8_t waypoint, uint8_t waypoint_oz1, uint8_t waypoint_oz2, uint8_t waypoint_oz3, uint8_t waypoint_oz4);
//extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
//extern uint8_t chooseRandomIncrementAvoidance(void);

#endif

