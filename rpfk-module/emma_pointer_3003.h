//
// Created by robert on 20-3-16.
//

/**
 * @file "modules/emma/emma_pointer.h"
 * @author Robert Koster
 * Move around in the cyberzoo
 */

#ifndef EMMA_POINTER_3003_H
#define EMMA_POINTER_3003_H
#include <inttypes.h>

extern uint8_t safeToGoForwards;
extern int32_t incrementForAvoidance;
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);
extern uint8_t increase_nav_heading(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);

// custom function for EMMA
// replace moveWaypointForwards(WP_GOAL,3.0) with moveWaypointToEdge(WP_GOAL,WP__OZ1,WP__OZ2,WP__OZ3,WP__OZ4) to use the new function in the flight plan
extern uint8_t moveWaypointToEdge(uint8_t waypoint, uint8_t waypoint_oz1, uint8_t waypoint_oz2, uint8_t waypoint_oz3, uint8_t waypoint_oz4);


#endif

