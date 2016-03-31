/*
 * Copyright (C) Matthias Baert
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/emma1/emma1.c"
 * @author Matthias Baert
 * This is a first module for Emma
 */

#include "modules/emma1/emma1.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
//#include "modules/computer_vision/blob/blob_finder.h"
//#include "modules/computer_vision/lib/vision/image.h"
//#include "modules/computer_vision/cv.h"
//#include "modules/computer_vision/bebop_front_camera.h"
//#include "modules/computer_vision/video_thread.h"

//#include "firmwares/rotorcraft/autopilot.h"

int k = 1;
int emmacount = 0;
int prevheading = 0;

void emma_init() {
	// Initialise the variables of the colorfilter to accept orange
	//color_lum_min=114;
	//color_lum_max=174;
	//color_cb_min=62;
	//color_cb_max=122;
	//color_cr_min=115;
	//color_cr_max=175;
        //nav_set_heading_deg(0);

}
void emmafunction() {
        // periodic
        //printf("wouter is: %d", color_count);
	
}
 
uint8_t emma69(uint8_t waypoint) {
	
	float dist_forward = 0.8;
	float circle_radius = 2.0;
	float wpXNew = 0.0;
	float wpYNew = 0.0;
        //uint8_t xtarget = 0;
        
        uint8_t newheading = 0;

        struct EnuCoor_i new_coor;
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
	//struct EnuCoor_f *pos = stateGetPositionEnu_f(); // Get your current position 
	
	printf("Current pos \t");
        printf("posX= %f \t", POS_FLOAT_OF_BFP(pos->x)); //POS_FLOAT_OF_BFP(pos->x)
        printf("posY= %f \t", POS_FLOAT_OF_BFP(pos->y)); //POS_FLOAT_OF_BFP(pos->y)
        printf("\n");
       
	//float wpX = waypoint_get_x(waypoint);
	//float wpY = waypoint_get_y(waypoint);

	newheading = prevheading + (xtarget-136)*80/136;

	if (POS_FLOAT_OF_BFP(pos->x)*POS_FLOAT_OF_BFP(pos->x) + POS_FLOAT_OF_BFP(pos->y)*POS_FLOAT_OF_BFP(pos->y) > circle_radius*circle_radius){

		newheading = 90 - atan2(-POS_FLOAT_OF_BFP(pos->y),-POS_FLOAT_OF_BFP(pos->x))*180/3.1415;
	}
	
	if (newheading < 0) {newheading = newheading + 360;}
	if (newheading > 360) {newheading = newheading - 360;}

	nav_set_heading_deg(newheading);
	prevheading = newheading;

	//move waypoint forward
	wpXNew = POS_FLOAT_OF_BFP(pos->x) + cos(3.1415*0.5 - (float)newheading/180*3.1415)*dist_forward;
	wpYNew = POS_FLOAT_OF_BFP(pos->y) + sin(3.1415*0.5 - (float)newheading/180*3.1415)*dist_forward;	

	// Set the waypoint to the calculated position
        printf("Set waypoint to \t");
	printf("wpsX: %f \t",wpXNew);
	printf("wpsY: %f \t",wpYNew);
	printf("newheading: %d \t", newheading);
        printf("xtarget: %d",xtarget);
        printf("\n");
	

	new_coor.x = POS_BFP_OF_REAL(wpXNew);
	new_coor.y = POS_BFP_OF_REAL(wpYNew);
	new_coor.z = pos->z;

	// Set the waypoint to the calculated position
        waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);


        printf("\n");
	printf("\n");

	return FALSE;

}



