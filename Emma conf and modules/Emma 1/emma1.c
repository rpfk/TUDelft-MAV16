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

//#ifndef BEBOP_FRONT_CAMERA_H
//#define BEBOP_FRONT_CAMERA_H

#include "modules/emma1/emma1.h"
#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/bebop_front_camera.h"
#include "modules/computer_vision/video_thread.h"

int i = 1;

void emma_init() {
	// initialize variables

}
void emmafunction() {

	// no idea
	//printf("it's working!!!!!");
}
uint8_t emma69(uint8_t waypoint) {
	float wp1_x = -1.0;
	float wp1_y = 1.0;
        float h1 = 0.0;
	float wp2_x = -1.0;
	float wp2_y = -1.0;
        float h2 = 0.0;
	float wp3_x = 1.0;
	float wp3_y = 1.0;
        float h3 = 0.0;
	float wp4_x = 1.0;
	float wp4_y = -1.0;
        float h4 = 0.0;
	float dist_threshold = 0.1;
	double wps[8] = {wp1_x, wp1_y, wp2_x, wp2_y, wp3_x, wp3_y, wp4_x, wp4_y};
        double headings[4] = {h1,h2,h3,h4}; 

        struct EnuCoor_i new_coor;
        struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position
	//struct EnuCoor_f *pos = stateGetPositionEnu_f(); // Get your current position 
	
	printf("Current pos \t");
        printf("posX= %f \t", POS_FLOAT_OF_BFP(pos->x)); //POS_FLOAT_OF_BFP(pos->x)
        printf("posY= %f \t", POS_FLOAT_OF_BFP(pos->y)); //POS_FLOAT_OF_BFP(pos->y)
        printf("\n");
       
	float wpX = waypoint_get_x(waypoint);
	float wpY = waypoint_get_y(waypoint);
        
	printf("Current wp \t");
        printf("wpX: %f \t", wpX);
        printf("wpY: %f \t", wpY);
        printf("\n");
        
	//float dist_curr = POS_FLOAT_OF_BFP((POS_BFP_OF_REAL(wpX) -  pos->x)*(POS_BFP_OF_REAL(wpX) -  pos->x) + (POS_BFP_OF_REAL(wpY) -  pos->y)*(POS_BFP_OF_REAL(wpY) -  pos->y)); // POS_BFP_OF_REAL POS_FLOAT_OF_BFP(pos->x)
	
	float dist_curr = (wpX -  POS_FLOAT_OF_BFP(pos->x))*(wpX -  POS_FLOAT_OF_BFP(pos->x)) + (wpY -  POS_FLOAT_OF_BFP(pos->y))*(wpY -  POS_FLOAT_OF_BFP(pos->y)); // POS_BFP_OF_REAL POS_FLOAT_OF_BFP(pos->x)

	printf("Dist to current wp \t");	
        printf("dist_curr: %f \t", dist_curr);        
        printf("\n");

        //float dist1 = (wpX - wp1_x)*(wpX - wp1_x) + (wpY - wp1_y)*(wpY - wp1_y); // Dist between current wp and navigation wps
	//float dist2 = (wpX - wp2_x)*(wpX - wp2_x) + (wpY - wp2_y)*(wpY - wp2_y);
	//float dist3 = (wpX - wp3_x)*(wpX - wp3_x) + (wpY - wp3_y)*(wpY - wp3_y);
	
	
	//if (dist1 < dist2 && dist1 < dist3){i=1;} 
	//else if (dist2 < dist1 && dist2 < dist3){i=2;}
	//else {i=3;}

	if (dist_curr < dist_threshold*dist_threshold){
		i = i + 1;
		if (i> 4){i=1;}
	}

	// Set the waypoint to the calculated position
        printf("Set waypoint to \t");
	printf("i: %d \t", i);
	printf("wpsX: %f \t",wps[(i-1)*2]);
	printf("wpsY: %f \t",wps[(i-1)*2+1]);
        printf("\n");

	//struct image_t *img = v4l2_image_get(bebop_front_camera.dev, &img);
	//struct a *img = v4l2_image_get(bebop_front_camera.dev, &img);
	//struct image_t *img;
	
	printf("image height:" "%f \t", emsimg->h);
	
	new_coor.x = POS_BFP_OF_REAL(wps[(i-1)*2]);
	new_coor.y = POS_BFP_OF_REAL(wps[(i-1)*2+1]);
	new_coor.z = pos->z;

	// Set the waypoint to the calculated position
        waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
        
        // Set heading to requested
        nav_set_heading_deg(headings[i-1]);

        printf("\n");
	printf("\n");

	return FALSE;

}

