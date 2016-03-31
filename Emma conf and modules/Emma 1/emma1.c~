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
/* 
void image_labeling(struct image_t *input, struct image_t *output, struct image_filter_t *filters, uint8_t filters_cnt,
                    struct image_label_t *labels, uint16_t *labels_count)
{
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint16_t *output_buf = (uint16_t *)output->buf;

  // Initialize labels
  uint16_t labels_size = *labels_count;
  uint16_t labels_cnt = 0;
  uint16_t i, x, y;

  // Initialize first line with empty groups
  uint16_t *p = output_buf;
  for (i = 0; i < output->w; i++) {
    *p++ = 0xffff;
  }

  // Do steps of 2 for YUV image
  // Skip first line as we need previous groups for connectivity
  for (y = 1; y < input->h; y++) {
    for (x = 0; x < input->w / 2; x++) {
      uint16_t lid = 0;
      uint8_t p_y = (input_buf[y * input->w * 2 + x * 4 + 1] + input_buf[y * input->w * 2 + x * 4 + 3]) / 2;
      uint8_t p_u = input_buf[y * input->w * 2 + x * 4];
      uint8_t p_v = input_buf[y * input->w * 2 + x * 4 + 2];

      // Go trough the filters
      uint8_t f = 0;
      for (; f < filters_cnt; f++) {
        if (p_y > filters[f].y_min && p_y < filters[f].y_max &&
            p_u > filters[f].u_min && p_u < filters[f].u_max &&
            p_v > filters[f].v_min && p_v < filters[f].v_max) {
          break;
        }
      }

      // Check if this pixel belongs to a filter else goto next
      if (f >= filters_cnt) {
        output_buf[y * output->w + x] = 0xFFFF;
        continue;
      }

      // Check pixel above (if the same filter then take same group)
      lid = output_buf[(y - 1) * output->w + x];
      if (y > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Check pixel top right (check for merging)
      lid = output_buf[(y - 1) * output->w + x + 1];
      if (y > 0 && x < output->w - 1 && lid < labels_size && labels[lid].filter == f) {

        // Merging labels if needed
        uint16_t lid_tl = output_buf[(y - 1) * output->w + x - 1]; // Top left
        uint16_t lid_l = output_buf[y * output->w + x - 1]; // Left
        uint16_t m = labels[lid].id, n = labels[lid].id;
        if (x > 0 && lid_tl < labels_size && labels[lid_tl].filter == f) {
          // Merge with top left
          m = labels[lid].id;
          n = labels[lid_tl].id;
        } else if (x > 0 && lid_l < labels_size && labels[lid_l].filter == f) {
          // Merge with left
          m = labels[lid].id;
          n = labels[lid_l].id;
        }

        // Change the id of the highest id label
        if (m != n) {
          if (m > n) {
            m = n;
            n = labels[lid].id;
          }

          for (i = 0; i < labels_cnt; i++) {
            if (labels[i].id == n) {
              labels[i].id = m;
            }
          }
        }

        // Update the label
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Take top left
      lid = output_buf[(y - 1) * output->w + x - 1];
      if (y > 0 && x > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Take left
      lid = output_buf[y * output->w + x - 1];
      if (x > 0 && lid < labels_size && labels[lid].filter == f) {
        output_buf[y * output->w + x] = lid;
        labels[lid].pixel_cnt++;
        labels[lid].x_sum += x;
        labels[lid].y_sum += y;
        continue;
      }

      // Check if there is enough space
      if (labels_cnt >= labels_size - 1) {
        break;
      }

      // Create new group
      lid = labels_cnt;
      output_buf[y * output->w + x] = lid;
      labels[lid].id = lid;
      labels[lid].filter = f;
      labels[lid].pixel_cnt = 1;
      labels[lid].x_min = x;
      labels[lid].y_min = y;
      labels[lid].x_sum = x;
      labels[lid].y_sum = y;
      labels_cnt++;
    }
  }

  if (labels_cnt >= labels_size - 1) {
    printf("Break did not work: we have %d labels\n", labels_cnt);
  }

  // Merge connected labels
  for (i = 0; i < labels_cnt; i++) {
    if (labels[i].id != i) {
      uint16_t new_id = labels[i].id;
      labels[new_id].pixel_cnt += labels[i].pixel_cnt;
      labels[new_id].x_sum += labels[i].x_sum;
      labels[new_id].y_sum += labels[i].y_sum;

      //printf("%d == %d,  ",new_id, i);

      if (labels[i].x_min < labels[new_id].x_min) { labels[new_id].x_min = labels[i].x_min; }
      if (labels[i].y_min < labels[new_id].y_min) { labels[new_id].y_min = labels[i].y_min; }
    }
  }

  *labels_count = labels_cnt;

  // Replace ID's
  for (y = 0; y < input->h; y++) {
    for (x = 0; x < input->w / 2; x++) {
      uint16_t lid = output_buf[y * output->w + x];
      if (lid < labels_cnt) {
        output_buf[y * output->w + x] = labels[lid].id;
      }
    }
  }
}
int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}
uint16_t BestEscape(struct image_label_t *labels, uint16_t width, uint16_t labels_count)
{

	uint16_t map[20];
	map[0] = 0;
        uint16_t x_max = 0;

	for(int i = 0; i < labels_count; i++)
	{
		// printf(labels[i].x_min);		
		x_max = (labels[i].x_sum / labels[i].pixel_cnt - labels[i].x_min) * 2 + labels[i].x_min;
		
		map[i*2 + 1] = labels[i].x_min;
		map[i*2 + 2] = x_max;
		
	}
	
	if(x_max < width)
	{
		map[labels_count*2 + 1] = width;
	}

	// uint16_t sortedmap[20];

	qsort (map, 20, sizeof(uint16_t), compare);

	int BiggestOpenDist = 0;
	int BiggestOpenIndex = 0;
	int CurrentOpenDist = 0;

	for(int j = 0; j < 20-1; j = j+2)
	{
		CurrentOpenDist = map[j+1] - map[j];
		
		if(CurrentOpenDist > BiggestOpenDist)
		{
			BiggestOpenDist = CurrentOpenDist;
			BiggestOpenIndex = j;
		}

	}
	
	
	return BiggestOpenDist / 2 + map[BiggestOpenIndex];	

}
*/
/*
uint8_t ScanObjects(struct image_t *img)
{
	//Orange pole detector

	uint8_t margin = 30;
	uint8_t ymin   = 70;//144 - margin;
	uint8_t ymax   = 134;//144 + margin;
	uint8_t umin   = 70;//92  - margin;
	uint8_t umax   = 124;//92  + margin;
	uint8_t vmin   = 127;//145 - margin;
	uint8_t vmax   = 255;//145 + margin;
        uint8_t xtarget = 0;

	struct image_filter_t filter[2];
	filter[0].y_min = ymin;
  	filter[0].y_max = ymax;
  	filter[0].u_min = umin;
  	filter[0].u_max = umax;
  	filter[0].v_min = vmin;
  	filter[0].v_max = vmax;

	uint16_t labels_count = 512;
  	struct image_label_t labels[512];

	struct image_t dst;
	image_create(&dst, img->w, img->h, IMAGE_GRADIENT);

	image_labeling(img, &dst, filter, 1, labels, &labels_count);


	if (labels_count > 0)
	{
		for(int j = 0; j < labels_count; j++)
		{
		   if (labels[j].pixel_cnt >= 100) {
		       printf("labels ID %d \t", labels[j].id);
		       printf("labels cnt %d \n", labels[j].pixel_cnt);
		   }
		}
		
		xtarget = BestEscape(labels, img->w,labels_count);
		printf("xtarget: %d \n",xtarget);
	}
        else {printf("lol");}
        
        return xtarget;
}
*/
uint8_t emma69(uint8_t waypoint) {
	float wp1_x = -0.0;
	float wp1_y = 0.0;
        float h1 = 0.0;
	float wp2_x = -0.0;
	float wp2_y = 0.0;
        float h2 = 0.0;
	float wp3_x = 0.0;
	float wp3_y = 0.0;
        float h3 = 0.0;
	float wp4_x = -0.0;
	float wp4_y = 0.0;
        float h4 = 0.0;
	float dist_threshold = 0.1;
	double wps[8] = {wp1_x, wp1_y, wp2_x, wp2_y, wp3_x, wp3_y, wp4_x, wp4_y};
        double headings[4] = {h1,h2,h3,h4}; 
        //uint8_t xtarget = 0;
        uint8_t prevheading = 0;
        uint8_t newheading = 0;

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
		k = k + 1;
		if (k> 4){k=1;}
	}

	// Set the waypoint to the calculated position
        printf("Set waypoint to \t");
	printf("k: %d \t", k);
	printf("wpsX: %f \t",wps[(k-1)*2]);
	printf("wpsY: %f \t",wps[(k-1)*2+1]);
        printf("\n");

	//struct image_t *img = v4l2_image_get(bebop_front_camera.dev, &img);
	//struct a *img = v4l2_image_get(bebop_front_camera.dev, &img);
	//struct image_t *img;
	
	//printf("image height: %d \t", emsimg->h);
        //printf("image width: %d \t", emsimg->w);

        //emmacount = image_yuv422_colorfilt(emsimg,emsimg,color_lum_min,color_lum_max,color_cb_min,color_cb_max,color_cr_min,color_cr_max);
        //printf("emmacount is: %d \n", emmacount);

        //xtarget = ScanObjects(emsimg);

        
	if (xtarget <= 125) {
            // turn left
            newheading = prevheading - 45;
	    if (newheading < 0) {newheading = newheading + 360;}
	    if (newheading > 360) {newheading = newheading - 360;} 
	    printf("newheading: %d \n", newheading);
            nav_set_heading_deg(newheading);
	    prevheading = newheading;
        }
        else if (125 < xtarget && xtarget < 145) {
            // continue straight        
        }
        else if (xtarget >= 145) {
	    // turn right
            newheading = prevheading + 45;
	    if (newheading < 0) {newheading = newheading + 360;}
	    if (newheading > 360) {newheading = newheading - 360;}
            printf("newheading: %d \n", newheading); 
            nav_set_heading_deg(newheading);
	    prevheading = newheading;
	}       
	

	new_coor.x = POS_BFP_OF_REAL(wps[(k-1)*2]);
	new_coor.y = POS_BFP_OF_REAL(wps[(k-1)*2+1]);
	new_coor.z = pos->z;

	// Set the waypoint to the calculated position
        waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
        
        // Set heading to requested
        //nav_set_heading_deg(headings[k-1]);

        printf("\n");
	printf("\n");

	return FALSE;

}



