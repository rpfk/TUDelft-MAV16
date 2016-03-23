#include <stdlib.h>     /* qsort */

void Objects(struct image_t *img)
{

	<!--
	Orange pole detector
	-->

	uint8_t margin = 30;
	uint8_t ymin   = 144 - margin;
	uint8_t ymax   = 144 + margin;
	uint8_t umin   = 92  - margin;
	uint8_t umax   = 92  + margin;
	uint8_t vmin   = 145 - margin;
	uint8_t vmax   = 145 + margin;

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


	if labels_count > 0
	{
		for(int i = 0; i < labels_count; i++)
		{
		   printf(labels[i].id);
		}
		
		BestEscape(labels, img->w);
		
	}

}

void ObjectsInit(void)
{
	cv_add(Objects);
}

uint16_t BestEscape(struct image_label_t labels, uint16_t width )
{

	uint16_t map[20];
	map[0] = 0;
 
	for(int i = 0; i < labels_count; i++)
	{
		// printf(labels[i].x_min);		
		uint16_t x_max = (labels[i].x_sum / labels[i].pixel_cnt - labels[i].x_min) * 2 + labels[i].x_min;
		
		map[i*2 + 1] = labels[i].x_min;
		map[i*2 + 2] = x_max;
		
	}
	
	if(x_max < width)
	{
		map[i*2 + 1] = width;
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

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}
