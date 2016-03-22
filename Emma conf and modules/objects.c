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
		for(i = 0; i < labels_count; i++)
		{
		   printf(labels[i].id);
		}
		
		   BestEscape(labels);
		
	}

}

void ObjectsInit(void)
{
	cv_add(Objects);

}

uint16_t BestEscape(labels)
{
	for(i = 0; i < labels_count; i++)
	{
	   printf(labels[i].x_min);
	}


}
