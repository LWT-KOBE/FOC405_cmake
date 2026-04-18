
#ifndef _ANTI_COGGING_H
#define _ANTI_COGGING_H

#include "MyProject.h"


/****************************************************************************/
#define  COG_num   3600
/****************************************************************************/
typedef struct 
{
	uint32_t index;
	float cogging_map[COG_num];
	bool pre_calibrated;
	bool calib_anticogging;
	float calib_pos_threshold;
	float calib_vel_threshold;
	float cogging_ratio;
	bool anticogging_enabled;
} Anticogging_t;

extern  Anticogging_t  anticogging;
extern  bool anticogging_valid_;
/****************************************************************************/
void anticogging_init(void);
void start_anticogging_calibration(void);
bool anticogging_calibration(float pos_estimate, float vel_estimate);
/****************************************************************************/




#endif

