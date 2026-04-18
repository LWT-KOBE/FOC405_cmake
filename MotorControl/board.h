
#ifndef __BOARD_LIB_H
#define __BOARD_LIB_H

#include "MyProject.h"

void control_loop_cb(void);
void IWDG_Init(void);
/****************************************************************************/
extern  Iph_ABC_t  current0;
extern  float  vbus_voltage;
/****************************************************************************/

#endif

