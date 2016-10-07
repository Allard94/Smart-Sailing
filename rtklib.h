/*
 * rtklib.h
 *
 *  Created on: Oct 7, 2016
 *      Author: root
 */

#ifndef RTKLIB_H_
#define RTKLIB_H_

#include "rtklib/rtklib.h"

extern nav_t nav;
extern obs_t obs;
extern rtk_t rtk;
extern sta_t sta;

void nav_init(nav_t *nav);
void obs_init(obs_t * obs);
void obs_assign(obs_t *obs);
void rtk_opt_init(rtk_t * rtk);

#endif /* RTKLIB_H_ */
