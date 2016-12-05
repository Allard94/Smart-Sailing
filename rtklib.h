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
extern rnxctr_t rnx;
extern rtk_t rtk;
extern sta_t sta;

void nav_init(nav_t *nav);
void obs_init(obs_t * obs);
void obs_assign(obs_t *obs);
void rtk_opt_init(rtk_t * rtk);
void rtk_sol_assign(rtk_t * rtk);
void nav_iono_assign(nav_t * nav);
void nav_eph_assign(nav_t * nav);
void obs_test_assign(obs_t * obs, int datanr);
void nav_eph_test_assign(nav_t * nav, obs_t * obs, int datanr);
void rtk_sol_test_assign(rtk_t * rtk, int datanr);
void nav_iono_test_assign(nav_t * nav, int datanr);

#endif /* RTKLIB_H_ */
