/*
 * sbp.h
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#ifndef SBP_H_
#define SBP_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/sbp.h>

#include <libswiftnav/time.h>
#include <libswiftnav/track.h>
#ifdef __cplusplus
}
#endif

#define TIME_MATCH_THRESHOLD 2e-3

extern gps_time_t tor;
extern sbp_state_t sbp_state;

void printmsg();
void sbp_setup(void);
void process_sbp();

#endif /* SBP_H_ */
