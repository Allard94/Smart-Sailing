/*
 * sbp_functions.h
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifndef SBP_FUNCTIONS_H_
#define SBP_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/gnss_signal.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libswiftnav/pvt.h>

#include <libswiftnav/signal.h>
#include <libswiftnav/time.h>
#ifdef __cplusplus
}
#endif
#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK
#define MSG_OBS_TOW_MULTIPLIER ((double)1000.0)

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_SNR_MULTIPLIER ((float)4)
#define MSG_OSB_LF_MULTIPLIER ((double) (1<<8))

void sbp_make_pos_llh(msg_pos_llh_t *pos_llh, const gnss_solution *soln, u8 flags);
gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from);
void unpack_obs_header(const observation_header_t *msg, gps_time_t* t, u8* total, u8* count);
void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L, double *snr, u16 *lock_counter, gnss_signal_t *sid);

#endif /* SBP_FUNCTIONS_H_ */
