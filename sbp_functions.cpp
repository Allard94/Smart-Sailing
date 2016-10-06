/*
 * spb_functions.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifdef __cplusplus
extern "C" {
#endif
#include <libswiftnav/signal.h>
#ifdef __cplusplus
}
#endif

#include "sbp_functions.h"

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from)
{
  gnss_signal_t sid = {
    from.sat,
		  (code_t)from.code
  };



  /* Maintain legacy compatibility with GPS PRN encoding. Sat values for other
   * constellations are "real" satellite identifiers.
   */
  if (code_valid(sid.code) &&
     (code_to_constellation(sid.code) == CONSTELLATION_GPS)) {
    sid.sat += GPS_FIRST_PRN;
  }

  return sid;
}

void unpack_obs_header(const observation_header_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->tow = ((double)msg->t.tow) / MSG_OBS_TOW_MULTIPLIER;
  t->wn  = msg->t.wn;
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L,
                        double *snr, u16 *lock_counter, gnss_signal_t *sid)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = -(((double)msg->L.i) + (((double)msg->L.f) / MSG_OSB_LF_MULTIPLIER));
  *snr = ((double)msg->cn0) / MSG_OBS_SNR_MULTIPLIER;
  *lock_counter = ((u16)msg->lock);
  *sid = sid_from_sbp(msg->sid);
}
