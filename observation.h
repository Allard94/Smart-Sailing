/*
 * observation.h
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifndef OBSERVATION_H_
#define OBSERVATION_H_

extern "C"{
#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>
}

typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[MAX_CHANNELS];
  /** Distances to each satellite based on `pos_ecef` and `nm`.
   * Used for observation propagation. */
  double sat_dists[MAX_CHANNELS];
} obss_t;

void obs_setup(void);

#endif /* OBSERVATION_H_ */
