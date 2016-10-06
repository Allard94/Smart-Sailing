/*
 * observation.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C"{
#include <libsbp/observation.h>
#include <libsbp/sbp.h>

#include <libswiftnav/constants.h>
//#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
//#include <libswiftnav/pvt.h>
#include <libswiftnav/signal.h>
}

#include "ephemeris.h"
#include "observation.h"
#include "sbp_functions.h"
#include "signal.h"

#define TIME_MATCH_THRESHOLD 2e-3

double soln_freq = 10.0;
u32 obs_output_divisor = 2;
extern bool disable_raim;

obss_t base_obss;

//static void update_obss(obss_t *new_obss)
//{
//	/* Ensure observations sorted by PRN. */
//	qsort(new_obss->nm, new_obss->n, sizeof(navigation_measurement_t), nav_meas_cmp);
//
//	/* Lock mutex before modifying base_obss.
//	 * NOTE: We didn't need to lock it before reading in THIS context as this
//	 * is the only thread that writes to base_obss. */
//	//chMtxLock(&base_obs_lock);
//
//	/* Create a set of navigation measurements to store the previous
//	 * observations. */
//	static u8 n_old = 0;
//	static gps_time_t tor_old = {0, 0};
//	static navigation_measurement_t nm_old[MAX_CHANNELS];
//
//    /* Fill in the navigation measurements in base_obss, using TDCP method to
//     * calculate the Doppler shift. */
//    base_obss.n = tdcp_doppler(new_obss->n, new_obss->nm,
//                               n_old, nm_old, base_obss.nm,
//                               gpsdifftime(&new_obss->tor, &tor_old));
//
//    /* Copy over sender ID. */
//    base_obss.sender_id = new_obss->sender_id;
//
//    /* Copy the current observations over to nm_old so we can difference
//     * against them next time around. */
//    memcpy(nm_old, new_obss->nm, new_obss->n * sizeof(navigation_measurement_t));
//    n_old = new_obss->n;
//    tor_old = new_obss->tor;
//
//    /* Copy over the time. */
//    base_obss.tor = new_obss->tor;
//
//    u8 has_pos_old = base_obss.has_pos;
//    if (base_obss.n >= 4) {
//	    gnss_solution soln;
//	    dops_t dops;
//
//	    /* Calculate a position solution. */
//        /* disable_raim controlled by external setting (see solution.c). */
//	    s32 ret = calc_PVT(base_obss.n, base_obss.nm, disable_raim, &soln, &dops);
//
//	    if (ret >= 0 && soln.valid) {
//		    /* The position solution calculation was sucessful. Unfortunately the
//		     * single point position solution is very noisy so lets smooth it if we
//		     * have the previous position available. */
//		    if (has_pos_old) {
//		  	    /* TODO Implement a real filter for base position (potentially in
//           	     observation space), so we can do away with this terrible excuse
//           	     for smoothing. */
//			    base_obss.pos_ecef[0] = 0.99995 * base_obss.pos_ecef[0]
//                                  + 0.00005 * soln.pos_ecef[0];
//			    base_obss.pos_ecef[1] = 0.99995 * base_obss.pos_ecef[1]
//                                  + 0.00005 * soln.pos_ecef[1];
//			    base_obss.pos_ecef[2] = 0.99995 * base_obss.pos_ecef[2]
//                                  + 0.00005 * soln.pos_ecef[2];
//		    }
//		    else {
//			    memcpy(base_obss.pos_ecef, soln.pos_ecef, 3 * sizeof(double));
//		    }
//		    base_obss.has_pos = 1;
//
//	    }
//	    else {
//	    	base_obss.has_pos = 0;
//	    	/* TODO(dsk) check for repair failure */
//	    	/* There was an error calculating the position solution. */
//	    	log_warn("Error calculating base station position: (%s).", pvt_err_msg[-ret-1]);
//	    }
//    }
//    else {
//    	base_obss.has_pos = 0;
//    }
//
//  /* If the base station position is known then calculate the satellite ranges.
//   * This calculation will be used later by the propagation functions. */
//  if (base_obss.has_pos) {
//    /* Check if the base station has sent us its position explicitly via a
//     * BASE_POS SBP message (as indicated by #base_pos_known).
//     * No need to lock before reading here as base_pos_* is only written
//     * from this thread (SBP).
//     */
//
//    for (u8 i=0; i < base_obss.n; i++) {
//      base_obss.sat_dists[i] = vector_distance(3, base_obss.nm[i].sat_pos,
//                                               base_obss.pos_ecef);
//    }
//  }
//
//  /* Unlock base_obss mutex. */
//  //chMtxUnlock(&base_obs_lock);
//
//  /* Signal that a complete base observation has been received. */
//  //chBSemSignal(&base_obs_received);
//}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	(void) context;

	  /* Keep track of where in the sequence of messages we were last time around
	   * so we can verify we haven't dropped a message. */
	  static s16 prev_count = 0;

	  static gps_time_t prev_tor = {.tow = 0.0, .wn = 0};

	  /* As we receive observation messages we assemble them into a working
	   * `obss_t` (`base_obss_rx`) so as not to disturb the global `base_obss`
	   * state that may be in use. */
	  static obss_t base_obss_rx;
	  base_obss_rx.has_pos = 0;

	  /* An SBP sender ID of zero means that the messages are relayed observations
	   * from the console, not from the base station. We don't want to use them and
	   * we don't want to create an infinite loop by forwarding them again so just
	   * ignore them. */
	  if (sender_id == 0) {
	    return;
	  }

	  /* We set the sender_id */
	  base_obss_rx.sender_id = sender_id;

	  /* Relay observations using sender_id = 0. */
	  //sbp_send_msg_(SBP_MSG_OBS, len, msg, 0);

	  /* GPS time of observation. */
	  gps_time_t tor;
	  /* Total number of messages in the observation set / sequence. */
	  u8 total;
	  /* The current message number in the sequence. */
	  u8 count;

	  /* Decode the message header to get the time and how far through the sequence
	   * we are. */
	  unpack_obs_header((observation_header_t*)msg, &tor, &total, &count);

	  /* Check to see if the observation is aligned with our internal observations,
	   * i.e. is it going to time match one of our local obs. */
	  u32 obs_freq = soln_freq / obs_output_divisor;
	  double epoch_count = tor.tow * obs_freq;
	  double dt = fabs(epoch_count - round(epoch_count)) / obs_freq;
	  if (dt > TIME_MATCH_THRESHOLD) {
	    log_warn("Unaligned observation from base station ignored, "
	             "tow = %.3f, dt = %.3f", tor.tow, dt);
	    return;
	  }

	  /* Verify sequence integrity */
	  if (count == 0) {
	    prev_tor = tor;
	    prev_count = 0;
	  } else if (prev_tor.tow != tor.tow ||
	             prev_tor.wn != tor.wn ||
	             prev_count + 1 != count) {
	    log_info("Dropped one of the observation packets! Skipping this sequence.");
	    prev_count = -1;
	    return;
	  } else {
	    prev_count = count;
	  }

	  /* Calculate the number of observations in this message by looking at the SBP
	   * `len` field. */
	  u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

	  /* If this is the first packet in the sequence then reset the base_obss_rx
	   * state. */
	  if (count == 0) {
	    base_obss_rx.n = 0;
	    base_obss_rx.tor = tor;
	  }

	  /* Pull out the contents of the message. */
	  packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
	  for (u8 i=0; i<obs_in_msg; i++) {
	    gnss_signal_t sid = sid_from_sbp(obs[i].sid);
	    if (!sid_supported(sid))
	      continue;

	    navigation_measurement_t *nm = &base_obss_rx.nm[base_obss_rx.n];

	    /* Unpack the observation into a navigation_measurement_t. */
	    unpack_obs_content(&obs[i], &nm->raw_pseudorange, &nm->raw_carrier_phase, &nm->snr, &nm->lock_counter, &nm->sid);

	    /* Set the time */
	    nm->tot = tor;
	    nm->tot.tow -= nm->raw_pseudorange / GPS_C;
	    normalize_gps_time(&nm->tot);

	    /* Calculate satellite parameters using the ephemeris. */
	    const ephemeris_t *e = ephemeris_get(nm->sid);
	    u8 eph_valid;
	    s8 ss_ret;
	    double clock_err;
	    double clock_rate_err;

	    //ephemeris_lock();
	    eph_valid = ephemeris_valid(e, &nm->tot);
	    if (eph_valid) {
	      ss_ret = calc_sat_state(e, &nm->tot, nm->sat_pos, nm->sat_vel,
	                              &clock_err, &clock_rate_err);
	    }
	    //ephemeris_unlock();

	    if (!eph_valid || (ss_ret != 0)) {
	      continue;
	    }

	    /* Apply corrections to the raw pseudorange, carrier phase and Doppler. */
	    /* TODO Make a function to apply some of these corrections.
	     *      They are used in a couple places. */
	    nm->pseudorange = nm->raw_pseudorange + clock_err * GPS_C;
	    nm->carrier_phase = nm->raw_carrier_phase - clock_err * GPS_L1_HZ;

	    /* Used in tdcp_doppler */
	    nm->doppler = clock_rate_err * GPS_L1_HZ;

	    /* We also apply the clock correction to the time of transmit. */
	    nm->tot.tow -= clock_err;
	    normalize_gps_time(&nm->tot);

	    base_obss_rx.n++;
	  }

	  /* If we can, and all the obs have been received, update to using the new
	   * obss. */
//	  if (count == total - 1) {
//	    update_obss(&base_obss_rx);
//	  /* Calculate packet latency. */
//	    if (time_quality >= TIME_COARSE) {
//	      gps_time_t now = get_current_time();
//	      float latency_ms = (float) ((now.tow - tor.tow) * 1000.0);
//	      log_obs_latency(latency_ms);
//	    }
//	  }

}

//void obs_setup()
//{
//	 static sbp_msg_callbacks_node_t obs_packed_node;



