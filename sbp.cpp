/*
 * sbp.c
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#include <stdio.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/navigation.h>
#include <libsbp/observation.h>

#include <libswiftnav/constants.h>
#ifdef __cplusplus
}
#endif

#include "port.h"
#include "sbp.h"
#include "sbp_functions.h"
#include "signal.h"

sbp_state_t sbp_state;


/* GPS time of observation. */
gps_time_t tor;

/* Total number of messages in the observation set / sequence. */
u8 total;

/* The current message number in the sequence. */
u8 count;

/* Number of observations in the message. */
u8 obs_in_msg;

/* Messages */
msg_pos_llh_t      		 pos_llh;
msg_obs_t				 obss;


/* Observation data gets stored in this struct. */
typedef struct {

	/* GPS system time of the observation. */
  gps_time_t tor;

  /* Number of observations in the set. */
  u8 n;

  /** Set of observations. */
  navigation_measurement_t nm[MAX_CHANNELS];

  /* Distances to each satellite based on `pos_ecef` and `nm`.
   * Used for observation propagation. */
  double sat_dists[MAX_CHANNELS];
} obss_t;

obss_t obst;

/* The callback nodes */
static sbp_msg_callbacks_node_t pos_llh_node;
static sbp_msg_callbacks_node_t vel_ned_node;
static sbp_msg_callbacks_node_t obs_node;

/* Prints messages */
void printmsg(){
	printf("Latitude: %f\n", pos_llh.lat);
	printf("Longtitude: %f\n", pos_llh.lon);
	printf("Altitude: %f\n", pos_llh.height);
	printf("Number of satellites: %d\n", pos_llh.n_sats);
	printf("Total:  %u\n", obs_in_msg);
	printf("Epoch: %f\n", tor.tow);
	for(u8 i =0; i < pos_llh.n_sats; i++){
		printf("Prn: %u", obst.nm[i].sid.sat);
		printf(", Pseudorange: %f", obst.nm[i].raw_pseudorange);
		printf(", Carrierphase %f\n", obst.nm[i].raw_carrier_phase);
	}
}

/* The observation callback. */
void sbp_obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
   obss = *(msg_obs_t *)msg;

   static s16 prev_count = 0;

   static gps_time_t prev_tor = {.tow = 0.0, .wn = 0};


   unpack_obs_header((observation_header_t*)msg, &tor, &total, &count);

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


   obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

   if(count == 0){
	   obst.n = 0;
	   obst.tor = tor;
   }
   packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
   for (u8 i=0; i<obs_in_msg; i++) {
       navigation_measurement_t *nm = &obst.nm[obst.n];

       /* Unpack the observation into a navigation_measurement_t. */
       unpack_obs_content(&obs[i], &nm->raw_pseudorange, &nm->raw_carrier_phase,
                          &nm->snr, &nm->lock_counter, &nm->sid);

       /* Set the time */
       nm->tot = tor;
       nm->tot.tow -= nm->raw_pseudorange / GPS_C;
       normalize_gps_time(&nm->tot);

       obst.n++;
     }
}

/* The position callback. */
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}

/* Initializes the sbp_state and registers callbacks. */
void sbp_setup(void){
	sbp_state_init(&sbp_state);

	sbp_register_callback(&sbp_state, SBP_MSG_OBS, &sbp_obs_callback,
		                  NULL, &obs_node);
    sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
	                      NULL, &pos_llh_node);
}




