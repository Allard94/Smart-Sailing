/*
 * spb_functions.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifdef __cplusplus
extern "C" {
#endif
#include <libswiftnav/constants.h>
#include <libswiftnav/signal.h>
#ifdef __cplusplus
}
#endif

#include "sbp_functions.h"

ephemeris_t es[255];

/* Converts sbp_gnss_signal_t to gnss_signal_t
 * Returns: gnss_signal_t.
 */
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

typedef struct {
  u16 constellation_start_index;
  u16 global_start_index;
} code_table_element_t;
static code_table_element_t code_table[CODE_COUNT];

u16 sid_to_global_index(gnss_signal_t sid)
{

  return code_table[sid.code].global_start_index +
      sid_to_code_index(sid);
}

void ephemeris_new(ephemeris_t *e)
{
	u32 index = sid_to_global_index(e->sid);
	es[index] = *e;
}
ephemeris_t *ephemeris_get(gnss_signal_t sid)
{
	//sid.sat -= 200; /* In simulation mode */
	return &es[sid_to_global_index(sid)];
}
/* Unpacks ephemeris content */
void unpack_ephemeris(const msg_ephemeris_dep_d_t *msg, ephemeris_t *e)
{
	  e->kepler.tgd       = msg->tgd;
	  e->kepler.crs       = msg->c_rs;
	  e->kepler.crc       = msg->c_rc;
	  e->kepler.cuc       = msg->c_uc;
	  e->kepler.cus       = msg->c_us;
	  e->kepler.cic       = msg->c_ic;
	  e->kepler.cis       = msg->c_is;
	  e->kepler.dn        = msg->dn;
	  e->kepler.m0        = msg->m0;
	  e->kepler.ecc       = msg->ecc;
	  e->kepler.sqrta     = msg->sqrta;
	  e->kepler.omega0    = msg->omega0;
	  e->kepler.omegadot  = msg->omegadot;
	  e->kepler.w         = msg->w;
	  e->kepler.inc       = msg->inc;
	  e->kepler.inc_dot   = msg->inc_dot;
	  e->kepler.af0       = msg->af0;
	  e->kepler.af1       = msg->af1;
	  e->kepler.af2       = msg->af2;
	  e->toe.tow          = msg->toe_tow;
	  e->toe.wn           = msg->toe_wn;
	  e->kepler.toc.tow   = msg->toc_tow;
	  e->kepler.toc.wn    = msg->toe_wn;
	  e->valid            = msg->valid;
	  e->healthy          = msg->healthy;
	  e->sid              = sid_from_sbp(msg->sid);
	  e->kepler.iode      = msg->iode;
	  e->kepler.iodc      = msg->iodc;
	  e->fit_interval     = 4 * 60 * 60; /* TODO: this is a work around until SBP updated */
	  e->ura              = 2.0f; /* TODO: this is a work around until SBP updated*/
	}

/* Unpacks observation header. */
void unpack_obs_header(const observation_header_t *msg, gps_time_t* t, u8* total, u8* count)
{
  t->tow = ((double)msg->t.tow) / MSG_OBS_TOW_MULTIPLIER;
  t->wn  = msg->t.wn;
  *total = (msg->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (msg->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

/* Unpacks observation content. */
void unpack_obs_content(const packed_obs_content_t *msg, double *P, double *L, double *snr, u16 *lock_counter, gnss_signal_t *sid)
{
  *P   = ((double)msg->P) / MSG_OBS_P_MULTIPLIER;
  *L   = -(((double)msg->L.i) + (((double)msg->L.f) / MSG_OSB_LF_MULTIPLIER));
  *snr = ((double)msg->cn0) / MSG_OBS_SNR_MULTIPLIER;
  *lock_counter = ((u16)msg->lock);
  *sid = sid_from_sbp(msg->sid);
}

