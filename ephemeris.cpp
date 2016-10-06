/*
 * ephemeris.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#include <assert.h>
#include <thread>


#include "ephemeris.h"
#include "signal.h"




static ephemeris_t es[PLATFORM_SIGNAL_COUNT];

ephemeris_t *ephemeris_get(gnss_signal_t sid)
{
  assert(sid_supported(sid));
  return &es[sid_to_global_index(sid)];
}


