/*
 * signal.cpp
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */
#include <assert.h>
#include "signal.h"

typedef struct {
  u16 constellation_start_index;
  u16 global_start_index;
} code_table_element_t;

static code_table_element_t code_table[CODE_COUNT];

static const u16 code_signal_counts[CODE_COUNT] = {
  [CODE_GPS_L1CA] = PLATFORM_SIGNAL_COUNT_GPS_L1CA,
  [CODE_GPS_L2CM] = PLATFORM_SIGNAL_COUNT_GPS_L2CM,
  [CODE_SBAS_L1CA] = PLATFORM_SIGNAL_COUNT_SBAS_L1CA,
  [CODE_GLO_L1CA] = PLATFORM_SIGNAL_COUNT_GLO_L1CA,
  [CODE_GLO_L2CA] = PLATFORM_SIGNAL_COUNT_GLO_L2CA,
};

u16 sid_to_global_index(gnss_signal_t sid)
{
  assert(code_supported(sid.code));
  return code_table[sid.code].global_start_index +
      sid_to_code_index(sid);
}

bool sid_supported(gnss_signal_t sid)
{
  /* Verify general validity */
  if (!sid_valid(sid))
    return false;

  /* Verify that the code is supported on this platform */
  if (!code_supported(sid.code))
    return false;

  return true;
}

bool code_supported(enum code code)
{
  /* Verify general validity */
  if (!code_valid(code))
    return false;
  return true;
}

