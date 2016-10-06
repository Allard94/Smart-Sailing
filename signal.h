/*
 * signal.h
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

extern "C"{
#include <libswiftnav/signal.h>
}

#define CODE_GPS_L1CA_SUPPORT     1
#define CODE_GPS_L2CM_SUPPORT     0
#define CODE_SBAS_L1CA_SUPPORT    1
#define CODE_GLO_L1CA_SUPPORT     0
#define CODE_GLO_L2CA_SUPPORT     0

#define PLATFORM_SIGNAL_COUNT_GPS_L1CA      (CODE_GPS_L1CA_SUPPORT ?          \
                                             NUM_SIGNALS_GPS_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GPS_L2CM      (CODE_GPS_L2CM_SUPPORT ?          \
                                             NUM_SIGNALS_GPS_L2CM : 0)
#define PLATFORM_SIGNAL_COUNT_SBAS_L1CA     (CODE_SBAS_L1CA_SUPPORT ?         \
                                             NUM_SIGNALS_SBAS_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GLO_L1CA      (CODE_GLO_L1CA_SUPPORT ?          \
                                             NUM_SIGNALS_GLO_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GLO_L2CA      (CODE_GLO_L2CA_SUPPORT ?          \
                                             NUM_SIGNALS_GLO_L2CA : 0)

#define PLATFORM_SIGNAL_COUNT_GPS     (PLATFORM_SIGNAL_COUNT_GPS_L1CA +       \
                                       PLATFORM_SIGNAL_COUNT_GPS_L2CM)
#define PLATFORM_SIGNAL_COUNT_SBAS    (PLATFORM_SIGNAL_COUNT_SBAS_L1CA)
#define PLATFORM_SIGNAL_COUNT_GLO     (PLATFORM_SIGNAL_COUNT_GLO_L1CA +       \
                                       PLATFORM_SIGNAL_COUNT_GLO_L2CA)

#define PLATFORM_SIGNAL_COUNT         (PLATFORM_SIGNAL_COUNT_GPS +            \
                                       PLATFORM_SIGNAL_COUNT_SBAS +           \
                                       PLATFORM_SIGNAL_COUNT_GLO)

u16 sid_to_global_index(gnss_signal_t sid);
bool sid_supported(gnss_signal_t sid);
bool code_supported(enum code code);

#endif /* SIGNAL_H_ */
