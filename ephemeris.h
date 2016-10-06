/*
 * ephemeris.h
 *
 *  Created on: Oct 5, 2016
 *      Author: root
 */

#ifndef EPHEMERIS_H_
#define EPHEMERIS_H_

extern "C"{
#include <libswiftnav/ephemeris.h>
}

ephemeris_t *ephemeris_get(gnss_signal_t sid);


#endif /* EPHEMERIS_H_ */
