/*
 * sbp.h
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#ifndef SBP_H_
#define SBP_H_

#include <fstream>
#include <map>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/sbp.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/time.h>
#include <libswiftnav/track.h>
#ifdef __cplusplus
}
#endif

#define TIME_MATCH_THRESHOLD 2e-3

extern msg_pos_ecef_t			 pos_ecef;
extern msg_pos_llh_t      		 pos_llh;

extern gps_time_t tor;
extern sbp_state_t sbp_state;

extern gps_time_t time_obs;
extern u8 nr_obs;
extern ephemeris_t e;
extern std::map<uint16_t, ephemeris_t> ephemerisMap;
extern navigation_measurement_t nav_m[MAX_CHANNELS];
extern double sat_dists[MAX_CHANNELS];

extern std::vector<std::vector<double> > ecef_pos;
extern std::vector<std::vector<double> > llh_pos;
extern std::vector<std::vector<double> > ecef_vel;
extern std::vector<std::vector<double> > ned_vel;
extern std::vector<std::vector<std::vector<double> > > observations;
extern std::vector<std::vector<std::vector<double> > > ephemerides;

void ephemerisVector();
void printmsg();
void printEphMsg();
void sbp_setup(void);
void process_sbp();
void writecsv();
void readcsv();
bool validNrSats();

#endif /* SBP_H_ */
