/*
 * sbp.c
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

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

gps_time_t time_obs;

u8 nr_obs;

navigation_measurement_t nav_m[MAX_CHANNELS];

ephemeris_t e;

std::vector<ephemeris_t> ephemerisis;
std::map<uint16_t, ephemeris_t> ephemerisMap;
std::vector<std::vector<double> > ecef_pos;
std::vector<std::vector<double> > llh_pos;
//std::vector<std::vector<double> > ecef_vel;
std::vector<std::vector<double> > ned_vel;
std::vector<std::vector<std::vector<double> > > observations;
std::vector<std::vector<std::vector<double> > > ephemerides;


/* Messages */
msg_ephemeris_dep_d_t    eph_gps;
msg_pos_ecef_t			 pos_ecef;
msg_pos_llh_t      		 pos_llh;
msg_vel_ecef_t			 vel_ecef;
msg_vel_ned_t			 vel_ned;
msg_obs_t				 obss;

/* The callback nodes */
static sbp_msg_callbacks_node_t eph_gps_node;
static sbp_msg_callbacks_node_t pos_ecef_node;
static sbp_msg_callbacks_node_t pos_llh_node;
static sbp_msg_callbacks_node_t vel_ecef_node;
static sbp_msg_callbacks_node_t vel_ned_node;
static sbp_msg_callbacks_node_t obs_node;

/* Prints messages */
void printmsg(){
//	printf("x: %f\n", pos_ecef.x);
//	printf("y: %f\n", pos_ecef.y);
//	printf("z: %f\n", pos_ecef.z);
//	printf("Number of satellites: %d\n", pos_llh.n_sats);
//	printf("Total:  %u\n", obs_in_msg);
//	printf("Epoch: %f\n", tor.tow);
//	for(u8 i =0; i < pos_llh.n_sats; i++){
//		printf("Satellite: %d\n", i);
//		printf("Prn: %u", nav_m[i].sid.sat);
//		printf(", Pseudorange: %f", nav_m[i].raw_pseudorange);
//		printf(", Carrierphase %f\n", nav_m[i].raw_carrier_phase);
//		printf("Prn eph: %u\n", e.sid.sat);
//		printf("Tgd: %f\n", es[i].kepler.tgd);
//		printf("Crs: %f", es[i].kepler.crs);
//		printf(", Cus: %f", es[i].kepler.cus);
//		printf(", Cis: %f\n", es[i].kepler.cis);
//		printf("Crc: %f", es[i].kepler.crc);
//		printf(", Cuc: %f", es[i].kepler.cuc);
//		printf(", Cic: %f\n", es[i].kepler.cic);
//		printf("Dn: %f", es[i].kepler.dn);
//		printf(", M0: %f", es[i].kepler.m0);
//		printf(", Ecc: %f\n", es[i].kepler.ecc);
//		printf("Sqrta: %f", es[i].kepler.sqrta);
//		printf(", Omega0: %f", es[i].kepler.omega0);
//		printf(", Omegadot: %f\n", es[i].kepler.omegadot);
//		printf("W: %f", es[i].kepler.w);
//		printf(", Inc: %f", es[i].kepler.inc);
//		printf(", Incdot: %f\n", es[i].kepler.inc_dot);
//		printf("Af0: %f", es[i].kepler.af0);
//		printf(", Af1: %f", es[i].kepler.af1);
//		printf(", Af2: %f\n", es[i].kepler.af2);
//		printf("ToeTime: %f", es[i].toe.tow);
//		printf(", Week: %d\n", es[i].toe.wn);
//		printf("TocTime: %f", es[i].kepler.toc.tow);
//		printf(", Week: %d\n", es[i].kepler.toc.wn);
//		printf("Valid: %u", es[i].valid);
//		printf(", Healthy: %u\n", es[i].healthy);
//		printf("Sid: %u", es[i].sid.sat);
//		printf(", Iode: %u", es[i].kepler.iode);
//		printf(", Iodc: %u\n", es[i].kepler.iodc);
		//printf("valid: %u\n", e.valid);
//	}
}

void ephemerisVector(){
	int times = 0;
	for(auto &e : ephemerisMap){
		printf("%u\n", e.second.sid.sat);
		printf("%f\n", e.second.kepler.af0);
		printf("%f\n", e.second.kepler.af1);
		printf("%f\n", e.second.kepler.af2);
		printf("%u\n", e.second.healthy);
		times++;
		std::cout << "times: " << times << std::endl;
	}
}

bool validNrSats(){
	for(int i = 0; i < pos_ecef.n_sats; i++){
		for(auto &e : ephemerisMap){
			if(nav_m[i].sid.sat == e.second.sid.sat){
				break;
			}
		}
		return false;
	}
	return true;
}

/* The ephemeris callback. */
void sbp_eph_callback(u16 sender_id, u8 len, u8 msg[], void *context){
	if (len != sizeof(msg_ephemeris_dep_d_t)) {
	    log_warn("Received bad ephemeris from peer");
	    return;
	}
	unpack_ephemeris((msg_ephemeris_dep_d_t *)msg, &e);
	ephemerisMap[e.sid.sat] = e;
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
   }
   else if (prev_tor.tow != tor.tow ||
               prev_tor.wn != tor.wn ||
               prev_count + 1 != count) {
      log_info("Dropped one of the observation packets! Skipping this sequence.");
      prev_count = -1;
      return;
    }
   	else {
      prev_count = count;
    }


   obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

   if(count == 0){
	   nr_obs = 0;
	   time_obs = tor;
   }
   packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
   for (u8 i=0; i<obs_in_msg; i++) {
       navigation_measurement_t *nm = &nav_m[nr_obs];

       /* Unpack the observation into a navigation_measurement_t. */
       unpack_obs_content(&obs[i], &nm->raw_pseudorange, &nm->raw_carrier_phase,
                          &nm->snr, &nm->lock_counter, &nm->sid);

       /* Set the time */
       nm->tot = tor;
       nm->tot.tow -= nm->raw_pseudorange / GPS_C;
       normalize_gps_time(&nm->tot);

       //const ephemeris_t *e = ephemeris_get(nm->sid);
       nr_obs++;
     }
}

/* The ECEF position callback */
void sbp_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_ecef = *(msg_pos_ecef_t *)msg;
}


/* The LLH position callback. */
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}

void sbp_vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ecef = *(msg_vel_ecef_t *)msg;
}

void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}

/* Initializes the sbp_state and registers callbacks. */
void sbp_setup(void){
	sbp_state_init(&sbp_state);

	sbp_register_callback(&sbp_state, SBP_MSG_EPHEMERIS_DEP_D, &sbp_eph_callback,
	    	        	  NULL, &eph_gps_node);
	sbp_register_callback(&sbp_state, SBP_MSG_OBS, &sbp_obs_callback,
		                  NULL, &obs_node);
	sbp_register_callback(&sbp_state, SBP_MSG_POS_ECEF, &sbp_pos_ecef_callback,
		                  NULL, &pos_ecef_node);
    sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
	                      NULL, &pos_llh_node);
    sbp_register_callback(&sbp_state, SBP_MSG_VEL_ECEF, &sbp_vel_ecef_callback,
    		              NULL, &vel_ecef_node);
    sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
    	                  NULL, &vel_ned_node);

}

void writecsv(){
	int nr_of_ephs = 0;
	std::ofstream file;
	file.open("test6.csv", std::ofstream::out | std::ofstream::app);
	file << pos_ecef.x << "," << pos_ecef.y << "," << pos_ecef.z << "," << (int)pos_ecef.n_sats << ", " << pos_ecef.tow << "\n";
	file << pos_llh.lat << "," << pos_llh.lon << "," << pos_llh.height << "," << pos_llh.tow << "\n";
	file << vel_ecef.x << "," << vel_ecef.y << "," << vel_ecef.z << "," << vel_ecef.tow << "\n";
	file << vel_ned.n << "," << vel_ned.e << "," << vel_ned.d << "," << vel_ned.tow << "\n";
	for(int i = 0; i < pos_llh.n_sats; i++){
		file << nav_m[i].sid.sat << "," << nav_m[i].sid.code << "," << nav_m[i].raw_carrier_phase << "," << nav_m[i].raw_pseudorange << ",";
		file << nav_m[i].raw_doppler << "," << nav_m[i].lock_counter << "," <<nav_m[i].lock_time << "," << nav_m[i].snr << ",";
		for(int j = 0; j < 3; j++){
			file << nav_m[i].sat_pos[j] << "," << nav_m[i].sat_vel[j] << ",";
		}
		file << nav_m[i].tot.tow << "," << nav_m[i].tot.wn << "\n";
	}
	for(int i = pos_llh.n_sats; i < 11;i++){
		file << 0 << "\n";;
	}
	for(auto &e : ephemerisMap){
		file << e.second.kepler.tgd << "," << e.second.kepler.crs << "," << e.second.kepler.crc << "," << e.second.kepler.cus << "," << e.second.kepler.cuc << "," << e.second.kepler.cis << "," << e.second.kepler.cic << ",";
		file << e.second.kepler.dn << "," << e.second.kepler.m0 << "," << e.second.kepler.ecc << "," << e.second.kepler.sqrta << "," << e.second.kepler.omega0 << "," << e.second.kepler.omegadot << ",";
		file << e.second.kepler.w << "," << e.second.kepler.inc << "," << e.second.kepler.inc_dot << "," << e.second.kepler.af0 << "," << e.second.kepler.af1 << "," << e.second.kepler.af2 << ",";
		file << e.second.toe.tow << "," << e.second.toe.wn << "," << e.second.kepler.toc.tow << "," << e.second.kepler.toc.wn << "," << (int)e.second.valid << "," << (int)e.second.healthy << ",";
		file << e.second.sid.sat << "," << e.second.sid.code << "," << (int)e.second.kepler.iode << "," << (int)e.second.kepler.iodc << "," << e.second.fit_interval << "," << e.second.ura << "\n";
		nr_of_ephs++;
		std::cout << "nr_of_ephs: "<< nr_of_ephs <<std::endl;
	}
	for(int i = 0; i < 32 - nr_of_ephs; i++){
		file << 0 << "\n";
	}
	file.close();
}

void split(const std::string& s, char c, std::vector<std::string>& v) {
   std::string::size_type i = 0;
   std::string::size_type j = s.find(c);
   while (j != std::string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == std::string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}

void readcsv(){
	std::ifstream stream("test6.csv", std::ifstream::in);
	std::string line;
	int linenr = -1;
	int satindex = 0;
	int maxrcvsats = 11;
	int maxsats = 32;
	int modvalue = maxrcvsats + maxsats + 4;
	std::vector<double> ecef_pos_temp;
	std::vector<double> llh_pos_temp;
	std::vector<double> ecef_vel_temp;
	std::vector<double> ned_vel_temp;
	std::vector<double> observation;
	std::vector<double> ephemeris;
	std::vector<std::vector<double> > observations_temp;
	std::vector<std::vector<double> > ephemerides_temp;
	std::string s = "-2";
	while(getline(stream, line)){
		std::vector<std::string> data;
		split(line, ',', data);
		if(linenr == 2209){
			for(int i = 0; i < 5; i++){
				ecef_pos_temp.push_back(stod(data[i]));
			}
			ecef_pos.push_back(ecef_pos_temp);
			ecef_pos_temp.clear();
		}
		else if(linenr > 2209){
			if(linenr % modvalue == 0){
				for(int i = 0; i < 5; i++){
					ecef_pos_temp.push_back(stod(data[i]));
				}
				satindex++;
				ecef_pos.push_back(ecef_pos_temp);
				ecef_pos_temp.clear();
			}
			else if(linenr % modvalue == 1){
				for(int i = 0; i < 4; i++){
					llh_pos_temp.push_back(stod(data[i]));
				}
				llh_pos.push_back(llh_pos_temp);
				llh_pos_temp.clear();
			}
//			else if(linenr % modvalue == 2){
//				for(int i = 0; i < 4; i++){
//					ecef_vel_temp.push_back(stod(data[i]));
//				}
//				ecef_vel.push_back(ecef_vel_temp);
//				ecef_vel_temp.clear();
//			}
			else if(linenr % modvalue == 3){
				for(int i = 0; i < 4; i++){
					ned_vel_temp.push_back(stod(data[i]));
				}
				ned_vel.push_back(ned_vel_temp);
				ned_vel_temp.clear();
			}
			for(int i = 4; i < maxrcvsats + 4; i++){
				if(linenr % modvalue == i){
					//if(data.size() == 0){
					//	break;
					//}
					if(std::stoi(data[0]) == 0){
						break;
					}
					else{
						for(int j = 0; j < 16; j++){
							observation.push_back(std::stod(data[j]));
						}
					}
					observations_temp.push_back(observation);
					observation.clear();
				}
			}
			if(linenr % modvalue == maxrcvsats + 4){
				observations.push_back(observations_temp);
				observations_temp.clear();
			}
			for(int i = maxrcvsats + 4; i < maxsats + maxrcvsats + 4; i++){

				if(linenr % modvalue == i){
					//if(data.size() == 0){
					//	break;
					//}
					if(std::stoi(data[0]) == 0){
						break;
					}
					else{
						for(uint8_t j = 0; j < data.size(); j++){
							ephemeris.push_back(std::stod(data[j]));
						}
						ephemerides_temp.push_back(ephemeris);
						ephemeris.clear();
					}
				}
			}
			if(linenr % modvalue == maxrcvsats + 35){
				ephemerides.push_back(ephemerides_temp);
				ephemerides_temp.clear();
			}
		}
		linenr++;
	}
	std::cout << linenr << std::endl;
	std::cout << satindex << std::endl;
//	for(int i = 0; i < ecef_pos.size(); i++){
//		std::cout << ecef_pos[i] << std::endl;
//	}
//	for(int i = 0, j = 0; i <= satindex; i++, j++){
//		for(int k = 0; k < observations[i][j].size(); k++){
//			std::cout << observations[i][0][k] << "   ";
//		}
//		std::cout << std::endl;
//	}
}



