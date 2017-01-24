/*
 * rtklib.cpp
 *
 *  Created on: Oct 7, 2016
 *      Author: root
 */

#include "rtklib.h"
#include "sbp.h"
#include "sbp_functions.h"
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/time.h>
#ifdef __cplusplus
}
#endif

#include <math.h>

nav_t nav;
obs_t obs;
rnxctr_t rnx;
rtk_t rtk;
sta_t sta;

/* Initializes rtk navigation data.
 * The readsp3 function requires some attributes to be initialized.
 */
void nav_init(nav_t *nav){
	peph_t peph0 = {0};
	eph_t eph0 = {0,-1,-1};
    nav->peph = NULL;
    nav->eph =NULL;
	if(!(nav->peph =(peph_t  *)malloc(sizeof(peph_t )*MAXSAT )) ||
	   !(nav->eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT ))){
		freenav(nav, 8);
	}

	for (int i=0;i<MAXSAT ;i++) nav->eph [i]=eph0;
	nav->ne = 0;
}

/* Initializes rtklib observation data. */
void obs_init(obs_t * obs){
	obsd_t data0={{0}};
	obs->data = NULL;
	if (!(obs->data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS ))){
		printf("ERROR!");
	}
	for (int i=0;i<64 ;i++) obs->data[i]=data0;
	obs->nmax = 64;
	for(int i = 0; i < obs->nmax; i++){
		obs->data[i].rcv = 1;
	}
}

/* Assigns piksi observation data to rtklib observation data. */
void obs_assign(obs_t *obs){
	for(uint8_t i = 0; i < pos_ecef.n_sats; i++){
		obs->n = pos_ecef.n_sats;
		obs->data[i].sat = nav_m[i].sid.sat;
		obs->data[i].code[0] = nav_m[i].sid.code;
		obs->data[i].L[0] = nav_m[i].raw_carrier_phase;
		obs->data[i].P[0] = nav_m[i].raw_pseudorange;
		obs->data[i].D[0] = nav_m[i].raw_doppler;
		obs->data[i].LLI[0] = nav_m[i].lock_counter;
		obs->data[i].SNR[0] = nav_m[i].snr;
		obs->data[i].time = gpst2time(nav_m[i].tot.wn, nav_m[i].tot.tow);
	}
}

/* Initializes rtklib processing options. */
void rtk_opt_init(rtk_t * rtk){
	rtk->opt = prcopt_default;
	rtk->opt.mode = 6;					/* Option mode:               	Kinematic Mode      */
	rtk->opt.tropopt = 3;				/* Troposphere option:        	ZTD estimation      */
	rtk->opt.sateph = 0;				/* Satellite ephemeris/clock: 	Broadcast ephemeris */
	rtk->opt.nf = 1;				    /* Number of frequencies:       L1				    */
	//rtk->opt.ionoopt = 5;				/* Ionosphere option:			IONEX TEC model     */
	rtk->opt.ionoopt = 1;
}

void rtk_sol_assign(rtk_t * rtk){
	rtk->sol.rr[0] = pos_ecef.x;
	rtk->sol.rr[1] = pos_ecef.y;
	rtk->sol.rr[2] = pos_ecef.z;
}

void nav_eph_assign(nav_t * nav){
	int sats = 0;
	for(uint8_t i = 0; i < pos_llh.n_sats; i++){
		for(auto &e : ephemerisMap){
			if(nav_m[i].sid.sat == e.second.sid.sat){
				sats++;
				nav->eph[i].tgd[0] = e.second.kepler.tgd;
				nav->eph[i].crs = e.second.kepler.crs;
				nav->eph[i].crc = e.second.kepler.crc;
				nav->eph[i].cus = e.second.kepler.cus;
				nav->eph[i].cuc = e.second.kepler.cuc;
				nav->eph[i].cis = e.second.kepler.cis;
				nav->eph[i].cic = e.second.kepler.cic;
				nav->eph[i].deln = e.second.kepler.dn;
				nav->eph[i].M0 = e.second.kepler.m0;
				nav->eph[i].e = e.second.kepler.ecc;
				nav->eph[i].A = pow(e.second.kepler.sqrta, 2);
				nav->eph[i].OMG0 = e.second.kepler.omega0;
				nav->eph[i].OMGd = e.second.kepler.omegadot;
				nav->eph[i].omg = e.second.kepler.w;
				nav->eph[i].i0 = e.second.kepler.inc;
				nav->eph[i].idot = e.second.kepler.inc_dot;
				nav->eph[i].f0 = e.second.kepler.af0;
				nav->eph[i].f1 = e.second.kepler.af1;
				nav->eph[i].f2 = e.second.kepler.af2;
				nav->eph[i].toes = e.second.toe.tow;
				nav->eph[i].toe = gpst2time(e.second.toe.wn, e.second.toe.tow);
				nav->eph[i].toc = gpst2time(e.second.kepler.toc.wn, e.second.kepler.toc.tow);
				if(e.second.healthy == 1){
					nav->eph[i].svh = e.second.healthy - 1;
				}
				else{
					nav->eph[i].svh = e.second.healthy + 1;
				}
				nav->eph[i].sat = e.second.sid.sat;
				nav->eph[i].code = e.second.sid.code;
				nav->eph[i].iode = e.second.kepler.iode;
				nav->eph[i].iodc = e.second.kepler.iodc;
				nav->eph[i].fit = e.second.fit_interval;
				nav->eph[i].sva = e.second.ura;
			}
		}
	}
	nav->n = sats;
}

void nav_iono_assign(nav_t * nav){
	nav->ion_gps[0] = iono.a0;
	nav->ion_gps[1] = iono.a1;
	nav->ion_gps[2] = iono.a2;
	nav->ion_gps[3] = iono.a3;
	nav->ion_gps[4] = iono.b0;
	nav->ion_gps[5] = iono.b1;
	nav->ion_gps[6] = iono.b2;
	nav->ion_gps[7] = iono.b3;
}

void obs_test_assign(obs_t * obs, int datanr){
	for(uint8_t i = 0; i < observations[datanr].size(); i++){
		obs->n = ecef_pos[datanr][3];
		auto &sat = observations[datanr];
		auto &sat2 = sat[i];
		auto &sat3 = sat2[0];
		obs->data[i].sat = observations[datanr][i][0];
		obs->data[i].code[0] = observations[datanr][i][1];
		obs->data[i].L[0] = observations[datanr][i][2];
		obs->data[i].P[0] = observations[datanr][i][3];
		//obs->data[i].D[0] = observations[datanr][i][4];
		obs->data[i].LLI[0] = observations[datanr][i][5];
		obs->data[i].SNR[0] = observations[datanr][i][7];
		obs->data[i].time = gpst2time(observations[datanr][i][15], observations[datanr][i][14]);
	}
}

void nav_eph_test_assign(nav_t * nav, obs_t * obs, int datanr){
	nav->n = observations[datanr].size();
	gps_time_t begin;
	gps_time_t end;
	double clock_rate_err;
	for(uint8_t i = 0; i < observations[datanr].size(); i++){
		for(int j = 0; j < ephemerides[datanr].size(); j++){
			if(observations[datanr][i][0] == ephemerides[datanr][j][25]){
				nav->eph[i].tgd[0] = ephemerides[datanr][j][0];
				nav->eph[i].crs = ephemerides[datanr][j][1];
				nav->eph[i].crc = ephemerides[datanr][j][2];
				nav->eph[i].cus = ephemerides[datanr][j][3];
				nav->eph[i].cuc = ephemerides[datanr][j][4];
				nav->eph[i].cis = ephemerides[datanr][j][5];
				nav->eph[i].cic = ephemerides[datanr][j][6];
				nav->eph[i].deln = ephemerides[datanr][j][7];
				nav->eph[i].M0 = ephemerides[datanr][j][8];
				nav->eph[i].e = ephemerides[datanr][j][9];
				nav->eph[i].A = pow(ephemerides[datanr][j][10], 2);
				nav->eph[i].OMG0 = ephemerides[datanr][j][11];
				nav->eph[i].OMGd = ephemerides[datanr][j][12];
				nav->eph[i].omg = ephemerides[datanr][j][13];
				nav->eph[i].i0 = ephemerides[datanr][j][14];
				nav->eph[i].idot = ephemerides[datanr][j][15];
				nav->eph[i].f0 = ephemerides[datanr][j][16];
				nav->eph[i].f1 = ephemerides[datanr][j][17];
				nav->eph[i].f2 = ephemerides[datanr][j][18];
				nav->eph[i].toes = ephemerides[datanr][j][21];
				nav->eph[i].toe = gpst2time(ephemerides[datanr][j][20], ephemerides[datanr][j][19]);
				nav->eph[i].toc = gpst2time(ephemerides[datanr][j][22], ephemerides[datanr][j][21]);
				nav->eph[i].svh = ephemerides[datanr][j][24] - 1;
				nav->eph[i].sat = ephemerides[datanr][j][25];
				nav->eph[i].code = ephemerides[datanr][j][26];
				nav->eph[i].iode = ephemerides[datanr][j][27];
				nav->eph[i].iodc = ephemerides[datanr][j][28];
				nav->eph[i].fit = ephemerides[datanr][j][29];
				nav->eph[i].sva = ephemerides[datanr][j][30];
				begin.tow = ephemerides[datanr][j][19];
				begin.wn = ephemerides[datanr][j][20];
				end.tow = ephemerides[datanr][j][21];
				end.wn = ephemerides[datanr][j][22];
				double dt = gpsdifftime(&begin, &end);
				clock_rate_err = ephemerides[datanr][j][17] + 2.0 * dt * ephemerides[datanr][j][18];
				obs->data[i].D[0] = clock_rate_err * GPS_L1_HZ;
				printf("doppler: %f\n", obs->data[i].D[0]);
			}
		}
	}
}

void nav_iono_test_assign(nav_t * nav, int datanr){
	nav->ion_gps[0] = ionosphere[datanr][0];
	nav->ion_gps[1] = ionosphere[datanr][1];
	nav->ion_gps[2] = ionosphere[datanr][2];
	nav->ion_gps[3] = ionosphere[datanr][3];
	nav->ion_gps[4] = ionosphere[datanr][4];
	nav->ion_gps[5] = ionosphere[datanr][5];
	nav->ion_gps[6] = ionosphere[datanr][6];
	nav->ion_gps[7] = ionosphere[datanr][7];
}

void rtk_sol_test_assign(rtk_t * rtk, int datanr){
	rtk->sol.rr[0] = ecef_pos[datanr][0];
	rtk->sol.rr[1] = ecef_pos[datanr][1];
	rtk->sol.rr[2] = ecef_pos[datanr][2];
}


