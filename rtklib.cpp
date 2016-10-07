/*
 * rtklib.cpp
 *
 *  Created on: Oct 7, 2016
 *      Author: root
 */

#include "rtklib.h"

nav_t nav;
obs_t obs;
rtk_t rtk;
sta_t sta;

/* Initializes rtk navigation data.
 * The readsp3 function requires some attributes to be initialized.
 */
void nav_init(nav_t *nav){
	peph_t peph0 = {0};
    nav->peph = NULL;
	if(!(nav->peph =(peph_t  *)malloc(sizeof(peph_t )*MAXSAT ))){
		freenav(nav, 8);
	}
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
	//obs->n = pos_llh.n_sats;
	//obs->n = obs_dep_b.header.n_obs;
//	obs->data->time = gpst2time((int)obs_dep_b.header.t.wn, (double)obs_dep_b.header.t.tow * 1000);
//	for(int i = 0; i < obs->n; i++){
//		obs->data[i].P[0] = obs_dep_b.obs[i].P;
//		obs->data[i].L[0] = obs_dep_b.obs[i].L.i;
//		obs->data[i].SNR[0] = obs_dep_b.obs[i].cn0;
//		obs->data[i].sat = ephemeris_gps.common.sid.sat;
//	}
}

/* Initializes rtklib processing options. */
void rtk_opt_init(rtk_t * rtk){
	rtk->opt = prcopt_default;
	rtk->opt.mode = 6;					/* Option mode:               	Kinematic Mode    */
	rtk->opt.tropopt = 3;				/* Troposphere option:        	ZTD estimation    */
	rtk->opt.dynamics = 1;				/* Dynamics mode:             	Velocity          */
	rtk->opt.sateph = 1;				/* Satellite ephemeris/clock: 	Precise ephemeris */
	rtk->opt.nf = 3;				    /* Number of frequencies:       L1+L2+L5		  */
}

