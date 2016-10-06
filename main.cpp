/*
 * Main.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: root
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include <libserialport.h>
#include "rtklib/rtklib.h"
extern "C"
{
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/acquisition.h>
#include <libsbp/navigation.h>
#include <libsbp/piksi.h>
#include <libsbp/observation.h>
#include <libsbp/gnss_signal.h>
#include <libsbp/settings.h>
#include <libsbp/tracking.h>
#include <libswiftnav/time.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
}

#include "ephemeris.h"
#include "observation.h"
#include "port.h"
#include "sbp.h"

using namespace std;

void usage(char *prog_name) {
  fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}

void obsinit(obs_t *obs){
	obsd_t data0={{0}};
	obs->data = NULL;
	if (!(obs->data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS ))){
		cout << "ERROR!" <<endl;
	}
	for (int i=0;i<64 ;i++) obs->data[i]=data0;
	obs->nmax = 64;
	for(int i = 0; i < obs->nmax; i++){
		obs->data[i].rcv = 1;
		obs->data[i].sat = 14;
	}
}

void obsassign(obs_t *obs){
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

//void initnav(nav_t *nav){
//	peph_t peph0 = {0};
//  nav->peph = NULL;
//	if !(rnx->nav.eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT ){
//		freenav(nav, 8);
//	nav->ne = 0;
//}

int main(int argc, char **argv)
{

  rtk_t rtk;
  rtk.opt = prcopt_default;
  obs_t obs;
  nav_t nav;
  sta_t sta;
  rnxctr_t rnx;
  obsinit(&obs);


  rtk.opt.mode = 6;					/* Option mode:               	Kinematic Mode    */
  rtk.opt.tropopt = 3;				/* Troposphere option:        	ZTD estimation    */
  rtk.opt.dynamics = 1;				/* Dynamics mode:             	Velocity      */
  rtk.opt.sateph = 1;				/* Satellite ephemeris/clock: 	Precise ephemeris */
  rtk.opt.nf = 3;					/* number of frequencies		L1 + L2 + L5 */
  //rtk.opt.posopt = 4;
  //nav.peph
  char oopt;
  const char *rinexfile = "Local-20160928-135925.obs";
  const char *sp3file = "igu19165_06.sp3";
  int opt;



  FILE * pFile;

  rtkinit(&rtk, &rtk.opt);
  init_rnxctr(&rnx);
  sortobs(&rnx.obs);
  uniqnav(&rnx.nav);
  //initnav(&rnx.nav);
  readsp3(sp3file, &nav, 1);

  rtk.sol.rr[0] = -2700370;
  rtk.sol.rr[1] = -4292500;
  rtk.sol.rr[2] = 3855470;

  obs.data->P[0] = 246669057;
  obs.data->L[0] = 163269;
  obs.data->SNR[0] = 179;
  obs.data->time.time = 1475071165;
  obs.data->time.sec = 0.60000000000000142;
  rtkpos(&rtk, &obs.data[0], 9, &nav);
  strncpy(rnx.opt, "-GLss -SYS=G", sizeof(rnx.opt));


  //Latitude: 37.4303
  //longtitude: -122.172
  //height: 70.1352
  readrnx(rinexfile, 1, &rnx.opt[0], &rnx.obs, &rnx.nav, &rnx.sta);
  //pFile = fopen ("myfile.txt","w");
  //pppoutsolstat(&rtk, 3, pFile);
  //fclose(pFile);

  if (argc <= 1) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }

  while ((opt = getopt(argc, argv, "p:")) != -1) {
      switch (opt) {
        case 'p':
          serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!serial_port_name) {
            fprintf(stderr, "Cannot allocate memory!\n");
            exit(EXIT_FAILURE);
          }
          strcpy(serial_port_name, optarg);
          break;
        case 'h':
          usage(argv[0]);
          exit(EXIT_FAILURE);
      }
    }

  check_port_name();
  locate_port();
  open_port();
  setup_port();

  //obs_setup;
  sbp_setup();

  while(1) {
	  process_sbp();


    //rtk.sol.rr[0] = pos_llh.lat;
    //rtk.sol.rr[1] = pos_llh.lon;
    //rtk.sol.rr[2] = pos_llh.height;
    //rtkpos(&rtk, &obds, 3, &nav);
    //obsassign(&obs);
    //rtkpos(&rtk, &obs.data[0], obs.n, &nav);
    sleep(1);
  }

  close_port();
  free(serial_port_name);

  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
int showmsg(char *fmt,...)
{
va_list args;
va_start(args, fmt);

while (*fmt != '\0') {
if (*fmt == 'd') {
int i = va_arg(args, int);
printf("%d\n", i);
} else if (*fmt == 'c') {
// note automatic conversion to integral type
int c = va_arg(args, int);
printf("%c\n", c);
} else if (*fmt == 'f') {
double d = va_arg(args, double);
printf("%f\n", d);
}
++fmt;
}

va_end(args);

return 0;
}

void settspan(gtime_t ts, gtime_t te)
{

}

void settime(gtime_t time)
{

}
#ifdef __cplusplus
}
#endif



