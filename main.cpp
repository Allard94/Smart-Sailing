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
#include <fstream>

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

#include <libswiftnav/coord_system.h>
#include <libswiftnav/time.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
}

#include "port.h"
#include "rtklib.h"
#include "sbp.h"

using namespace std;

/* Prints out how to use the program.
 * Use -p 'serial port'.
 */
void usage(char *prog_name) {
  fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}

int main(int argc, char **argv)
{
	/* TODO: Make a nav iono test function and regular function */
	/* TODO: Record test data*/
	/* TODO: Calculate doppler. see libswiftnav. */

	//init_rnxctr(&rnx);
	//const char *rinexfile = "navigationdata.txt";
	//readrnx(rinexfile, 0, " " , &obs, &nav, NULL);
	/* PPP with test data */
	ofstream fs;
	fs.open("testBultPPP.csv", ofstream::out);
	fs << "test9\n";
	fs.close();
//	traceopen("test.trace");
//	tracelevel(5);
//	readcsv();
//
//	int ppp;
//	const char *sp3file1 = "igu19211_06.sp3";
//	const char *tecfile = "jprg3050.16i";
//	nav_init(&nav);
//	readsp3(sp3file1, &nav, 1);
//	readtec(tecfile, &nav, 1);
//	rtk_opt_init(&rtk);
//
//	for(int i = 0; i < ecef_pos.size(); i++){
//		printf("i: %d\n",i);
//		cout << "test1" << endl;
//		double r[3];
//		double posrad[3];
//		double posdeg[3];
//		rtkinit(&rtk, &rtk.opt);
//		nav_init(&nav);
//		obs_init(&obs);
//		nav_eph_test_assign(&nav, &obs, i);
//		uniqnav(&nav);
//		obs_test_assign(&obs, i);
//		cout << "test" << endl;
//		rtk_sol_test_assign(&rtk, i);
//		cout << "Old position LLH: " << endl;
//		cout << llh_pos[i][0] << endl;
//		cout << llh_pos[i][1] << endl;
//		cout << llh_pos[i][2] << endl;
//		cout << "Old position ECEF; " << endl;
//		cout << ecef_pos[i][0] << endl;
//		cout << ecef_pos[i][1] << endl;
//		cout << ecef_pos[i][2] << endl;
//		cout << "test3" << endl;
//
//		cout << "test4" << endl;
//		ppp = rtkpos(&rtk, obs.data, observations[i].size(), &nav);
//		printf("ppp: %d\n", ppp);
//		for(int j = 0; j < 3; j++){
//			r[j] = rtk.sol.rr[j];
//		}
//		if(ppp){
//			ecef2pos(r, posrad);
//			llhrad2deg(posrad, posdeg);
//			cout << "New position LLH: " << endl;
//			cout << posdeg[0] << endl;
//			cout << posdeg[1] << endl;
//			cout << posdeg[2] * (180 / M_PI) << endl;
//			cout << "New position ECEF: " << endl;
//			cout << rtk.sol.rr[0] << endl;
//			cout << rtk.sol.rr[1] << endl;
//			cout << rtk.sol.rr[2] << endl;
//			cout << "i " << i << endl;
//		}
//		else{
//			printf("Not able to calculate PPP.\nPlease wait for the system to process the data correctly.\n ");
//		}
//
//		/* TODO: Write coordinates to csv so we're able to plot the data in excel */
//	}
//	traceclose();
	int opt;
	/* Checks if user put in 2 arguments. */
    if (argc <= 1) {
    	usage(argv[0]);
    	exit(EXIT_FAILURE);
    }
    /* sets serial port name. */
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

    sbp_setup();



    const char *sp3file = "igu19251_06.sp3";
    nav_init(&nav);
    readsp3(sp3file, &nav, 1);
    rtk_opt_init(&rtk);
    double prev_tor = 0.0;
    bool found = false;
    /* Keeps running until user quits program. */
    while(1) {
    	s8 ret = sbp_process(&sbp_state, &piksi_port_read);
	    if (ret < 0){
	    	printf("sbp_process error: %d\n", (int)ret);
	    	continue;
	    }
	    if(!validNrSats() && !found){
	    	cout << "Not enough ephemeris data." << endl;
	    	continue;
	    }
	    if(pos_llh.n_sats < 4){
	    	if(!found){
	    		cout << "only " << (int)pos_llh.n_sats << " satellites, please wait for piksi to find more satellites..." << endl;
	    		found = true;
	    	}
	    	cout << "\r" << "searching..." << flush;
	    	continue;
	    }


	    /* Checks if previous epoch is not the same as current epoch */
	    if(prev_tor!=tor.tow){
//	    	nav_init(&nav);
//	    	found = true;
//	    	cout << "test2" << endl;
//
//	    	cout << "test3" << endl;
//	    	rtkinit(&rtk, &rtk.opt);
//	    	cout << "test4" << endl;
//
//
//
//	    	obs_init(&obs);
//
//	    	obs_assign(&obs);
//	    	rtk_sol_assign(&rtk);
//	    	nav_eph_assign(&nav);
//	    	nav_iono_assign(&nav);
//	    	uniqnav(&nav);
//	    	cout << "test" << endl;
//
//
//
//	    	rtkpos(&rtk, &obs.data[0], pos_llh.n_sats, &nav);
//	    	cout << "test1" << endl;
	    	prev_tor = tor.tow;
	        writecsv();
//	    	double r[3];
//	    	double posrad[3];
//	    	double posdeg[3];
//	    	for(int i = 0; i < 3; i++){
//	    		r[i] = rtk.sol.rr[i];
//	    	}
//	    	wgsecef2llh(r, posrad);
//	    	llhrad2deg(posrad, posdeg);
//
//	    	/* write SP and PPP to csv file */
//	    	ofstream fs;
//	    	fs.open("stilldata1_sp_pp.csv", ofstream::out | ofstream::app);
//	    	fs << pos_llh.lat << "," << pos_llh.lon << "," << pos_llh.height << "," << posdeg[0]<< "," << posdeg[1] << "," << posdeg[2] << "," << baseline_ecef.x << "," << baseline_ecef.y << "," << baseline_ecef.z << "," << tor.tow <<"\n";
//	    	fs.close();
//
//	    	//ephemerisVector();
//	    	cout << "|--------------------------Old Position----------------------------|--------------------------New Position----------------------------|" << endl;
//	    	cout.precision(10);
//	    	cout << fixed;
//	    	cout << "|    latitude: " << pos_llh.lat << "    ";
//	    	cout << "longtitude: " << pos_llh.lon << "    ";
//	    	cout << "height: " << pos_llh.height << "    |    ";
//	    	cout << "latitude: " << posdeg[0] << "    ";
//	    	cout << "longtitude: " << posdeg[1] << "    ";
//	    	cout << "height: " << posdeg[2] << "    |" << endl;
	    }

    sleep(0.5);
    }
    close_port();
    free(serial_port_name);

    return 0;
}

/* Program needs this to run. */
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



