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

    nav_init(&nav);
    obs_init(&obs);
    rtkinit(&rtk, &rtk.opt);

    const char *sp3file = "igu19165_06.sp3";
    readsp3(sp3file, &nav, 1);

    double prev_tor = 0.0;

    /* Keeps running until user quits program. */
    while(1) {
    	s8 ret = sbp_process(&sbp_state, &piksi_port_read);
	    if (ret < 0){
	    	printf("sbp_process error: %d\n", (int)ret);
	    }
	    /* Checks if previous epoch is not the same as current epoch */
	    if(prev_tor!=tor.tow){
	    	rtkpos(&rtk, &obs.data[0], 9, &nav);
	    	obs_assign(&obs);
	    	printmsg();
	        prev_tor = tor.tow;
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



