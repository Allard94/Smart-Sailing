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

#include <libserialport.h>
#include "rtklib/rtklib.h"
extern "C"
{
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/acquisition.h>
#include <libsbp/navigation.h>
#include <libsbp/piksi.h>
}


using namespace std;

sbp_state_t sbp_state;
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

msg_pos_llh_t      pos_llh;
msg_baseline_ned_t baseline_ned;
msg_vel_ned_t      vel_ned;
msg_dops_t         dopss;
msg_gps_time_t     gps_time;
msg_device_monitor_t device_monitor;

sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t device_monitor_node;
static sbp_msg_callbacks_node_t heartbeat_callback_node;


void usage(char *prog_name) {
  fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}



void setup_port()
{
  int result;

  result = sp_set_baudrate(piksi_port, 1000000);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set port baud rate!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set flow control!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set data bits!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set parity!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set stop bits!\n");
    exit(EXIT_FAILURE);
  }
}

void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  baseline_ned = *(msg_baseline_ned_t *)msg;
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  dopss = *(msg_dops_t *)msg;
}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void sbp_device_monitor_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	device_monitor = *(msg_device_monitor_t *)msg;
}

void sbp_setup(void)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
                        NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback,
                        NULL, &baseline_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,
                        NULL, &dops_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DEVICE_MONITOR, &sbp_device_monitor_callback, NULL, &device_monitor_node);
}

u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}


static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;
    }
}

int main(int argc, char **argv)
{

  rtk_t rtk;
  prcopt_t copt = prcopt_default;
  obsd_t obds;
  obs_t obs;
  nav_t nav;
  sta_t sta;
  rnxctr_t rnx;
  copt.mode = 6;
  copt.tropopt = 3;
  copt.dynamics = 1;
  copt.sateph = 1;
  cout << MAXSAT << endl;
  rtkinit(&rtk, &copt);
  init_rnxctr(&rnx);
  char oopt;
  const char *rinexfile = "Local-20140406-035808.obs";
  const char *sp3file = "igu19161_00.sp3";
  //readrnx(rinexfile, 1, &oopt, &rnx.obs, &rnx.nav, &rnx.sta);
  readsp3(sp3file, &nav, 1);
  //rtk.x[0] = 2;
  int opt;
  int result = 0;
  FILE * pFile;
  pFile = fopen ("myfile.txt","w");
  //pppoutsolstat(&rtk, 3, pFile);
  //sbp_state_t s;
  fclose(pFile);

  rtk.sol.rr[0] = 43;
  rtk.sol.rr[1] = 42;
  rtk.sol.rr[2] = 41;
  rtkpos(&rtk, &obds, 3, &nav);
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

  if (!serial_port_name) {
    fprintf(stderr, "Please supply the serial port path where the Piksi is " \
                    "connected!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_get_port_by_name(serial_port_name, &piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot find provided serial port!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_open(piksi_port, SP_MODE_READ);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
    exit(EXIT_FAILURE);
  }

  setup_port();

  sbp_setup();

  //sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
  //                        &heartbeat_callback_node);



  while(1) {
    sbp_process(&sbp_state, &piksi_port_read);
    //rtk.sol.rr[0] = pos_llh.lat;
    //rtk.sol.rr[1] = pos_llh.lon;
    //rtk.sol.rr[2] = pos_llh.height;
    //rtkpos(&rtk, &obds, 3, &nav);

    cout << rtk.sol.rr[0] <<endl;
//    cout << "GPS TIME:" << endl;
//    cout << (float)gps_time.tow/1e3 << endl;
//    cout << "Absolute Position:" << endl;
//    cout << "Latitude: " << pos_llh.lat << endl;
//    cout << "longtitude: " << pos_llh.lon << endl;
//    cout << "Height: " << pos_llh.height << endl;
//    cout << "Temperature: " << device_monitor.cpu_temperature << endl;
//    cout << "Satellites: " << unsigned(pos_llh.n_sats) << endl;
sleep(0.05);
  }

  result = sp_close(piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
  }

  sp_free_port(piksi_port);

  free(serial_port_name);

  return 0;
}

extern "C" {
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
}



