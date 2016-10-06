/*
 * port.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#include <stdio.h>
#include <stdlib.h>

#include <libserialport.h>

#include "port.h"

char *serial_port_name = NULL;
int result = 0;
struct sp_port *piksi_port = NULL;


u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}

void check_port_name(){
	if (!serial_port_name) {
	    fprintf(stderr, "Please supply the serial port path where the Piksi is " \
	                    "connected!\n");
	    exit(EXIT_FAILURE);
	  }
}

void locate_port(){
	result = sp_get_port_by_name(serial_port_name, &piksi_port);
	  if (result != SP_OK) {
	    fprintf(stderr, "Cannot find provided serial port!\n");
	    exit(EXIT_FAILURE);
	  }
}

void open_port(){
	result = sp_open(piksi_port, SP_MODE_READ);
	  if (result != SP_OK) {
	    fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
	    exit(EXIT_FAILURE);
	  }
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

void close_port(){
	result = sp_close(piksi_port);
	  if (result != SP_OK) {
	    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
	  }
}

void free_port(){
	sp_free_port(piksi_port);
}