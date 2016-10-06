/*
 * port.h
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/sbp.h>
#ifdef __cplusplus
}
#endif

extern char *serial_port_name;


u32 piksi_port_read(u8 *buff, u32 n, void *context);

void check_port_name();
void locate_port();
void open_port();
void setup_port();
void close_port();
void free_port();

#endif /* PORT_H_ */
