/*
 * sbp.c
 *
 *  Created on: Oct 6, 2016
 *      Author: root
 */

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <libsbp/navigation.h>
#include <libsbp/sbp.h>
#ifdef __cplusplus
}
#endif

#include "port.h"

sbp_state_t sbp_state;

msg_pos_llh_t      		 pos_llh;
msg_vel_ned_t      		 vel_ned;

static sbp_msg_callbacks_node_t pos_llh_node;
static sbp_msg_callbacks_node_t vel_ned_node;

void printmsg(){
	printf("latitude: %d\n", pos_llh.lat);
	printf("velocity ned: %d\n", vel_ned.n);
}

void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}

void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}

void sbp_setup(void){
	sbp_state_init(&sbp_state);

    sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
	                      NULL, &pos_llh_node);
    sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                          NULL, &vel_ned_node);
}

void process_sbp(){
	s8 ret = sbp_process(&sbp_state, &piksi_port_read);
    if (ret < 0){
    	printf("sbp_process error: %d\n", (int)ret);
    }
    printmsg();
}




