#ifndef _COMS_H_
#define _COMS_H_

#include <stdint.h>

/**
 * Message received from the Rasberry Pi over UART
 * 
 * Message byte order:
 * 8 bytes: Motor 1 Speed
 * 8 bytes: Motor 2 Speed
 * 8 bytes: Motor 3 Speed
 * 8 bytes: Motor 4 Speed
 * 1 bit: Reset left encoder
 * 1 bit: Reset right encoder
 * 6 bits: Unused
 * 
 * Total: 5 bytes
*/
typedef struct
{
    uint8_t m1_setpt;
    uint8_t m2_setpt;
    uint8_t m3_setpt;
    uint8_t m4_setpt;
    
    int reset_left_enc :1;
    int reset_right_enc :1;
    int :6;

} recv_msg_t;

/**
 * Message sent to Raspberry Pi over UART
 * 
 * Message byte order:
 * 4 bytes: left encoder counter
 * 4 bytes: right encoder counter
 * 
 * Total: 8 bytes
*/
typedef struct
{
    int32_t left_enc_count;
    int32_t right_enc_count;
} send_msg_t;

void coms_recv_data(recv_msg_t *msg);

void coms_send_data(send_msg_t *msg);



#endif