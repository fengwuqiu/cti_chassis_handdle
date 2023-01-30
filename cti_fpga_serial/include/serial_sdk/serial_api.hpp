#ifndef SERIAL_API_H
#define SERIAL_API_H

#include "serial_sdk/serial_cmd.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <serial_sdk/crc16.hpp>
#include "serial_sdk/serial_api_al_linux.hpp"
#include "serial_sdk/udp_api_al_linux.hpp"
#include <unistd.h>
/*----------------------------------------------*
 * macros & types                               *
 *----------------------------------------------*/
/*debug switch*/
#define DEBUG_PRINTF(format, args...)   //printf("%s %s():"format, __FILE__, __FUNCTION__, ##args)
#define DEBUG_PRINTF_L2(format, args...)  // printf("%s %s():"format, __FILE__, __FUNCTION__, ##args)

#define DEBUG_PRINTF2(format, args...)  //printf(format, ##args)
#define DEBUG_PRINTF2_L2(format, args...)  //printf(format, ##args)

//#define DEBUG_PRINTF(format, ...)  printf(format, ##__VA_ARGS__)

/*recv ring buffer size*/
#define RECEIVE_BUFFER_SIZE             2048

#define MAX_RECV_FRAME_QUEUE            16 
#define MAX_SEND_FRAME_QUEUE            8  //min 3 for resend no warning

#define SERIAL_FRAME_SIZE               sizeof(serial_frame_type)

/*serial receive ring buffer type*/
typedef struct
{
    int start;
    int cnt;
    int state;
    unsigned char buf[RECEIVE_BUFFER_SIZE];
}serial_recv_buf_type;


typedef struct
{
    int cnt;
    unsigned char buf[SERIAL_FRAME_SIZE];
}serial_send_buf_type;


/*serial received frame queue type*/
typedef struct
{
    unsigned char head;
    unsigned char cnt;
    serial_frame_type frame[MAX_RECV_FRAME_QUEUE];
}recv_frame_queue_type;


/*serial send frame queue type*/
typedef struct
{
    unsigned char state;
    unsigned char head;
    unsigned char cnt;
    unsigned short resend[MAX_SEND_FRAME_QUEUE];
    serial_send_buf_type frame[MAX_SEND_FRAME_QUEUE];
}send_frame_queue_type;



extern serial_frame_type serial_frame;
extern recv_frame_queue_type recv_frame_queue;

extern uint16_t recv_pthread_cnt;
extern uint16_t recv_pthread_crc_status;

//for serial and udp
extern int module_serial_fd;
//extern int udp_dest_fd;
extern const unsigned char serial_frame_header[];
extern void init_frame_queue(void);
extern void init_recv_buffer(void);
extern void clear_resend_param(unsigned char cmd);
extern int process_cmd(serial_frame_type *frame);
extern int (*process_nomal_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length);
extern int (*process_update_cmd_cb)(unsigned char cmd, unsigned char *data, unsigned int length);
extern int pack_send_frame_buf_in_queue(serial_frame_type *frame, unsigned short resend);
extern void module_serial_process(void);
extern void process_received_serial_frames(void);
extern int construct_serial_frame_ex(serial_frame_type * frame, unsigned char cmd, unsigned int length, void * data);
//for serial
//open and close serial port and init queue
extern int open_serial_port(void);
extern void close_serial_port(int fd);
extern void module_serial_init(void);
//find frame in data from serial port
extern void find_serial_frame(int fd);
//send single normal or update frame to serial port
extern int send_single_serial_frame(serial_frame_type *frame, unsigned char need_setting);
extern int send_update_serial_frame(const unsigned char *data, unsigned int len);
//send data to serial port if necessary 
extern void send_serial_frames_if_necessary(void);

//for udp
//openor close udp port && init queue
extern int  open_udp_port(void);
extern void close_udp_port(int fd);
extern void module_udp_init(void);
//find frame in data from udp port
extern void find_udp_frame(int fd);
//send single normal or update frame to udp port
extern int send_single_udp_frame(serial_frame_type *frame, unsigned char need_setting);
extern int send_update_udp_frame(const unsigned char *data, unsigned int len);
//send data to udp port if necessary 
extern void send_udp_frames_if_necessary(void);


extern int recv_data_cnt;
extern int recv_ret;
extern int recv_crc_status;

#endif

