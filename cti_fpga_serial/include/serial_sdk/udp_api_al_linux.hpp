#ifndef UDP_API_AL_H
#define UDP_API_AL_H

#include "serial_cmd.hpp"

#define DEVICE_PORT_STR_LENGTH          30
extern char UDP_IP[];
extern int UDP_PORT;
extern struct sockaddr_in sockaddr;

extern char UDP_IP_DEST[];
extern int UDP_PORT_DEST;
extern struct sockaddr_in sockaddr_dest;

extern int open_udp_port(void);
//extern int open_udp_dest_port(void);
extern void colse_udp_port(int fd);
//extern void close_udp_dest_port(int fd);
extern int send_udp_flow(int fd, unsigned char *src, int cnt);
extern int recv_udp_flow(int fd, unsigned char *dest, int cnt);

#endif

