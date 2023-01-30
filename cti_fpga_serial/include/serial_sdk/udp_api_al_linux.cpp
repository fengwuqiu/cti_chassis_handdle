
/*linux header*/
#include <termios.h>     //struct termios
#include <stdio.h>       //delete file
#include <stdlib.h>      //perror printf
#include <unistd.h>  
#include <fcntl.h>       //open close read write
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/select.h>
#include <string.h>      //memset
/*common header*/
#include "udp_api_al_linux.hpp"
#include "serial_api.hpp"
#include "serial_cmd.hpp"
#include "crc16.hpp"
//linux header
#include <inttypes.h>    //or stdint.h uint16_t uint32_t
#include <sys/types.h> 
#include <unistd.h>  //file operation open read write lseek close
#include<fcntl.h> 
#include<linux/serial.h>
//for udp
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>

char UDP_IP[DEVICE_PORT_STR_LENGTH] = {0};
int UDP_PORT = 0;
struct sockaddr_in sockaddr = {0};
fd_set udp_fds;

char UDP_IP_DEST[DEVICE_PORT_STR_LENGTH] = {0};
int UDP_PORT_DEST = 0;
struct sockaddr_in sockaddr_dest = {0};

int set_socket(int fd, char* ip,int port){
    sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons(port);
	sockaddr.sin_addr.s_addr = inet_addr(ip);
    if(bind(fd,(struct sockaddr*)&sockaddr,sizeof(sockaddr)) < 0){
        std::cout<<"server bind error!"<<std::endl;
        return -1;
    }
    return 0;
}
int set_dest_socket(char* ip,int port){
	printf("set_dest_socket: ip<%s>,port<%d>\n",ip,port);
    sockaddr_dest.sin_family = AF_INET;
	sockaddr_dest.sin_port = htons(port);
	sockaddr_dest.sin_addr.s_addr = inet_addr(ip);
    return 0;
}


int do_open_udp(void)
{
    int fd1, ret;  

    fd1 = socket(AF_INET,SOCK_DGRAM,0);
    if (fd1 == -1)
    {
        std::cout<<"udp socket not ready"<<std::endl;
        return -1; 
    }
        std::cout<<"open udp socket success"<<std::endl;

    ret = set_socket(fd1,UDP_IP,UDP_PORT);
    set_dest_socket(UDP_IP_DEST,UDP_PORT_DEST);
    printf("sockaddr:<ip %s> <port %d> \n",inet_ntoa(sockaddr_dest.sin_addr),ntohs(sockaddr_dest.sin_port));
    if (ret == -1)
    {
        std::cout<<"set udp socket udp_ip: "<<UDP_IP<<" udp_port: "<<UDP_PORT<<" failed!"<<std::endl;
        return -1; 
    }
        std::cout<<"set udp socket udp_ip: "<<UDP_IP<<" udp_port: "<<UDP_PORT<<" successed!"<<std::endl;
    return fd1;
}

// int do_open_udp_dest(void){
//     int fd1,ret;
//     fd1 = socket(AF_INET,SOCK_DGRAM,0);
//     if (fd1 == -1)
//     {
//         std::cout<<"udp  dest socket not ready"<<std::endl;
//         return -1; 
//     }
//     std::cout<<"open udp  dest socket success"<<std::endl;
//     ret = set_socket(fd1,UDP_IP_DEST,UDP_PORT_DEST);
//     if (ret == -1)
//     {
//         std::cout<<"set udp dest socket udp_ip: "<<UDP_IP_DEST<<" udp_port: "<<UDP_PORT_DEST<<" failed!"<<std::endl;
//         return -1; 
//     }
//     std::cout<<"set udp dest socket udp_ip: "<<UDP_IP_DEST<<" udp_port: "<<UDP_PORT_DEST<<" successed!"<<std::endl;
//     return fd1;
// }

int open_udp_port(void)
{
    std::cout<<"open udp port start"<<std::endl;
    int fd1;
    if(module_serial_fd != -1)
    {
        return module_serial_fd;
    }
    while(-1 == (fd1 = do_open_udp()))
    {
        std::cout<<"open udp port failed, try again!"<<std::endl;
        usleep(1000000UL);
    }
    std::cout<<"open udp port successed! fd: "<<fd1<<std::endl;
    return fd1;
}

// int open_udp_dest_port(void)
// {
//     std::cout<<"open udp dest port start"<<std::endl;
//     int fd1;
//     if(udp_dest_fd != -1)
//     {
//         return udp_dest_fd;
//     }
//     while(-1 == (fd1 = do_open_udp_dest()))
//     {
//         std::cout<<"open udp dest port failed, try again!"<<std::endl;
//         usleep(1000000UL);
//     }
//     std::cout<<"open udp dest port successed! fd: "<<fd1<<std::endl;
//     return fd1;
// }


void close_udp_port(int fd)
{
    close(fd);
    module_serial_fd = 0;
}
// void colse_udp_dest_port(int fd)
// {
//     close(fd);
//     udp_dest_fd = 0;
// }


int send_udp_flow(int fd, unsigned char *src, int cnt)
{
    int ret = 0;
    int temp = cnt;
    while(temp)
    {
        ret = sendto(fd, src, temp, 0, (struct sockaddr *)&sockaddr_dest,sizeof(sockaddr));
        temp -= ret;
    //printf("send:<fd %d> <ip %s> <port %d>\n",fd,inet_ntoa(sockaddr_dest.sin_addr),ntohs(sockaddr_dest.sin_port));
    }
    //test
    DEBUG_PRINTF("send out buf size %d.\n", cnt);
    return cnt;
}

struct timeval udp_timeout={0,100};
int recv_udp_flow(int fd, unsigned char *dest, int cnt)
{
    int ret=-1;
    FD_ZERO(&udp_fds); 
    FD_SET(fd,&udp_fds); 
    switch(select(fd+1,&udp_fds,NULL,NULL,&udp_timeout)) 
    {
    case -1: 
        printf("serial read error!\n"); 
        break; 
    case 0:
        break; 
    default:
        if(FD_ISSET(fd,&udp_fds))
        {
            socklen_t addrlen = sizeof(sockaddr);
            ret = recvfrom(fd,dest,cnt,0,(struct sockaddr*)&sockaddr,&addrlen);
	        // printf("rev:<ip %s> <port %d>\n",inet_ntoa(sockaddr.sin_addr),ntohs(sockaddr.sin_port));
        }
        break;
    }
    return (int)ret;
}







