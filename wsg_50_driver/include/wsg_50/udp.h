//======================================================================
/**
 *  @file
 *  udp.h
 *
 *  @section udp.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section udp.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef UDP_H_
#define UDP_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#ifdef WIN32
// Note: Compiler has to link against -lwsock32 or -lws2_32 on MinGW
// @todo Have to adjust some code to make udp work on MinGW
#include "winsock.h"
#include "winsock2.h"
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include "common.h"


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------

#define UDP_RCV_BUFSIZE		1024		// Size of UDP receive buffer. This is the maximum size a command message may have, including preamble etc.
										// Minimum is 8 (3 bytes preamble, 1 byte command id, 2 bytes size, 2 bytes checksum)


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	unsigned short local_port;
	ip_addr_t addr;
	unsigned short remote_port;
} udp_params_t;


typedef struct
{
	int sock;
	unsigned char rcv_buf[UDP_RCV_BUFSIZE];
	unsigned int rcv_bufptr;
	unsigned int rcv_bufsize;
	struct sockaddr_in si_listen;
	struct sockaddr_in si_server;
	struct sockaddr_in si_incoming;
	ip_addr_t server;
} udp_conn_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int udp_open( const void *params );
void udp_close( void );
int udp_read( unsigned char *buf, unsigned int len );
int udp_write( unsigned char *buf, unsigned int len );


#ifdef __cplusplus
}
#endif

#endif /* UDP_H_ */
