//======================================================================
/**
 *  @file
 *  tcp.h
 *
 *  @section tcp.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	08.07.2011
 *  
 *  
 *  @section tcp.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef TCP_H_
#define TCP_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#ifdef WIN32
	// Note: Compiler has to link against -lwsock32 or -lws2_32 on MinGW
	// @todo Have to adjust some code to make tcp work on MinGW
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


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	ip_addr_t addr;
	unsigned short port;
} tcp_params_t;


typedef struct
{
	int sock;
	struct sockaddr_in si_server;
	ip_addr_t server;
} tcp_conn_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int tcp_open( const void *params );
void tcp_close( void );
int tcp_read( unsigned char *buf, unsigned int len );
int tcp_write( unsigned char *buf, unsigned int len );


#ifdef __cplusplus
}
#endif

#endif /* TCP_H_ */
