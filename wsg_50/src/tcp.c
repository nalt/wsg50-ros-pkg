//======================================================================
/**
 *  @file
 *  tcp.c
 *
 *  @section tcp.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	08.07.2011
 *  
 *  
 *  @section tcp.c_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wsg_50/interface.h"
#include "wsg_50/tcp.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------

#define TCP_RCV_TIMEOUT_SEC					60

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

const interface_t tcp =
{
	.name = "tcp",
	.open = &tcp_open,
	.close = &tcp_close,
	.read = &tcp_read,
	.write = &tcp_write
};

static tcp_conn_t conn;


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Unit Testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

/**
 * Open TCP socket
 *
 * @param *params		Connection parameters
 *
 * @return 0 on success, else -1
 */

int tcp_open( const void *params )
{
	int res;
	tcp_params_t *tcp = (tcp_params_t *) params;

	conn.server = tcp->addr;

	conn.sock = socket( PF_INET, SOCK_STREAM, IPPROTO_TCP );
	if( conn.sock < 0 )
	{
		fprintf( stderr, "Cannot open TCP socket\n" );
		return -1;
	}

    memset( (char *) &conn.si_server, 0, sizeof(conn.si_server) );
    conn.si_server.sin_family = AF_INET;
    conn.si_server.sin_port = htons( tcp->port );
    conn.si_server.sin_addr.s_addr = tcp->addr;

	unsigned int val = 1024;
    setsockopt( conn.sock, SOL_SOCKET, SO_RCVBUF, (void *) &val, (socklen_t) sizeof( val ) );

    struct timeval timeout = { .tv_sec = TCP_RCV_TIMEOUT_SEC, .tv_usec = 0 };
    setsockopt( conn.sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout, (socklen_t) sizeof( struct timeval ) );

    res = connect( conn.sock, (struct sockaddr *) &conn.si_server, sizeof(conn.si_server) );
    if ( res < 0 ) return -1;

    return 0;
}


/**
 * Close TCP socket
 *
 * @return 0
 */

void tcp_close( void )
{
	close( conn.sock );
	conn.sock = 0;
}


/**
 * Read character from TCP socket
 *
 * @return Character read
 */

int tcp_read( unsigned char *buf, unsigned int len )
{
    int res;

    if ( conn.sock <= 0 || buf == NULL ) return -1;
    if ( len == 0 ) return 0;

	// Read desired number of bytes
	res = recv( conn.sock, buf, len, 0 );
	if ( res < 0 )
	{
		close( conn.sock );
		quit( "Failed to read data from TCP socket\n" );
	}

    return res;
}


/**
 * Write to TCP socket
 *
 * @param *buf		Pointer to buffer that holds data to be sent
 * @param len		Number of bytes to send
 *
 * @return 0 if successful, -1 on failure
 */

int tcp_write( unsigned char *buf, unsigned int len )
{
    int res;

	if ( conn.sock <= 0 ) return( -1 );

	res = send( conn.sock, buf, len, 0 );
    if ( res >= 0 ) return( res );
    else
    {
    	fprintf( stderr, "Failed to send data using TCP socket\n" );
    	return -1;
    }
}


//------------------------------------------------------------------------
// Test implementation
//------------------------------------------------------------------------
