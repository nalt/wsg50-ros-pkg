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
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the and Weiss Robotics GmbH nor the names of its 
 *       contributors may be used to endorse or promote products derived from
 *	 this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
//======================================================================


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <signal.h>

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
	.write = &tcp_write,
	.data_available = &tcp_data_available
};

static tcp_conn_t conn;
static struct pollfd tcp_poll_set[1];
static unsigned char peak_read[1];

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

    tcp_poll_set[0].fd = conn.sock;
    tcp_poll_set[0].events = POLLIN | POLLPRI | POLLRDHUP | POLLHUP | POLLRDNORM | POLLERR;

    return 0;
}


/**
 * Close TCP socket
 *
 * @return 0
 */

void tcp_close( void )
{
	tcp_poll_set[0].fd = NULL;
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
	if (res == 0) {
		printf("Socket closed unexpectedly.\n");
		close( conn.sock );
		tcp_poll_set[0].fd = NULL;
		return -1;
	}

	if ( res < 0 )
	{
		close( conn.sock );
		tcp_poll_set[0].fd = NULL;
		printf( "Failed to read data from TCP socket\n" );
		return -1;
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

/**
 * Check if the socket has data to read
 * @return 1 if data is available, 0 if no data is available, -1 on error
 */

int tcp_data_available(unsigned int timeout)
{
	int res = poll(tcp_poll_set, 1, timeout);
	if (res > 0) {
		if( (tcp_poll_set[0].fd != NULL) && (tcp_poll_set[0].revents & POLLIN) ) {
			if (recv(conn.sock, peak_read, sizeof(peak_read), MSG_PEEK | MSG_DONTWAIT) == 0) {
				// if recv returns zero, that means the connection has been closed
				tcp_close();
				return -1;
			}
			return 1;
		} else {
			printf("-- -1 \n");
			return -1;
		}
	} else if (res == 0){
		return 0;
	}

	return -1;
}

//------------------------------------------------------------------------
// Test implementation
//------------------------------------------------------------------------
