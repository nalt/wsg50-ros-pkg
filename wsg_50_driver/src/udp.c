//======================================================================
/**
 *  @file
 *  udp.c
 *
 *  @section udp.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section udp.c_copyright Copyright
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wsg_50/interface.h"
#include "wsg_50/udp.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

const interface_t udp =
{
	.name = "udp",
	.open = &udp_open,
	.close = &udp_close,
	.read = &udp_read,
	.write = &udp_write
};

static udp_conn_t conn;


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

/**
 * Open UDP socket
 *
 * @param *params		Connection parameters
 *
 * @return 0 on success, else -1
 */

int udp_open( const void *params )
{
	udp_params_t *udp = (udp_params_t *) params;

	conn.server = udp->addr;

	conn.rcv_bufptr = 0;
	conn.rcv_bufsize = UDP_RCV_BUFSIZE;
	
	conn.sock = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );
	if( conn.sock < 0 )
	{
		fprintf( stderr, "Cannot open UDP socket\n" );
		return -1;
	}

    memset( (char *) &conn.si_server, 0, sizeof(conn.si_server) );
    conn.si_server.sin_family = AF_INET;
    conn.si_server.sin_port = htons( udp->remote_port );
    conn.si_server.sin_addr.s_addr = udp->addr;

    conn.si_listen.sin_family = AF_INET;
    conn.si_listen.sin_addr.s_addr = htonl( INADDR_ANY );
    conn.si_listen.sin_port = htons( udp->local_port );

	unsigned int val = UDP_RCV_BUFSIZE;
    setsockopt( conn.sock, SOL_SOCKET, SO_RCVBUF, (void *) &val, (socklen_t) sizeof( val ) );

    struct timeval timeout = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt( conn.sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout, (socklen_t) sizeof( struct timeval ) );

    if ( bind( conn.sock, (struct sockaddr *) &conn.si_listen, sizeof(conn.si_listen) ) < 0 )
    {
    	fprintf( stderr, "Cannot bind port %d\n", udp->local_port );
    	return -1;
    }

    return 0;
}


/**
 * Close UDP socket
 *
 * @return 0
 */

void udp_close( void )
{
	close( conn.sock );
	conn.sock = 0;
}


/**
 * Read character from UDP socket
 *
 * Important note:
 * UDP works with datagrams and not with streams.
 * This means that recvfrom() gets the whole datagram
 * and dumps any data that exceeds the desired length!
 * This is why we check the size of an incoming datagram
 * before doing recvfrom() and read it out as a whole
 * into some buffer that is big enough to hold even large
 * datagrams.
 * If none-requested data is left in the buffer, subsequent
 * read calls will take the data from the buffer until it's
 * empty rather than getting new data from the net.
 *
 * @param *buf		Pointer to input buffer
 * @param len		Number of bytes that should be read
 *
 * @return Number of characters read
 */

int udp_read( unsigned char *buf, unsigned int len )
{
	fd_set readfds;
	unsigned int bytes_left;
    int res,
       	incoming,
    	packsize,
    	slen = sizeof( conn.si_incoming );

    if ( conn.sock <= 0 || buf == NULL )
    {
    	fprintf( stderr, "Parameter error (sock=%d, buf=%p)\n", conn.sock, buf );
    	return -1;
    }

    if ( len == 0 ) return 0;

    if ( conn.rcv_bufptr == 0 )
    {
    	conn.rcv_bufsize = UDP_RCV_BUFSIZE;

    	// Wait for packet to arrive
    	FD_ZERO( &readfds );
    	FD_SET( conn.sock, &readfds );
        struct timeval timeout;
        timeout.tv_sec = 0; timeout.tv_usec = 150000;
        res = select( conn.sock + 1, &readfds, NULL, NULL, &timeout);
        if ( res == 0 ) return -1; /* timeout */
        if ( res < 0 ) return -1;  /* error */

    	// Get size of packet pending
        res = ioctl( conn.sock, FIONREAD, &packsize );
        if ( res < 0 ) return -1;

        // Check if buffer is big enough to hold datagram
        if ( packsize > UDP_RCV_BUFSIZE )
        {
        	fprintf( stderr, "UDP buffer too small for incoming datagram" );
        	return -1;
        }

        // Read packet non-blocking
		incoming = recvfrom( conn.sock, conn.rcv_buf, packsize, MSG_DONTWAIT, (struct sockaddr *) &conn.si_incoming, (socklen_t *) &slen );
		if ( incoming < 0 )
		{
			fprintf( stderr, "recvfrom() returned error (%d)\n", incoming );
			return -1;
		}
		if ( conn.si_incoming.sin_addr.s_addr != conn.server )
		{
			fprintf( stderr, "Message from unknown server!\n" );
			return -1;
		}
		
		conn.rcv_bufsize = (unsigned int) incoming;
    }

    bytes_left = conn.rcv_bufsize - conn.rcv_bufptr;
    if ( len < bytes_left )
    {
    	memcpy( buf, &conn.rcv_buf[conn.rcv_bufptr], len );
    	conn.rcv_bufptr += len;
    	res = (int) len;
    }
    else
    {
    	memcpy( buf, &conn.rcv_buf[conn.rcv_bufptr], len );
    	conn.rcv_bufptr = 0;
    	res = (int) bytes_left;
    }

    return res;
}


/**
 * Write to UDP socket
 *
 * @param *buf		Pointer to buffer that holds data to be sent
 * @param len		Number of bytes to send
 *
 * @return 0 if successful, -1 on failure
 */

int udp_write( unsigned char *buf, unsigned int len )
{
    int res,
    	slen = sizeof( conn.si_incoming );

	if ( conn.sock <= 0 ) return( -1 );

	res = sendto( conn.sock, buf, len, 0, (struct sockaddr *) &conn.si_server, (socklen_t) slen );
    if ( res >= 0 ) return res;
    else return -1;
}
