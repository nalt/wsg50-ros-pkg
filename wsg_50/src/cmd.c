//======================================================================
/**
 *  @file
 *  cmd.c
 *
 *  @section cmd.c_general General file information
 *
 *  @brief
 *  Command abstraction layer
 *
 *  @author wolfer
 *  @date	20.07.2011
 *  
 *  
 *  @section cmd.c_copyright Copyright
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
#include <assert.h>

#include "wsg_50/common.h"
#include "wsg_50/msg.h"
#include "wsg_50/cmd.h"

#include "wsg_50/tcp.h"
#include "wsg_50/udp.h"
#include "wsg_50/serial.h"


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

static bool connected = false;


//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------


/**
 * Send command and wait for answer
 *
 * @param id		Command ID
 * @param len		Payload length
 * @param *payload	Payload data
 * @param pending	Flag indicating whether CMD_PENDING
 * 					is allowed return status
 *
 * @return Number of bytes received. -1 on error.
 */

int cmd_submit( unsigned char id, unsigned char *payload, unsigned int len,
			    bool pending, unsigned char **response, unsigned int *response_len )
{
	int res;
	status_t status;

	// Assemble message struct
	msg_t msg =
	{
		.id = id,
		.len = len,
		.data = payload
	};

	// Check if we're connected
	if ( !connected )
	{
		fprintf( stderr, "Interface not connected\n" );
		return -1;
	}

	// Send command
	res = msg_send( &msg );
	if ( res < 0 )
	{
		fprintf( stderr, "Message send failed\n" );
		return -1;
	}

	// Reuse message struct to receive response
	memset( &msg, 0, sizeof( msg ) );

	// Receive response. Repeat if pending.
	do
	{
		// Free response
		msg_free( &msg );

		// Receive response data
		res = msg_receive( &msg );
		if ( res < 0 )
		{
			fprintf( stderr, "Message receive failed\n" );
			return -1;
		}

		// Check response ID
		if ( msg.id != id )
		{
			fprintf( stderr, "Response ID (%2x) does not match submitted command ID (%2x)\n", msg.id, id );
			return -1;
		}

		if ( pending )
		{
			if ( msg.len < 2 )
			{
				fprintf( stderr, "No status code received\n" );
				return -1;
			}

			status = (status_t) make_short( msg.data[0], msg.data[1] );
		}
	}
	while( pending && status == E_CMD_PENDING );


	// Return payload
	*response_len = msg.len;
	if ( msg.len > 0 ) *response = msg.data;
	else *response = 0;

	return (int) msg.len;
}


/**
 * Open TCP connection
 *
 * @param *addr				String containing IP address
 * @param port				Port number (remote)
 *
 * @return 0 on success, else -1
 */

int cmd_connect_tcp( const char *addr, unsigned short port )
{
	int res;
	tcp_params_t params;
	const interface_t *iface;

	// IP address string must be given
	if ( !addr ) return -1;

	// If already connected, return error
	if ( connected ) return -1;

	// Get interface with the given name
	iface = interface_get( "tcp" );
	if ( !iface ) return -1;

	// Create parameter struct
	params.addr = str_to_ipaddr( addr );
	params.port = port;

	// Open connection
	res = msg_open( iface, &params );
	if ( res < 0 ) return -1;

	// Set connected flag
	connected = true;

	//printf( "TCP connection established. \n" );

	return 0;
}


/**
 * Open up UDP connection
 *
 * @param local_port		Local port number (for answer)
 * @param *addr				String containing IP address
 * @param remote_port		Remote port number
 *
 * @return 0 on success, else -1
 */

int cmd_connect_udp( unsigned short local_port, const char *addr, unsigned short remote_port )
{
	int res;
	udp_params_t params;
	const interface_t *iface;

	// IP address string must be given
	if ( !addr ) return -1;

	// If already connected, return error
	if ( connected ) return -1;

	// Get interface with the given name
	iface = interface_get( "udp" );
	if ( !iface ) return -1;

	// Create parameter struct
	params.addr = str_to_ipaddr( addr );
	params.local_port = local_port;
	params.remote_port = remote_port;

	// Open connection
	res = msg_open( iface, &params );
	if ( res < 0 ) return -1;

	// Set connected flag
	connected = true;

	printf( "UDP connection established\n" );

	return 0;
}


/**
 * Open up serial connection
 *
 * @param *device		Device string
 *
 * @return 0 on success, else -1
 */

int cmd_connect_serial( const char *device, unsigned int bitrate )
{
	int res;
	ser_params_t params;
	const interface_t *iface;

	// Device parameter must be given
	if ( !device || bitrate == 0 ) return -1;

	// Set connection parameters
	params.device = device;
	params.bitrate = bitrate;

	// If already connected, return error
	if ( connected ) return -1;

	// Get interface with the given name
	iface = interface_get( "serial" );
	if ( !iface ) return -1;

	// Open connection
	res = msg_open( iface, (void *) &params );
	if ( res < 0 ) return -1;

	// Set connected flag
	connected = true;

	printf( "Serial connection established\n" );

	return 0;
}


/**
 * Disconnect
 */

void cmd_disconnect( void )
{
	status_t status;
	int res;
	unsigned char *resp;
	unsigned int resp_len;

	printf( "Closing connection\n" );

	res = cmd_submit( 0x07, NULL, 0, false, &resp, &resp_len );
	if ( res != 2 ) printf( "Disconnect announcement failed: Response payload length doesn't match (is %d, expected 2)\n", res );
	else
	{
		// Check response status
		status = cmd_get_response_status( resp );
		if ( status != E_SUCCESS ) printf( "Command ANNOUNCE DISCONNECT not successful: %s\n", status_to_str( status ) );
	}

	if ( res > 0 ) free( resp );

	msg_close();
}


/**
 * Get connection state
 *
 * @return true if the command interface is connected, else false
 */

bool cmd_is_connected( void )
{
	return connected;
}


/**
 * Get status code from response data
 *
 * @return Status code
 */

status_t cmd_get_response_status( unsigned char *response )
{
	status_t status;

	assert( response != NULL );

	status = (status_t) make_short( response[0], response[1] );

	return status;
}

//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
