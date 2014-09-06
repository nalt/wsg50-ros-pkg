//======================================================================
/**
 *  @file
 *  msg.c
 *
 *  @section msg.c_general General file information
 *
 *  @brief
 *  Raw send and receive functions for command messages
 *
 *  @author wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section msg.c_copyright Copyright
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

#include "wsg_50/common.h"
#include "wsg_50/checksum.h"
#include "wsg_50/interface.h"
#include "wsg_50/msg.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

static const interface_t *interface;


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
 * Receive answer
 *
 * @param **response		Data buffer
 * @param len				Expected size of message
 *
 * @return Overall number of bytes received, including header and checksum. -1 on error.
 */

int msg_receive( msg_t *msg )
{
	int res;
	unsigned char header[3];			// 1 byte command, 2 bytes payload length
	unsigned short checksum = 0x50f5;	// Checksum over preamble (0xaa 0xaa 0xaa)
	unsigned int sync;

	// Syncing - necessary for compatibility with serial interface
	sync = 0;
	while( sync != MSG_PREAMBLE_LEN )
	{
		res = interface->read( header, 1 );
		if ( header[0] == MSG_PREAMBLE_BYTE ) sync++;
	}

	// Read header
	res = interface->read( header, 3 );
	if ( res < 3 )
	{
		fprintf( stderr, "Failed to receive header data (%d bytes read)\n", res );
		return -1;
	}

	// Calculate checksum over header
	checksum = checksum_update_crc16( header, 3, checksum );

	// Get message id of received
	msg->id = header[0];

	// Get payload size of received message
	msg->len = make_short( header[1], header[2] );

	// Allocate space for payload and checksum
	msg->data = malloc( msg->len + 2u );
	if ( !msg->data ) return -1;

	// Read payload and checksum
	res = interface->read( msg->data, msg->len + 2 );
	if ( res < (int) (msg->len + 2) )
	{
		fprintf( stderr, "Not enough data (%d, expected %d)\n", res, msg->len + 2 );
		return -1;
	}

	// Check checksum
	checksum = checksum_update_crc16( msg->data, msg->len + 2, checksum );
	if ( checksum != 0 )
	{
		fprintf( stderr, "Checksum error\n" );
		return -1;
	}

	return msg->len + 8;
}


/**
 * Send command
 *
 * @param id		Command ID
 * @param len		Payload length
 * @param *payload	Payload data
 *
 * @return 0 on success, else -1
 */

int msg_send( msg_t *msg )
{
	unsigned char header[MSG_PREAMBLE_LEN + 3];
    //unsigned char checksum[2];
	unsigned short crc;
	int i, res;

	// Preamble
	for ( i = 0; i < MSG_PREAMBLE_LEN; i++ ) header[i] = MSG_PREAMBLE_BYTE;

	// Command ID
	header[MSG_PREAMBLE_LEN] = msg->id;

	// Length
	header[MSG_PREAMBLE_LEN + 1] = lo( msg->len );
	header[MSG_PREAMBLE_LEN + 2] = hi( msg->len );

	// Checksum
	crc = checksum_crc16( header, 6 );
	crc = checksum_update_crc16( msg->data, msg->len, crc );

    //checksum[0] = lo( crc );
    //checksum[1] = hi( crc );

	if ( interface->write )
	{

		unsigned char *buf = malloc( 6 + msg->len + 2 ); // 6+2 fixes (PREAMBLE, ID, PAILOAD / ... / CRC)
		memcpy( buf, header, 6 );
		memcpy( buf + 6, msg->data, msg->len );
		memcpy( buf + 6 + msg->len, (unsigned char *) &crc, 2 );

		res = interface->write( buf, 6 + msg->len + 2 );
        if ( res < 6 + (int)msg->len + 2 )
		{
			interface->close();
			quit( "Failed to submit message checksum" );
		}

		free( buf );

// The following implementation doesn't work:
//
//		// Submit header
//		res = interface->write( header, 6 );
//		if ( res < 6 )
//		{
//			interface->close();
//			quit( "Failed to submit message header" );
//		}
//
//		// Submit payload
//		res = interface->write( msg->data, msg->len );
//		if ( res < msg->len )
//		{
//			interface->close();
//			quit( "Failed to submit message payload" );
//		}
//
//		res = interface->write( (unsigned char *) &crc, 2 );
//		if ( res < sizeof( crc ) )
//		{
//			interface->close();
//			quit( "Failed to submit message checksum" );
//		}

//		unsigned short check = checksum_crc16( header, 6 );
//		check = checksum_update_crc16( msg->data, msg->len, check );
//		check = checksum_update_crc16( (unsigned char *) &crc, sizeof( short ), check );

		return msg->len + 8;
	}

	return -1;
}


/**
 * Change command interface
 *
 * @param *iface		Pointer to interface struct
 * 						describing new interface
 *
 * @return 0 on success, else -1
 */

int msg_change_interface( const interface_t *iface )
{
	if ( !iface ) return -1;

	if ( interface && iface != interface && interface->close ) interface->close();

	interface = iface;

	return 0;
}


/**
 * Open command interface
 *
 * @param *iface
 * @param *params		Pointer referencing a struct that holds
 * 						parameters for the interface (e.g. address)
 *
 * @return 0 on success, else -1
 */

int msg_open( const interface_t *iface, const void *params )
{
	int res;

	res = msg_change_interface( iface );
	if ( res < 0 ) return( res );

	if ( iface->open ) return iface->open( params );
	else return -1;
}


/**
 * Close command interface
 */

void msg_close( void )
{
	if ( !interface || !interface->close ) return;
	interface->close();
}


/**
 * Free message struct
 *
 * @param *msg		Pointer to message struct
 */

void msg_free( msg_t *msg )
{
	if ( msg->data ) free( msg->data );
	memset( msg, 0, sizeof( msg ) );
}


//------------------------------------------------------------------------
// Test implementation
//------------------------------------------------------------------------
