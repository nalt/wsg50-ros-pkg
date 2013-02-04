//======================================================================
/**
 *  @file
 *  serial.c
 *
 *  @section serial.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	08.07.2011
 *  
 *  
 *  @section serial.c_copyright Copyright
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

#ifdef WIN32
	// Note: Windows implementation is different
	#include "windows.h"
#else
	#include <fcntl.h>
	#include <termios.h>
	#include <errno.h>
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/stat.h>
#endif

#include "wsg_50/interface.h"
#include "wsg_50/serial.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

const interface_t serial =
{
	.name = "serial",
	.open = &serial_open,
	.close = &serial_close,
	.read = &serial_read,
	.write = &serial_write
};

#ifdef WIN32

#else

static ser_conn_t conn;

#endif


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------

static inline tcflag_t __bitrate_to_flag( unsigned int bitrate );


//------------------------------------------------------------------------
// Unit Testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

/**
 * Convert integer bitrate to flag
 *
 * @param bitrate		Bitrate
 *
 * @return Bitrate flag for serial driver
 */

static inline tcflag_t __bitrate_to_flag( unsigned int bitrate )
{
	switch( bitrate )
	{
		case   1200: return   B1200;
		case   2400: return   B2400;
		case   4800: return   B4800;
		case   9600: return   B9600;
		case  19200: return  B19200;
		case  38400: return  B38400;
		case  57600: return  B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		default: return 0;
	}
}


/**
 * Open serial device
 *
 * @param *device		Path to serial device, e.g. /dev/ttyS0
 *
 * @return 0 on success, else -1
 */

int serial_open( const void *params )
{
	ser_params_t *serial = (ser_params_t *) params;
    struct termios settings;
    tcflag_t bitrate;

    // Convert bitrate to flag
    bitrate = __bitrate_to_flag( serial->bitrate );
    if ( bitrate == 0 )
    {
		fprintf( stderr, "Invalid bitrate '%d' for serial device\n", serial->bitrate );
		return -1;
    }

    // Open serial device
	conn.fd = open( serial->device, O_RDWR | O_NOCTTY );
	if ( conn.fd < 0 )
	{
		fprintf( stderr, "Failed to open serial device '%s' (errno: %s)\n", serial->device, strerror(errno) );
		return -1;
	}

	// Check if device is a terminal device
    if ( !isatty( conn.fd ) )
    {
        fprintf( stderr, "Device '%s' is not a terminal device (errno: %s)!\n", serial->device, strerror(errno) );
        close( conn.fd );
        return -1;
    }

    // Set input flags
    settings.c_iflag =  IGNBRK          // Ignore BREAKS on Input
                     |  IGNPAR;         // No Parity
    									// ICRNL: map CR to NL (otherwise a CR input on the other computer will not terminate input)

    // Set output flags
    settings.c_oflag = 0;				// Raw output

    // Set controlflags
    settings.c_cflag = bitrate
    				 | CS8              // 8 bits per byte
    				 | CSTOPB			// Stop bit
                     | CREAD            // characters may be read
                     | CLOCAL;          // ignore modem state, local connection

    // Set local flags
    settings.c_lflag = 0;				// Other option: ICANON = enable canonical input

    // Set maximum wait time on input - cf. Linux Serial Programming HowTo, non-canonical mode
    // http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
    settings.c_cc[VTIME] = 10;			// 0 means timer is not uses

    // Set minimum bytes to read
    settings.c_cc[VMIN]  = 0;			// 1 means wait until at least 1 character is received

	// Now clean the modem line and activate the settings for the port
	tcflush( conn.fd, TCIFLUSH );
	tcsetattr( conn.fd, TCSANOW, &settings );

	return(0);
}


/**
 * Close serial device
 */

void serial_close( void )
{
	close( conn.fd );
}


/**
 * Read from serial device
 *
 * @param *buf		Pointer to receive buffer
 * @param len		Number of bytes wished to read
 *
 * @return Number of bytes read
 */

int serial_read( unsigned char *buf, unsigned int len )
{
	int res;

	res = read( conn.fd, buf, len );
	if ( res < 0 )
	{
		fprintf( stderr, "Failed to read from serial device\n" );
		exit(1);
	}

	return res;
}


/**
 * Write to serial device
 *
 * @param *buf		Pointer to buffer that holds data to be sent
 * @param len		Number of bytes to send
 *
 * @return Number of bytes written
 */

int serial_write( unsigned char *buf, unsigned int len )
{
	return( write( conn.fd, (void *) buf, len ) );
}


