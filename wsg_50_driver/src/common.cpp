//======================================================================
/**
 *  @file
 *  common.c
 *
 *  @section common.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	19.07.2011
 *  
 *  
 *  @section common.c_copyright Copyright
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


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


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
 * Convert IP address string to IP address type
 *
 * @param *str		String containing IP address
 *
 * @return IP address type
 */

ip_addr_t str_to_ipaddr( const char *str )
{
	unsigned int i, res;
	unsigned int buf[4];
	ip_addr_t addr = 0;
	res = sscanf( str, "%d.%d.%d.%d", &buf[3], &buf[2], &buf[1], &buf[0] );
	if ( res != 4 ) return( 0 );
	for ( i = 0; i < 4; i++ )
	{
		if ( buf[i] > 255 ) return( 0 );
		addr <<= 8;
		addr |= (int) buf[i];
	}
	return( addr );
}


/**
 * Convert status code to string
 *
 * @param status	Status code
 *
 * @return Status string
 */

const char * status_to_str( status_t status )
{
	switch( status )
	{
		case E_SUCCESS:					return( "No error" );
		case E_NOT_AVAILABLE:			return( "Service or data is not available" );
		case E_NO_SENSOR:				return( "No sensor connected" );
		case E_NOT_INITIALIZED:			return( "The device is not initialized" );
		case E_ALREADY_RUNNING:			return( "Service is already running" );
		case E_FEATURE_NOT_SUPPORTED:	return( "The requested feature is not supported" );
		case E_INCONSISTENT_DATA:		return( "One or more dependent parameters mismatch" );
		case E_TIMEOUT:					return( "Timeout error" );
		case E_READ_ERROR:				return( "Error while reading from a device" );
		case E_WRITE_ERROR:				return( "Error while writing to a device" );
		case E_INSUFFICIENT_RESOURCES:	return( "No memory available" );
		case E_CHECKSUM_ERROR:			return( "Checksum error" );
		case E_NO_PARAM_EXPECTED:		return( "No parameters expected" );
		case E_NOT_ENOUGH_PARAMS:		return( "Not enough parameters" );
		case E_CMD_UNKNOWN:				return( "Unknown command" );
		case E_CMD_FORMAT_ERROR:		return( "Command format error" );
		case E_ACCESS_DENIED:			return( "Access denied" );
		case E_ALREADY_OPEN:			return( "Interface already open" );
		case E_CMD_FAILED:				return( "Command failed" );
		case E_CMD_ABORTED:				return( "Command aborted" );
		case E_INVALID_HANDLE:			return( "Invalid handle" );
		case E_NOT_FOUND:				return( "Device not found" );
		case E_NOT_OPEN:				return( "Device not open" );
		case E_IO_ERROR:				return( "General I/O-Error" );
		case E_INVALID_PARAMETER:		return( "Invalid parameter" );
		case E_INDEX_OUT_OF_BOUNDS:		return( "Index out of bounds" );
		case E_CMD_PENDING:				return( "Command is pending..." );
		case E_OVERRUN:					return( "Data overrun" );
		case E_RANGE_ERROR:				return( "Value out of range" );
		case E_AXIS_BLOCKED:			return( "Axis is blocked" );
		case E_FILE_EXISTS:				return( "File already exists" );
		default:						return( "Internal error. Unknown error code." );
	}
}


/**
 * Quit program for the given reason
 *
 * @param *reason	String telling why we're quitting
 */

void quit( const char *reason )
{
	if ( reason ) fprintf( stderr, "%s\n", reason );
	exit(1);
}


const char * getStateValues( unsigned char *b ){

	/*
	unsigned char aux[4]; 

	
	aux[0] = b[0];
	aux[1] = b[1];
	aux[2] = b[2];
	aux[3] = b[3];
	
	dbgPrint("Dins de getStateValues.\n");
	dbgPrint("b[2] = 0x%x\n", b[2]);
	dbgPrint("b[3] = 0x%x\n", b[3]);
	dbgPrint("b[4] = 0x%x\n", b[4]);
	dbgPrint("b[5] = 0x%x\n", b[5]);
	*/

	char resp[1024] = "| ";

	if (b[2] & 0x1){	// D0 ==> LSB
		//dbgPrint("Fingers Referenced.\n");
		char aux0[21] = "Fingers Referenced |";
		strcat(resp, aux0);
	}
	if (b[2] & 0x2){  // D1
		//dbgPrint("The Fingers are currently moving.\n");
		char aux1[36]=" The Fingers are currently moving |";
		strcat(resp, aux1);
	}
	if (b[2] & 0x4){  // D2
		//dbgPrint("Axis is blocked in negative moving direction.\n");
		char aux2[48] =" Axis is blocked in negative moving direction |";
		strcat(resp, aux2);
	}
     	if (b[2] & 0x8){  // D3
		//dbgPrint("Axis is blocked in positive moving direction.\n");
		char aux3[48] =" Axis is blocked in positive moving direction |";
		strcat(resp, aux3);
	}
	if (b[2] & 0x10){ // D4
		//dbgPrint("Negative direction soft limit reached.\n");
		char aux4[42] = " Negative direction soft limit reached |";
		strcat(resp, aux4);
	}
	if (b[2] & 0x20){ // D5
		//dbgPrint("Positive direction soft limit reached.\n");
		char aux5[42] = " Positive direction soft limit reached |";
		strcat(resp, aux5);
	}
	if (b[2] & 0x40){ // D6
		//dbgPrint("Axis Stopped.\n");
		char aux6[18] = " Axis Stopped |";
		strcat(resp, aux6);
	}
	if (b[2] & 0x80){ // D7
		//dbgPrint("Target Pos reached.\n");
		char aux7[22] = " Target Pos reached |";
		strcat(resp, aux7);
	}

	if (b[3] & 0x1){ // D8
		//dbgPrint("Overdrive Mode.\n");
		char aux8[18] = " Overdrive Mode |";
		strcat(resp, aux8);
	}
	if (b[3] & 0x10){ // D12
		//dbgPrint("Fast Stop.\n");
		char aux12[13] = " Fast Stop |";
		strcat(resp, aux12);
	}
	if (b[3] & 0x20){ // D13
		//dbgPrint("Temperature Warning.\n");
		char aux13[23] = " Temperature Warning |";
		strcat(resp,aux13);
	}	
	if (b[3] & 0x40){ // D14
		//dbgPrint("Temperature Error.\n");
		char aux14[21]= " Temperature Error |";
		strcat(resp, aux14);
	}	
	if (b[3] & 0x80){ // D15
		//dbgPrint("Power Error.\n");
		char aux15[15]= " Power Error |";
		strcat(resp, aux15);
	}

	if (b[4] & 0x1){  // D16
		//dbgPrint("Engine Current Error.\n");
		char aux16[24]= " Engine Current Error |";
		strcat(resp, aux16);
	}
	if (b[4] & 0x2){  // D17
		//dbgPrint("Finger Fault.\n");
		char aux17[16] = " Finger Fault |";
		strcat(resp, aux17);
	}
	if (b[4] & 0x4){  // D18
		//dbgPrint("Command Error.\n");
		char aux18[17] = " Command Error |";
		strcat(resp, aux18);
	}
     	if (b[4] & 0x8){  // D19
		//dbgPrint("A script is currently running.\n");
		char aux19[33] = " A script is currently running |";
		strcat(resp, aux19);
	}
	if (b[4] & 0x10){ // D20
		//dbgPrint("Script Error.\n");
		char aux20[16] = " Script Error |";
		strcat(resp, aux20);
	}

	// [D21 - D31] RESERVED
	
	// D31 ==> MSB

	//dbgPrint("%s\n", resp);
	return resp;
}


//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
