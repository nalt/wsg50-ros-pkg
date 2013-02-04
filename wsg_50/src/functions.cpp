//======================================================================
/**
 *  @file
 *  functions.c
 *
 *  @section functions.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author Marc
 *  @date   06.06.2012
 *  
 *  
 *  @section functions.c_copyright Copyright
 *  
 *  Copyright 2012 Robotnik Automation, SLL
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
#include "wsg_50/cmd.h"
#include "wsg_50/functions.h"

//------------------------------------------------------------------------
// Support functions
//------------------------------------------------------------------------

float convert(unsigned char *b){
	float tmp;
  	unsigned int src = 0;

	/*
	dbgPrint("b[3]=%x\n", b[3]);
	dbgPrint("b[2]=%x\n", b[2]);
	dbgPrint("b[1]=%x\n", b[1]);
	dbgPrint("b[0]=%x\n", b[0]);
	*/

  	src = b[3] * 16777216 + b[2] * 65536 + b[1] * 256 + b[0];

  	memcpy(&tmp, &src, sizeof tmp);
  	//printf("Converted value: %f \n", tmp);

	return tmp;
}


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Note: Argument values that are outside the gripper’s physical limits are clamped to the highest/lowest available value. //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////
// ACTUATION FUNCTIONS //
/////////////////////////


int homing( void )
{
	status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

	// Set flags: Homing direction (0: default, 1: widthitive movement, 2: negative movement).
	payload[0] = 0x00;

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x20, payload, 1, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	//dbgPrint("&resp: %s\n", resp);
	//dbgPrint("&resp_len: %d\n", resp_len);

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command HOMING not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


int move( float width, float speed )
{

	status_t status;
	int res;
	unsigned char payload[9];
	unsigned char *resp;
	unsigned int resp_len;

	// Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
	payload[0] = 0x02;

	// Copy target width and speed
	memcpy( &payload[1], &width, sizeof( float ) );
	memcpy( &payload[5], &speed, sizeof( float ) );

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x21, payload, 9, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command MOVE not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


int stop( void )
{
	status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

	//payload[0] = 0x00;

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x22, payload, 0, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command STOP not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


int ack_fault( void )
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

	payload[0] = 0x61;  //MBJ: Està ben enviat, si es posa alrevés no torna error en terminal però si que es posa roig el LED
	payload[1] = 0x63;
	payload[2] = 0x6B;

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x24, payload, 3, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}


	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command ACK not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


int grasp( float objWidth, float speed )
{
	status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy part width and speed
	memcpy( &payload[0], &objWidth, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x25, payload, 8, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GRASP not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return( 0 );
}


int release( float width, float speed )
{
	status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy part width and speed
	memcpy( &payload[0], &width, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x26, payload, 8, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return -1;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command RELEASE not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


///////////////////
// SET FUNCTIONS //
///////////////////


int setAcceleration( float acc )
{
	status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy target width and speed
	memcpy( &payload[0], &acc, sizeof( float ) );

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x30, payload, 4, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command SET ACCELERATION not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}

int setGraspingForceLimit( float force )
{
	status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy target width and speed
	memcpy( &payload[0], &force, sizeof( float ) );

	// Submit command and wait for response. Push result to stack.
	res = cmd_submit( 0x32, payload, 4, true, &resp, &resp_len );
	if ( res != 2 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	free( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command SET GRASPING FORCE LIMIT not successful: %s\n", status_to_str( status ) );
		return -1;
	}

	return 0;
}


///////////////////
// GET FUNCTIONS //
///////////////////


const char * systemState( void ) 
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 3 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x40, payload, 3, false, &resp, &resp_len );
	if ( res != 6 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 6)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );

	/*
	dbgPrint("LSB -> resp[0]: %x\n", resp[2]);
	dbgPrint("       resp[1]: %x\n", resp[3]);
	dbgPrint("       resp[2]: %x\n", resp[4]);
	dbgPrint("MSB -> resp[3]: %x\n", resp[5]);
	*/

	return getStateValues(resp);

	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET SYSTEM STATE not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}

	free( resp );

	//return (int) resp[2]; MBJ
}


int graspingState( void )
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 3 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x41, payload, 3, false, &resp, &resp_len );
	if ( res != 3 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET GRASPING STATE not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}

	free( resp );

	dbgPrint("GRASPING STATUS: %s\n", status_to_str (status) );

	return (int) resp[2];
}


int getOpening( void )  
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 3 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x43, payload, 3, false, &resp, &resp_len ); // 0x43
	if ( res != 6 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET OPENNING not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}

	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];

	//dbgPrint("OPENING WIDTH: %f mm\n", convert(vResult));

	free( resp );

	return convert(vResult);

	// return (int) resp[2];
}


int getForce( void )  
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 3 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x45, payload, 3, false, &resp, &resp_len ); // 0x43
	if ( res != 6 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET SPEED not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}

	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];

	free( resp );

	return convert(vResult);

	//return (int) resp[2];
}


int getAcceleration( void )  
{
	status_t status;
	int res;
	unsigned char payload[6];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 1 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x31, payload, 0, false, &resp, &resp_len );
	if ( res != 6 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET ACCELERATION not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}
	
	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];
	
	free( resp );

	return convert(vResult);

	//return (int) resp[2];
}

int getGraspingForceLimit( void )  
{
	status_t status;
	int res;
	unsigned char payload[6];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 1 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = cmd_submit( 0x33, payload, 0, false, &resp, &resp_len );
	if ( res != 6 )
	{
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = cmd_get_response_status( resp );
	if ( status != E_SUCCESS )
	{
		dbgPrint( "Command GET GRASPING FORCE not successful: %s\n", status_to_str( status ) );
		free( resp );
		return 0;
	}
	
	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];
	
	free( resp );

	return convert(vResult);

	//return (int) resp[2];
}

// MAIN
/*
void test( void )
{
	while( 1 )
	{
		dbgPrint("MOVE\n");
		move( 0.0f, 60.0f );
		getOpening();

		//sleep(1);
		getForce();
		//sleep(1);

		dbgPrint("HOMING\n");
		homing();
		getOpening();
	}
}
*/

//------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------
