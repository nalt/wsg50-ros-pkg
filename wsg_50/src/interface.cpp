//======================================================================
/**
 *  @file
 *  interface.c
 *
 *  @section interface.c_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section interface.c_copyright Copyright
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
#include <string.h>

#include "wsg_50/common.h"
#include "wsg_50/interface.h"

// Available interfaces
#include "wsg_50/tcp.h"
#include "wsg_50/udp.h"
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

// Interface structs
extern const interface_t tcp;
extern const interface_t udp;
extern const interface_t serial;

// Collection of interfaces, NULL terminated
static const interface_t *interfaces[] =
{
	&tcp,
	&udp,
	&serial,
	NULL
};


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
 * Get interface with the given name
 *
 * @param *name		Interface name string
 *
 * @return Pointer to interface struct
 */

const interface_t * interface_get( const char *name )
{
	unsigned int i = 0;

	while ( interfaces[i] != NULL )
	{
		if ( strcmp( name, interfaces[i]->name ) == 0 ) return interfaces[i];
		i++;
	}

	return NULL;
}


//------------------------------------------------------------------------
// Test implementation
//------------------------------------------------------------------------
