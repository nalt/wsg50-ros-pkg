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
