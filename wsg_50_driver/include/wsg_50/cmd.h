//======================================================================
/**
 *  @file
 *  cmd.h
 *
 *  @section cmd.h_general General file information
 *
 *  @brief
 *  Command abstraction layer (Header file)
 *
 *  @author wolfer
 *  @date	20.07.2011
 *  
 *  
 *  @section cmd.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef CMD_H_
#define CMD_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "common.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------



#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int cmd_connect_tcp( const char *addr, unsigned short port );
int cmd_connect_udp( unsigned short local_port, const char *addr, unsigned short remote_port );
int cmd_connect_serial( const char *device, unsigned int bitrate );

void cmd_disconnect( void );
bool cmd_is_connected( void );
status_t cmd_get_response_status( unsigned char *response );

int cmd_submit( unsigned char id, unsigned char *payload, unsigned int len,
			    bool pending, unsigned char **response, unsigned int *response_len );


#ifdef __cplusplus
}
#endif

#endif /* CMD_H_ */
