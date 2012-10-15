//======================================================================
/**
 *  @file
 *  serial.h
 *
 *  @section serial.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	08.07.2011
 *  
 *  
 *  @section serial.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef SERIAL_H_
#define SERIAL_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------



#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	const char *device;
	unsigned int bitrate;
} ser_params_t;


typedef struct
{
	int fd;
} ser_conn_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int serial_open( const void *params );
void serial_close( void );
int serial_read( unsigned char *buf, unsigned int len );
int serial_write( unsigned char *buf, unsigned int len );


#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H_ */
