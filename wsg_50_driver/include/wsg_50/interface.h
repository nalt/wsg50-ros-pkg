//======================================================================
/**
 *  @file
 *  interface.h
 *
 *  @section interface.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section interface.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef INTERFACE_H_
#define INTERFACE_H_

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
	const char *name;
	int ( *open ) ( const void *params );
	void ( *close ) ( void );
	int ( *read ) ( unsigned char *, unsigned int );
	int ( *write ) ( unsigned char *, unsigned int );
} interface_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

const interface_t * interface_get( const char *name );


#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_H_ */
