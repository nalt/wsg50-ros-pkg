//======================================================================
/**
 *  @file
 *  checksum.h
 *
 *  @section checksum.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	19.07.2011
 *  
 *  
 *  @section checksum.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef CHECKSUM_H_
#define CHECKSUM_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------



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

unsigned short checksum_update_crc16( unsigned char *data, unsigned int size, unsigned short crc );
unsigned short checksum_crc16( unsigned char *data, unsigned int size );

#ifdef __cplusplus
}
#endif

#endif /* CHECKSUM_H_ */
