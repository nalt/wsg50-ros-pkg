//======================================================================
/**
 *  @file
 *  functions_can.h
 *
 *  @section functions_can.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author Marc Benetó
 *  @date   14.09.2012
 *  
 *  
 *  @section functions_can.h_copyright Copyright
 *  
 *  Copyright 2012 Robotnik Automation, SLL
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

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

// CAN bus management functions
bool CAN_connect( const char *dev );
void CAN_disconnect( void );

// Goto/grasp commands
void homing( void );
int move( float width, float speed );
int grasp( float objWidth, float speed );
int release( float width, float speed );
//int stop( void );
//int ack_fault( void );

// Set functions
void setAcceleration( float acc );
void setGraspingForceLimit( float force );

// Get functions
float getOpening( void );
float getGraspingForceLimit( void );
float getAcceleration( void );
//const char * systemState( void );
//int graspingState( void );


#ifdef __cplusplus
}
#endif

#endif /* FUNCTIONS_H_ */
