//======================================================================
/**
 *  @file
 *  functions.h
 *
 *  @section testing.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	30.04.2012
 *  
 *  
 *  @section testing.h_copyright Copyright
 *  
 *  Copyright 2012 Weiss Robotics, D-71636 Ludwigsburg, Germany
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

int homing( void );
int move( float width, float speed );
int stop( void );
int grasp( float objWidth, float speed );
int release( float width, float speed );
int ack_fault( void );

int setAcceleration( float acc );
int setGraspingForceLimit( float force );

const char * systemState( void );
int graspingState( void );
int getOpening( void );
int getForce( void );
int getAcceleration( void );
int getGraspingForceLimit( void );

//void getStateValues(); //(unsigned char *);

//void test( void );


#ifdef __cplusplus
}
#endif

#endif /* FUNCTIONS_H_ */
