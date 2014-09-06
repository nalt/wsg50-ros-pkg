//======================================================================
/**
 *  @file
 *  char.h
 *
 *  @section char.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author wolfer
 *  @date	16.09.2011
 *  
 *  
 *  @section char.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71636 Ludwigsburg, Germany
 *  
 *  The distribution of this code and excerpts thereof, neither in 
 *  source nor in any binary form, is prohibited, except you have our 
 *  explicit and written permission to do so.
 *
 */
//======================================================================


#ifndef CHAR_H_
#define CHAR_H_

//------------------------------------------------------------------------
// Character constants
//------------------------------------------------------------------------

/* Special character encodings */

#define ISO_NEWLINE     ( (unsigned char) 0x0a )	// '\ņ'
#define ISO_TAB     	( (unsigned char) 0x09 )	// '\t'
#define ISO_NULL		( (unsigned char) 0x00 )	// '\0'
#define ISO_CARRIAGE	( (unsigned char) 0x0d )	// '\r' (Carriage return)
#define ISO_ESC			( (unsigned char) 0x1b )	// '\033' (ESCAPE)
#define ISO_SPACE    	( (unsigned char) 0x20 )	// ' '
#define ISO_BANG     	( (unsigned char) 0x21 )	// '!'
#define ISO_PERCENT  	( (unsigned char) 0x25 )	// '%'
#define ISO_PERIOD   	( (unsigned char) 0x2e )	// '.'
#define ISO_SLASH    	( (unsigned char) 0x2f )	// '/'
#define ISO_BACKSLASH	( (unsigned char) 0x5c )	// '\'
#define ISO_COLON    	( (unsigned char) 0x3a )	// ':'
#define ISO_AMP			( (unsigned char) 0x26 )	// '&'
#define ISO_HYPHEN		( (unsigned char) 0x2d )	// '-'
#define ISO_QUOTE		( (unsigned char) 0x22 )	// '"'
#define ISO_EQUALS		( (unsigned char) 0x3d )	// '='
#define ISO_SEMICOLON	( (unsigned char) 0x3b )	// ';'
#define ISO_QUESTION	( (unsigned char) 0x3f )	// '?'


#endif /* CHAR_H_ */
