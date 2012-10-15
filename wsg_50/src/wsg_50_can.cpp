#include "wsg_50/checksum.h"
#include "wsg_50/msg.h"
#include "wsg_50/common.h"
#include <libpcan.h>
#include <fcntl.h> 
#include <stdio.h>
#include <pcan.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_NODE "/dev/pcan32"


int main( void ){

	HANDLE h;
  	const char *szDevNode = DEFAULT_NODE;
	char txt[64];
	int errno;

	printf("Openning PEAK CAN device...\n");

  	h = LINUX_CAN_Open(szDevNode, O_RDWR | O_NONBLOCK);

  	if (!h) {
		printf("Can't open PEAK CAN device.\n");
		return -1;
    	}

	CAN_VersionInfo(h, txt);

	printf("Version Info: %s\n", txt);

  	errno = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST); // _ST:standard frames, _EX:extended frames 

  	if (errno) {
		printf("Can't init PEAK CAN device.\n");
    		return -1;
    	}

	sleep(2);

	// Create the message to send

	TPCANMsg msg;
	unsigned short crc;
	unsigned char header[6]; 
	unsigned char data[1];
	char can_id;
	
	
	header[0] = 0xAA;
	header[1] = 0xAA;
	header[2] = 0xAA;
	header[3] = 0x20;
	header[4] = 0x01;
	header[5] = 0x00;


	data[0] = 0x00;								// DATA  0x00 => Normal homing. 0x02 => Inverted homing.

	crc = checksum_crc16( header, 6 );
	crc = checksum_update_crc16( data, 1, crc );

	printf("CRC: %x\n", crc);
	printf("CRC-lo (0x8F): %x\n", lo(crc));
	printf("CRC-hi (0x83): %x\n", hi(crc));

	can_id = 0x01; // DEVICE ID
	

	// Homing message (1st part)
    msg.ID = can_id;
    msg.LEN = 0x08;         // 8 data byte
    msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.DATA[0]= header[0];		// PREAMBLE[0]
    msg.DATA[1]= header[1];		// PREAMBLE[1]
	msg.DATA[2]= header[2];		// PREAMBLE[2]
    msg.DATA[3]= header[3];		// ID MESSAGE
    msg.DATA[4]= header[4];		// SIZE PAYLOAD [0]
    msg.DATA[5]= header[5];		// SIZE PAYLOAD [1]
    msg.DATA[6]= data[0];		// DATA[0]
    msg.DATA[7]= (unsigned char) lo(crc); //0x8F; //0xCD; //lo(crc);		// CRC[0]


	CAN_Write(h, &msg);

	 
	/*
     	int iRet = CAN_Status(h);
     	if(iRet < 0){
          	int err = nGetLastError();
        	printf("Send error in CAN_Write() errno=%d iRet=%d nGetLastError=%d\n", errno, iRet, err);
        }
	*/

	// Homing message (2nd part)
		
    msg.ID = can_id;
    msg.LEN = 0x01;         // 1 data byte
	msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.DATA[0]= (unsigned char) hi(crc); //0x83; //0xA3; //hi(crc);		// CRC[1]
    msg.DATA[1]= 0;
    msg.DATA[2]= 0;
    msg.DATA[3]= 0;
    msg.DATA[4]= 0;
    msg.DATA[5]= 0;
    msg.DATA[6]= 0;
    msg.DATA[7]= 0;

	CAN_Write(h, &msg);

     	int iRet = CAN_Status(h);
     	if(iRet < 0){
          	int err = nGetLastError();
        	printf("Send error in CAN_Write() errno=%d iRet=%d nGetLastError=%d\n", errno, iRet, err);
        }
	
	printf("HOMING message sent.\n");


	//sleep(3); 
	
	// Perform a simple movement
	
	float width = 35.0; // mm
	float speed = 100.0; // mm/s
	
	can_id = 0x01;
	
	header[0] = 0xAA;
	header[1] = 0xAA;
	header[2] = 0xAA;
	header[3] = 0x21;
	header[4] = 0x09;
	header[5] = 0x00;

	unsigned char dataMove[9];
	dataMove[0] = 0x00;
	/*
	dataMove[1] = 0x00;
    dataMove[2] = 0x00;
	dataMove[3] = 0x48; //4842 = 50
	dataMove[4] = 0x42;
	dataMove[5] = 0x00;
	dataMove[6] = 0x00;
	dataMove[7] = 0x48;
	dataMove[8] = 0x42;
	*/			
						
	memcpy( &dataMove[1], &width, sizeof( float ) );
	memcpy( &dataMove[5], &speed, sizeof( float ) );
	
	//for (int i = 0; i<9; i++){	
	//	printf("dataMove[%d]: 0x%x\n", i, dataMove[i]); 
	//}
	
	crc = checksum_crc16( header, 6 );
	crc = checksum_update_crc16( dataMove, 9, crc );

	
    msg.ID = can_id;
	msg.LEN = 0x08;         // 8 data byte
	msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.DATA[0]= header[0];		// PREAMBLE[0]
    msg.DATA[1]= header[1];		// PREAMBLE[1]
    msg.DATA[2]= header[2];		// PREAMBLE[2]
    msg.DATA[3]= header[3];		// ID MESSAGE
    msg.DATA[4]= header[4];		// SIZE PAYLOAD [0]
    msg.DATA[5]= header[5];		// SIZE PAYLOAD [1]
    msg.DATA[6]= dataMove[0];	// DATA[0]
    msg.DATA[7]= dataMove[1];	// DATA[1]

	CAN_Write(h, &msg);

    msg.ID = can_id;
	msg.LEN = 0x08;         // 8 data byte
	msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.DATA[0]= dataMove[2];	// DATA[2]
    msg.DATA[1]= dataMove[3];	// DATA[3]
    msg.DATA[2]= dataMove[4];	// DATA[4]
    msg.DATA[3]= dataMove[5];	// DATA[5]
    msg.DATA[4]= dataMove[6];	// DATA[6]
    msg.DATA[5]= dataMove[7];	// DATA[7]
    msg.DATA[6]= dataMove[8];	// DATA[8]
    msg.DATA[7]= (unsigned char) lo(crc); // CRC[0]
    
    CAN_Write(h, &msg);

    msg.ID = can_id;
	msg.LEN = 0x01;         // 1 data byte
	msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.DATA[0]= (unsigned char) hi(crc);	// CRC[1]
    msg.DATA[1]= 0;
    msg.DATA[2]= 0;
    msg.DATA[3]= 0;
    msg.DATA[4]= 0;
    msg.DATA[5]= 0;
    msg.DATA[6]= 0;
    msg.DATA[7]= 0;
 
    CAN_Write(h, &msg);
    
    printf("MOVE (50 mm) message sent.\n");
    
    
	TPCANRdMsg *tpcmsg;
	
/*
	int i = 0;

	while(i < 100){
	iRet = LINUX_CAN_Read(h, tpcmsg);

    	if (iRet == CAN_ERR_OK){  // Read OK
		printf("Success reading.\n");
	}else{
		 int last_err = nGetLastError();
		 printf("wsg_50_can: error in LINUX_CAN_Read() - MSGTYPE_STATUS - err=%d\n", last_err);
	}

	i++;

	}
*/
	sleep (10);
	
	CAN_Close(h);
	
}
