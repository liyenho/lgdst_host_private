/*
*
* MavLink.h
*
*/


#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_CHKSUM_LEN							2  // two bytes
#ifdef MAVLINK_V1
  #define MAVLINK_START_SIGN					0x55
  #define MAVLINK_HDR_LEN						6
#elif defined(MAVLINK_V2)
  #define MAVLINK_START_SIGN					0xFD
  #define MAVLINK_HDR_LEN						10
#endif

#define MAVLINK_ID_DATA								0x00

//checksum parameters
#define X25_INIT_CRC 								0xffff
#define X25_VALIDATE_CRC 							0xf0b8

#define MAVLINK_MAX_PAYLOAD_LEN				255
#define MAX_MAVLINK_SIZE		(MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN+MAVLINK_MAX_PAYLOAD_LEN)

#define MAVLINK_USB_TRANSFER_LEN		(30+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN)

typedef struct
{
	uint8_t header;
	uint8_t length; //this describes how long the data payload is
 #ifdef MAVLINK_V2
	uint8_t comp_flags[2]; // compatibility flags
 #endif
	uint8_t sequence;
	uint8_t system_ID;
	uint8_t component_ID;
 #ifdef MAVLINK_V1
	uint8_t message_ID;
 #elif defined(MAVLINK_V2)
	uint8_t message_ID[3];
 #endif
	uint8_t data[MAVLINK_MAX_PAYLOAD_LEN]; //actual occupied size determined by length parameter, 255 is max
	uint8_t checksum[MAVLINK_CHKSUM_LEN];
}MavLinkPacket;

#define UART_STR_LEN	/*512*/1024  // for efficiency, to cover 2 max sized pkt back to back

typedef struct { // added by liyenho
	uint32_t sts_next;
	int32_t idx_wr;
	int32_t idx_rd;
	uint8_t pkt_buff[UART_STR_LEN];
} stream_uart;

uint32_t Compute_Mavlink_Checksum(MavLinkPacket *packet);
bool Check_Mavlink_Checksum(MavLinkPacket *packet);
void Set_Mavlink_Checksum(uint8_t *packet) ; //added by liyenho
void Build_Mavlink_Data_Packet(uint8_t *pkt, uint8_t num_bytes, uint8_t *data); // for efficiency & adequacy
uint32_t MavLink_Total_Bytes_Used(MavLinkPacket *pkt);

bool Build_MavLink_from_Byte_Stream(uint8_t *pkt, bool *overrun, uint8_t *rdptr_str, unsigned len);
void PrintMavLink(MavLinkPacket *pkt);

#endif