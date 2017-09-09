/*
 * MavLink.h
 *
 */ 


#ifndef MAVLINK_H_
#define MAVLINK_H_


#define MAVLINK_CHKSUM_LEN					2  // two bytes
#define MAVLINK_START_SIGN					0x55
#define MAVLINK_HDR_LEN						6
#define MAVLINK_MAX_PAYLOAD_LEN				255
#define MAX_MAVLINK_SIZE				(MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN+MAVLINK_MAX_PAYLOAD_LEN)

#define MAVLINK_ID_DATA						0x00

//checksum parameters
#define X25_INIT_CRC 								0xffff
#define X25_VALIDATE_CRC 							0xf0b8

typedef int bool;

typedef struct 
{
	uint8_t header;
	uint8_t length; //this describes how long the data payload is
	uint8_t sequence;
	uint8_t system_ID;
	uint8_t component_ID;
	uint8_t message_ID;
	uint8_t data[MAVLINK_MAX_PAYLOAD_LEN]; //actual size determined by length parameter, 255 is max
	uint8_t checksum[MAVLINK_CHKSUM_LEN];
}MavLinkPacket;

uint32_t MavLink_Total_Bytes_Used(MavLinkPacket pkt);
uint32_t Compute_Mavlink_Checksum(MavLinkPacket packet);
bool Check_Mavlink_Checksum(MavLinkPacket packet);
uint32_t MavLink_PackData(MavLinkPacket pkt, uint8_t *buffer);
MavLinkPacket Build_Mavlink_Data_Packet(uint8_t num_bytes, uint8_t *data);
void PrintMavLink(MavLinkPacket pkt);
#endif /* MAVLINK_H_ */