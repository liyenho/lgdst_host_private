/*
 * MavLink.c
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "MavLink.h"

#define X25_INIT_CRC 							0xffff
#define X25_VALIDATE_CRC 				0xf0b8

const uint8_t MAVLINK_MESSAGE_CRCS[] = {50, 124, 137,   0, 237, 217, 104, 119,   0,   0,\
									     0,  89,   0,   0,   0,   0,   0,   0,   0,   0,\
									   214, 159, 220, 168,  24,  23, 170, 144,  67, 115,\
									    39, 246, 185, 104, 237, 244, 222, 212,   9, 254,\
									   230,  28,  28, 132, 221, 232,  11, 153,  41,  39,\
									   214, 223, 141,  33,  15,   3, 100,  24, 239, 238,\
									    30, 200, 183,   0, 130,   0, 148,  21,   0,  52,\
									   124,   0,   0,   0,  20,   0, 152, 143,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0, 231,\
									   183,  63,  54,   0,   0,   0,   0,   0,   0,   0,\
									   175, 102, 158, 208,  56,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\ 
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0, 204,\
									    49, 170,  44,  83,  46,   0};

static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        //Accumulate one byte of data into the CRC
        uint8_t tmp;
        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ ((uint16_t)tmp<<8) ^ ((uint16_t)tmp <<3) ^ (tmp>>4);
}


uint32_t Compute_Mavlink_Checksum(MavLinkPacket packet){
	
	uint16_t checksum = X25_INIT_CRC;
	//compute checksum, excluding packet start sign
	for (int i =0; i< (packet.length+MAVLINK_HDR_LEN-1); i++){
		crc_accumulate(*(uint8_t *)((&packet.length)+i), &checksum);
	}
	//add CRC Extra per MavLink definition
	crc_accumulate(MAVLINK_MESSAGE_CRCS[packet.message_ID], &checksum);
	return checksum;
}


bool Check_Mavlink_Checksum(MavLinkPacket packet){
	return (Compute_Mavlink_Checksum(packet) == *(uint16_t *)packet.checksum);
}

uint32_t MavLink_Total_Bytes_Used(MavLinkPacket pkt){
	uint32_t bytes_used = MAVLINK_HDR_LEN + MAVLINK_CHKSUM_LEN + pkt.length;
	return bytes_used;
}


uint32_t MavLink_PackData(MavLinkPacket pkt, uint8_t *buffer){
	memcpy(buffer, &pkt, pkt.length+MAVLINK_HDR_LEN);
	memcpy(buffer+pkt.length+MAVLINK_HDR_LEN, pkt.checksum, MAVLINK_CHKSUM_LEN);
	return MavLink_Total_Bytes_Used(pkt);
}

static uint8_t sequence_cnt = 0;
MavLinkPacket Build_Mavlink_Data_Packet(uint8_t num_bytes, uint8_t *data){
	
	MavLinkPacket pkt;
	pkt.header = MAVLINK_START_SIGN;
	pkt.length = num_bytes;
	pkt.sequence = sequence_cnt++;
	pkt.system_ID = 0xAB; //made up values for testing
	pkt.component_ID = 0xCD; //made up values for testing
	pkt.message_ID = MAVLINK_ID_DATA;
	memcpy(pkt.data, data, num_bytes); //fill in packet payload

	*(uint16_t *) pkt.checksum = (uint16_t) Compute_Mavlink_Checksum(pkt);

	return pkt;
}

void PrintMavLink(MavLinkPacket pkt){
	for (int i=0; i<(pkt.length+MAVLINK_HDR_LEN); i++){
		printf("%02x ", *(uint8_t*)(&(pkt.header)+i) );
	}
	for (int i=0; i<MAVLINK_CHKSUM_LEN; i++){
		printf("%02x ", pkt.checksum[i]);
	}

}