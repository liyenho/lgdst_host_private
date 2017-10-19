/*
*
* MavLink.c
*
*/
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "MavLink.h"

uint8_t MAVLINK_MESSAGE_CRCS[] = {50, 124, 137,   0, 237, 217, 104, 119,   0,   0,\
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



uint32_t Compute_Mavlink_Checksum(MavLinkPacket *packet){

	uint16_t checksum = X25_INIT_CRC;
	//compute checksum, excluding packet start sign
	for (int i =0; i< (packet->length+MAVLINK_HDR_LEN-1); i++){
		crc_accumulate(*(uint8_t *)((&packet->length)+i), &checksum);
	}
	//add CRC Extra per MavLink definition
	int msg_id = (sizeof(MAVLINK_MESSAGE_CRCS)<=packet->message_ID)?0:packet->message_ID;
	crc_accumulate(MAVLINK_MESSAGE_CRCS[msg_id/*protect against msg corruption*/], &checksum);

	return checksum;
}


static uint8_t sequence_cnt = 0;
void Build_Mavlink_Data_Packet(uint8_t *pkt0, uint8_t num_bytes, uint8_t *data)
{

	MavLinkPacket *pkt = (MavLinkPacket*)pkt0; // revised as below,
	pkt->header = MAVLINK_START_SIGN;
	pkt->length = num_bytes;
	pkt->sequence = sequence_cnt++;
	pkt->system_ID = 0xAB; //made up values for testing
	pkt->component_ID = 0xCD; //made up values for testing
	pkt->message_ID = MAVLINK_ID_DATA;
	memcpy(pkt->data, data, num_bytes); //fill in packet payload

	uint16_t chksm = Compute_Mavlink_Checksum(pkt);
	uint8_t lo=0, *pcs = (uint8_t*)&chksm,
					*pkt1 = ((uint8_t*)pkt+num_bytes+MAVLINK_HDR_LEN);
	do {	// in place of memcpy for efficiency,
		*((uint8_t*)pkt1+lo) = *pcs++;
	} while(++lo < MAVLINK_CHKSUM_LEN);

}


void Set_Mavlink_Checksum(uint8_t *packet){
	int pk_len = ((MavLinkPacket*)packet)->length;
	uint8_t *chk = packet+ MAVLINK_HDR_LEN+ pk_len;
	*(packet+MAX_MAVLINK_SIZE-1) = *(chk+1);
	*(packet+MAX_MAVLINK_SIZE-2) = *chk;
}

bool Check_Mavlink_Checksum(MavLinkPacket *packet){
	uint16_t expected = Compute_Mavlink_Checksum(packet);
	uint16_t actual = *(uint16_t *)packet->checksum;
	return (expected == actual);
}


uint32_t MavLink_Total_Bytes_Used(MavLinkPacket *pkt){
	uint32_t bytes_used = MAVLINK_HDR_LEN + MAVLINK_CHKSUM_LEN + pkt->length;
	return bytes_used;
}

static bool header_found = false;
static uint32_t idx =0;
static uint32_t checksum_idx=0;

bool Build_MavLink_from_Byte_Stream(uint8_t *pkt, bool *overrun, uint8_t *rdptr_str, unsigned len){
#define SEARCH_NEXT(sn) \
		idx_rd = (UART_STR_LEN-1) & (idx_rd+1); \
		if (idx_wr == idx_rd) { \
			ctx_uart.idx_rd = idx_wr; \
			ctx_uart.sts_next = sn; \
			return (false);  /*search exhausted*/ \
		} \
		next_byte = *(ctx_uart.pkt_buff+idx_rd);
#define SEARCH_COMPLETE(sn) \
		if (sn) { \
			ctx_uart.idx_rd = idx_rd; \
			ctx_uart.sts_next = sn; \
			return false; \
		} \
		else /*search restart*/\
			goto sts_next_0;

	static stream_uart ctx_uart= { 0};
	uint8_t tmp_l, next_byte;
	int32_t idx_wr = ctx_uart.idx_wr,
					idx_rd = ctx_uart.idx_rd,
					tlen0, tlen1, tlen2;
	// check overrun condition
	*overrun = false;
	if (idx_rd<=idx_wr && UART_STR_LEN<idx_wr+len) {
		tlen0 = (UART_STR_LEN-1) & (idx_wr+len);
		if (tlen0 >= idx_rd)
			*overrun = true;
	}
	else if (idx_rd>idx_wr && (idx_rd-1)<(idx_wr+len)) {
		*overrun = true;
	}
	// transfer to intr buffer from bit stream
	if (UART_STR_LEN<=len+idx_wr) {
		tlen0 = UART_STR_LEN-idx_wr;
		memcpy (ctx_uart.pkt_buff+idx_wr,
							rdptr_str, tlen0);
		idx_wr= len-tlen0;
		memcpy(ctx_uart.pkt_buff,
							rdptr_str+tlen0, idx_wr);
	}
	else {
		memcpy (ctx_uart.pkt_buff+idx_wr,
							rdptr_str, len);
		idx_wr += len;
	}
	ctx_uart.idx_wr =idx_wr ;
	// load a byte from intr buffer
	next_byte = *(ctx_uart.pkt_buff+idx_rd);
	// check for next state
	switch(ctx_uart.sts_next) {
		case 1: goto sts_next_1;
		default: break;
	}
	// search start sign only when previous search ended on mav pkt boundary
	while (1) {
sts_next_0:
		if (MAVLINK_START_SIGN == next_byte) {
			SEARCH_NEXT(1)
sts_next_1:
			tlen1 = MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN+next_byte-1/*-1*/;
			tlen0 = (UART_STR_LEN-1) & (idx_wr - idx_rd);
			if (tlen1<tlen0) {
				tlen2 = (UART_STR_LEN-1)&(idx_rd+tlen1);
				tmp_l = *(ctx_uart.pkt_buff+tlen2);
				if (MAVLINK_START_SIGN != tmp_l) {
					SEARCH_COMPLETE(0)
				}
				else {
					pkt[0] = MAVLINK_START_SIGN;
					pkt[1]= next_byte;
					// found a mav pkt
					if (UART_STR_LEN<idx_rd+tlen1) {
						tlen2 = UART_STR_LEN-idx_rd-1;
						memcpy(pkt+2, ctx_uart.pkt_buff+idx_rd+1, tlen2);
						memcpy(pkt+tlen2+2, ctx_uart.pkt_buff, tlen1-tlen2-1);
					}
					else {
						memcpy(pkt+2,
										ctx_uart.pkt_buff+idx_rd+1, tlen1-1);
					}
					// update ctx for next run
					ctx_uart.idx_rd = (UART_STR_LEN-1) & (idx_rd+ tlen1);
					ctx_uart.sts_next = 0;
				return true;
			}
		}
			else {
				SEARCH_COMPLETE(1)
			}
		}
		SEARCH_NEXT(0)
	}

#undef SEARCH_NEXT
#undef SEARCH_COMPLETE
}

void PrintMavLink(MavLinkPacket *pkt)
{
	for (int i=0; i<(pkt->length+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN); i++){
		printf("%02x ", *(uint8_t*)(&(pkt->header)+i) );
	}

}