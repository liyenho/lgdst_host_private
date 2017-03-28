

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "usb_tx.h"
#include "radio_link_quality.h"



#define SAMPLE_SIZE 500
#define MAX_ID 		255

uint8_t get_pkt_id(uint8_t * pkt_start);
bool check_pkt_contents(uint8_t * pkt_start);

//statistics
double pkt_receive_rate = 0;
double pkt_correct_percent = 0;

//circular buffers for updating statistics
uint32_t missing_pkts[SAMPLE_SIZE] = {0};
uint32_t pkt_ids[SAMPLE_SIZE] = {0};
bool pkt_payload_ok[SAMPLE_SIZE] = {0};

uint32_t pkt_missing_cnt =0;
uint32_t pkt_payload_ok_cnt=0;

//index for circular buffer
uint32_t idx =0;


uint32_t expec_pkt_id=0;

//keep track of number of packets we should have seen
//used to calculate the packet receive rate
uint32_t expec_total_pkt_cnt = 0; 

bool first_n_samples = true; //for tracking first SAMPLE_SIZE iterations

void check_packet(uint8_t * radio_packet){

	if (!first_n_samples){
		//remove last entry 
		pkt_missing_cnt -= missing_pkts[idx];
		pkt_payload_ok_cnt -= pkt_payload_ok[idx]; 
		expec_total_pkt_cnt -= (missing_pkts[idx]+1);
	}
	//checking packet ID
	pkt_ids[idx] = get_pkt_id(radio_packet);
	missing_pkts[idx] = (pkt_ids[idx] - expec_pkt_id);
	expec_total_pkt_cnt += missing_pkts[idx] + 1; //
	expec_pkt_id = (pkt_ids[idx]+1)%(MAX_ID+1); //update for next call

	//checking packet contents
	pkt_payload_ok[idx] = check_pkt_contents(radio_packet)?1:0;

	//update statistics
	pkt_missing_cnt += missing_pkts[idx];
	pkt_payload_ok_cnt += pkt_payload_ok[idx]; 
	

	pkt_receive_rate = (100.0*(expec_total_pkt_cnt-pkt_missing_cnt))/expec_total_pkt_cnt;
	pkt_correct_percent =  (100.0*pkt_payload_ok_cnt)/SAMPLE_SIZE;


	if ((idx%100) == 0){
		//print periodic status
		printf("Packet ID correct rate: %.2f\%\n", pkt_receive_rate);
		printf("Packet payload correct rate: %.2f\%\n", pkt_correct_percent);
	}

	//update index for next call
	idx++; 
	if (idx >= SAMPLE_SIZE)
	{
		first_n_samples = false;
		
		idx =0;
	}
	return;
}

//index of the packet's 'ID'
const uint8_t pkt_id_idx = 5;

uint8_t get_pkt_id(uint8_t * pkt_start){
	return *(pkt_start+pkt_id_idx);
}


const uint8_t EXPECTED_MSG_CHAR = 0xB5;
bool check_pkt_contents(uint8_t * pkt_start){

	for (int i=0;i<RADIO_USR_TX_LEN;i++){
		//skip counter indexes
		if ((i>=2) && (i<=5)){
			continue;
		}

		if (EXPECTED_MSG_CHAR != *(pkt_start+i)){
			//mismatch
			//printf("Packet Error!\t");
			return false;
		}

	}

	//didn't encounter any problems, return message ok
	//printf("Packet OK\t");
	return true;
}