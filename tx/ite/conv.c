#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <math.h>
#include <assert.h>
#include "usb_tx.h"

 #define true						    1
 #define false							0
// sampled from current ts (ts file used by fstream)
  //#define TP_CONV // converted ts sample file, liyenho

#ifdef TP_CONV
#define check_payload_unit_start(diff) \
 					assert(sizeof(buff_cnv) >=cnv_sz_p+188-(diff)); \
 					memcpy(buff_cnv+cnv_sz_p, tspkt+(diff), 188-(diff)); \
					if (start) { \
						tspkt[1] &= 0xbf; /*mask off p-u-s*/ \
						tspkt[5] &= 0xef; /*mask off pcr*/ \
						psr = tspkt+(diff); \
						assert(0>cnv_st_p ); \
						assert (0x0 == *(psr+0) || \
										0x0 == *(psr+1) || \
										0x1 == *(psr+2) || \
										0xe0/*mpeg video*/ == *(psr+3)); \
						cnv_st_p = cnv_sz_p; \
						memcpy(pcr_pus,tspkt+6,sizeof(pcr_pus)); \
					} \
 					cnv_sz_p += 188-(diff);

#define generate_extra_tp(size_aval) \
		 			memcpy(extra_tp, \
		 								(pus)?ts_rdo_hdr_pus:ts_rdo_hdr, \
		 								(pus)?sizeof(ts_rdo_hdr_pus):sizeof(ts_rdo_hdr)); \
 					if (pus) { \
	 					memcpy(extra_tp+6, pcr_pus, sizeof(pcr_pus)); \
	 					*(extra_tp+7+sizeof(pcr_pus)) = (unsigned char)(cnt_seq>>8); \
	 					*(extra_tp+8+sizeof(pcr_pus)) = (unsigned char)(cnt_seq); \
 					} else { \
	 					*(extra_tp+7) = (unsigned char)(cnt_seq>>8); \
	 					*(extra_tp+8) = (unsigned char)(cnt_seq); \
					} \
	 				cnt_seq += 1; \
	 				process_payload_unit_start(extra_tp+((pus)?(9+sizeof(pcr_pus)):9/*4*/), \
	 																			(size_aval), extra_tp);

	static /*const*/ unsigned char ts_rdo_hdr[] = { // ctrl radio pid : 0x1000
		0x47,0x10,0x00,0x30/*0x10*/,0x03+1,0x82,0x02, /*ext seq cnt to be filled*/ };
	static /*const*/ unsigned char ts_rdo_hdr_pus[] = { // ctrl radio pid : 0x1000
		0x47,0x10,0x00,0x30/*0x10*/,0x03+1+6,0x92,0x0,0x0,0x0,0x0,0x0,0x0,0x02, /*pcr & ext seq cnt to be filled*/ };
  static unsigned char buff_cnv[3*(188-4)]= {0xff}; // this shall handle all cases,
  static bool pus = false;
  static unsigned char pcr_pus[6 /*PCR:6 bytes*/] ;
  static int cnv_st_p= -1; // invalidated
  static unsigned cnv_sz_p = 0;
	const unsigned ld = 188 - 9/*4*/;
  //static FILE *fdmp = NULL;

static void process_payload_unit_start(unsigned char* dp, int sz, unsigned char *pkt) {
					if (pus) {
//						puts("set P.U.S field");
				 		pkt[1] |= 0x40;  // payload unit start
						pkt[5] |= 0x10; // append pcr associated with 'pus' packet
				 		pus = false;
					}
				unsigned ofst ;
			 		if (0<=cnv_st_p) {
			 			if (sz < cnv_st_p) {
				 			memcpy(dp, buff_cnv, sz);
				 			dp += sz;
				 			cnv_sz_p -= sz;
				 			memmove(buff_cnv, buff_cnv+sz, cnv_sz_p);
				 			cnv_st_p -= sz;
			 			}
			 			else {
				 			memcpy(dp, buff_cnv, cnv_st_p);
				 			dp += cnv_st_p;
				 			cnv_sz_p -= cnv_st_p;
				 			memmove(buff_cnv, buff_cnv+cnv_st_p, cnv_sz_p);
			 				cnv_st_p = -1;
			 				pus = true;
			 			}
		 			}
		 			else {
			 			memcpy(dp, buff_cnv, sz);
				 		dp += sz;
			 			cnv_sz_p -= sz;
				 		memmove(buff_cnv, buff_cnv+sz, cnv_sz_p);
		 			}
		 			static unsigned char cc= 0;
		 			pkt[3] &= 0xf0;
		 			pkt[3] |= 0x0f& cc++ ;
				 	ofst = (unsigned)dp - (unsigned)pkt;
				 	if (188 > ofst)
				 		memset (dp, 0xff, 188-ofst);
	 		}

static void flush_cnv_buff(unsigned char *extra_tp,
														unsigned short *p_cnt_seq,
														/*FILE *fdmp*/unsigned char **obpp,
														uint32_t *obcp ) {
				unsigned short cnt_seq = *p_cnt_seq;
				while (cnv_sz_p) {
					int ld1 = ld-((pus)?sizeof(pcr_pus):0);
					generate_extra_tp((ld1<=cnv_sz_p)?ld1:cnv_sz_p)
			 		//fwrite(extra_tp, 188, 1, fdmp);
			 		memcpy(*obpp, extra_tp, 188);
			 		*obpp += 188;
			 		*obcp += 188;
				}
				//fflush(fdmp);
				*p_cnt_seq = cnt_seq;
		}
#endif

void conv_ts_w_ext_seq_cnt(unsigned char *opbf, uint32_t *obytes, unsigned char *pbf, uint32_t bytes) {
	unsigned char *tspkt = pbf, *outp= opbf;
	unsigned  pid, n;
	*obytes = 0;
 #if false
	static unsigned short cnt_seq = 0; // 14 bits
	unsigned  char *cnt_seq_hi= \
										((unsigned  char *)&cnt_seq)+1;  // use 6 bits
	unsigned  char *cnt_seq_lo = \
										((unsigned  char *)&cnt_seq);  //use 8 bits
	unsigned  ts_type;
	for (n=0; n<bytes/188; tspkt+=188, n++) {
		pid =( ((unsigned)(tspkt[1]&0x1f))<<8) + tspkt[2];
		if(   pid == 0x1fff)  ts_type = 7;
		else if(pid == 0)   ts_type = 0;
		else if (pid == 0xfff) ts_type = 1; // added to differentiate pmt packets, liyenho
		else if(pid == 0x100) ts_type = 2;
		else
		{
		   printf("ERROR: unexpected pid type. \n");
		  ts_type = 7;
		}
		if(ts_type != 7)
		{
		  tspkt[3] =( tspkt[3]&0xf0) |( *cnt_seq_lo |0x0f);
//#define YENDO
#ifndef YENDO
		  tspkt[1] = tspkt[1];
		  tspkt[2]=  tspkt[2];
#else // yendo's scheme
		  tspkt[1] =( tspkt[1]&0xe0) |( ts_type<<2) | (*cnt_seq_hi>>4);
		  tspkt[2]= (*cnt_seq_lo>>4) | (*cnt_seq_hi<<4);
#endif
		  if(*cnt_seq_lo==255){
		   if(*cnt_seq_hi==63){ *cnt_seq_lo=0; *cnt_seq_hi=0;}
		    else {*cnt_seq_lo=0; *cnt_seq_hi++;}
		  }
		else *cnt_seq_lo++;
		}
	}
 #else
 	#ifdef TP_CONV
		ts_rdo_hdr [1] = ts_rdo_hdr_pus [1] = 0x1f & (PID_VID>>8);
		ts_rdo_hdr [2] = ts_rdo_hdr_pus [2] =(unsigned char)PID_VID;
		unsigned char extra_tp[188];
		/*if (!fdmp) {
		 	fdmp = fopen("./vid_ext_seq_cnt.ts", "w");
		}*/
	#endif
 	unsigned char adpf, adfb[3+2];
	static unsigned short cnt_seq = 0; // full 16 bits
	for (n=0; n<bytes/188; tspkt+=188, n++) {
		bool extra= false;
		assert(0x47== *tspkt);
		pid =( ((unsigned)(tspkt[1]&0x1f))<<8) + tspkt[2];
		if ( PID_VID== pid) {
			adpf = tspkt[3]&0x30;
 		  #ifdef TP_CONV
 		   unsigned char *psr, start = 0x40 & tspkt[1];
			if (0x0 == adpf) {
//				puts("adaptation field control is 0, skipped");
				flush_cnv_buff(extra_tp, &cnt_seq,/*fdmp*/ &outp,obytes);
				// throw away the packet then align cnv buffer, in theroem this can't happen...
			}
 			else if (0x10 == adpf) { // no adpf
 				check_payload_unit_start(4)
 				adpf = 0x30;  // attach adpf with private use, followed by ts
 				tspkt[3] = adpf | (tspkt[3]&0x0f);
 				adfb[0] = (pus)?(6+3+1):(3+1);  // 2 byte for ext seq cnt + 1 byte of spec flags
 				adfb[1] = (pus)?0x92:0x82; // turn on private usage
 				adfb[2] = 2;
 			/************************************************/
 				*(unsigned char*)(adfb+3) = (unsigned char)(cnt_seq>>8);
 				*(unsigned char*)(adfb+4) = (unsigned char)(cnt_seq);
 				cnt_seq += 1;
 			/************************************************/
 				unsigned cpy=188-4-sizeof(adfb), ld1 ;
 				if (pus) {
	 				memcpy(tspkt+4, adfb, 2);
	 				memcpy(tspkt+6, pcr_pus, sizeof(pcr_pus));
	 				*(tspkt+6+sizeof(pcr_pus)) = adfb[2];
	 				*(tspkt+7+sizeof(pcr_pus)) = adfb[3];
	 				*(tspkt+8+sizeof(pcr_pus)) = adfb[4];
	 				cpy -= sizeof(pcr_pus);
	 				process_payload_unit_start(tspkt+4+sizeof(adfb)+sizeof(pcr_pus), cpy, tspkt);
 				}
 				else {
	 				memcpy(tspkt+4, adfb, sizeof(adfb));
	 				process_payload_unit_start(tspkt+4+sizeof(adfb), cpy, tspkt);
 				}
 				if (pus)
 					ld1 = ld -sizeof(pcr_pus);
 				else
 					ld1 = ld ;
		 		if (ld1 <=cnv_sz_p) {
			 		generate_extra_tp(ld1)
					extra = true;
	 			}
			} else {
				unsigned  usr = 0x2&*(tspkt+5),
										adpfl = *(tspkt+4),
										ofst=0;
//				printf("adaptation field length = %d\n", adpfl);
				assert(0==usr) ; // can NOT support ts already associated with private data! liyenho
	 				ofst += (0x10 & *(tspkt+5))?6:0;
	 				ofst += (0x08 & *(tspkt+5))?6:0;
	 				ofst += (0x04 & *(tspkt+5))?1:0;
	 			int pcr_adj_sz = sizeof(pcr_pus);
	 			if (0x10 & *(tspkt+5))
	 				pcr_adj_sz = 0;
 				if (0x30 == adpf) {// adpf with ts
 					unsigned vid_ts = 188-(5+adpfl);
					check_payload_unit_start(5+adpfl)
				}  // no ts but adpf
		 			int lusr=/*2+1*/0;
		 			if (188<(/*lusr*/3+((pus)?pcr_adj_sz:0)+adpfl+5)) {
						; // all about stuffing bytes
		 			}
		 			else if ((adpfl-1)<(ofst+3+((pus)?pcr_adj_sz:0))) {
			 			lusr = (ofst+3+((pus)?pcr_adj_sz:0)-(adpfl-1));
		 				ofst += ((pus)?pcr_adj_sz:0);
		 				*(tspkt+4) += lusr;
	 				}
		 			{
	 				*(tspkt+5) |= (pus)?0x92:0x82; // turn on private usage
	 			/************************************************/
	 				*(unsigned char*)(adfb+2) = (unsigned char)(cnt_seq>>8);
	 				*(unsigned char*)(adfb+3) = (unsigned char)(cnt_seq);
	 				cnt_seq += 1;
	 			/************************************************/
	 				*(tspkt+6+ofst) = 2;
	 				memcpy(tspkt+6+ofst+1, &adfb[2], 2);
	 				if (pus)
	 					memcpy(tspkt+6, pcr_pus, sizeof(pcr_pus));
	 			// shift out remained ts if there is any
	 			if (cnv_sz_p) {
		 			*(tspkt+3) |= 0x30; // add vid payload
		 			unsigned left ;
		 			left= 188-(lusr+adpfl+5);
		 			if (left >=cnv_sz_p) {
			 			process_payload_unit_start(tspkt+5+adpfl+lusr,
			 																	cnv_sz_p, tspkt);
					}
		 			else {
			 			process_payload_unit_start(tspkt+5+adpfl+lusr,
			 																	left, tspkt);
			 			int ld1 = ld- ((pus)?sizeof(pcr_pus):0);
			 			if (ld1<=cnv_sz_p) {
				 			generate_extra_tp(ld1)
							extra = true;
			 			}
		 			}
	 			}}
			}
 		  #endif
		}
 	#ifdef TP_CONV
			 	//fwrite(tspkt, 188, 1, fdmp);
			 	memcpy(outp, tspkt, 188);
			 	*obytes += 188;
			 	outp += 188;
			 	if (extra) {
			 		//fwrite(extra_tp, 188, 1, fdmp);
			 		memcpy(outp, extra_tp, 188);
			 		*obytes += 188;
			 		outp += 188;
		 		}
			 	//fflush(fdmp);
		#undef check_payload_unit_start()
		#undef generate_extra_tp()
	#endif
	}
 #endif
}
