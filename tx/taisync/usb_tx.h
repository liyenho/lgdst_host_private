#ifndef __USB_TX_H__
#define __USB_TX_H__
#include <stdbool.h>  // typedef int bool, liyenho

//#define MEDIA_ON_FLASH
#define SRC_FRM_ENET
#define RADIO_TAISYNC
//#define RFFE_PARAMS
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core

#define USE_MAVLINK								/*0*/1
#ifdef USE_MAVLINK
	#define MAVLINK_USB_LEN						263 //(30+6+2)
	#define MAVLINK_START_SIGN		0x55
	#define MAVLINK_HDR_LEN				6
	#define START_SIGN_LEN				1  // one byte
	#define CHKSUM_LEN							2  // two bytes
	#define X25_INIT_CRC 							0xffff
	 #define MAX_MSG_LEN						85  // max msg length allowed
	#define CTRL_SEND_MAV_POLLPERIOD				100000
	#define CTRL_RECV_MAV_POLLPERIOD				100000
#endif

#define DEBUG_CTRLRX                    1/*0*/
#define DEBUG_CTRLTX                    /*1*/0
#define DEBUG_VIDEO                     1/*0*/

#define LO_Frequency 							/*1583000*/ 1693000
	#define VID_CH_BW							6000
	#define VID_CH_TTL							(VID_CH_BW+1000) // include guard band
	#define MAX_VID_CH_F					2478000
	#define MIN_VID_CH_F					2406000
	#define NUM_OF_VID_CH			((MAX_VID_CH_F-MIN_VID_CH_F)/VID_CH_TTL)
 #if NUM_OF_VID_CH < 3
	#error Number of video channels cannot be less than 3
 #endif
	#define VID_IF_CH_BASE				(MIN_VID_CH_F-LO_Frequency)
	#define VID_IF_CH_CEIL				VID_IF_CH_BASE+(NUM_OF_VID_CH-1)*VID_CH_TTL
	#define VID_CH_STR_THR			-61	// translated to approximately 1 miles attenuation

#define UDP_PACKET_MAX 							1880
#define HOST_BUFFER_SIZE						(256-1) // max data len-1
enum TYPE {
	CMD0,	/*w/o params*/
	CMD1,	/*w single param*/
	ACS, 	/*regs access*/
};
typedef struct {
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address, 12 bit
	//uint8_t toflash; // request to burn to flash or not
	uint8_t data[1]; // only valid on lower half
} dev_access;
typedef struct {
	dev_access hdr;
	uint8_t data[HOST_BUFFER_SIZE];
} dAccess; // dev access of ctrl xfer
typedef struct {
	uint16_t  wDir;  // usb direction
	uint16_t  wValue;
	uint16_t  wIndex;
} uTag; // usb tags of ctrl xfer
typedef struct {
  int active;
  bool echo;
	uint16_t  type;
	uint16_t len;
	uTag   tag;
	dAccess access;
} ipcLgdst;

typedef enum {
	FIVE_MHZ,
	TEN_MHZ,
	TWENTY_MHZ,
	FORTY_MHZ,
} VCH_BW;

typedef enum {
	BPSK_1B2,
	BPSK_3B4,
	BPSK_5B6,
	QPSK_1B2,
	QPSK_3B4,
	QPSK_5B6,
	QAM16_1B2,
	QAM16_3B4,
	QAM16_5B6,
	QAM64_1B2,
	QAM64_3B4,
	QAM64_5B6,
} VCH_MD;

typedef enum {
	OFDM_BW = 0,
	OFDM_MCS = 1,
	OFDM_CARR = 2,
	RDO_PWR = 3,	// max 26 dbm
	RDO_ANT = 4,	// on:1/off:0
	RDO_SNR = 5,
	LDPC_FAILED= 6,
	RDO_RSSI = 7,
	RDO_RX_VGA = 8,
	RX_BB_STS= 9,
	DNLK_BUF_STS= 10,
	UPLK_BUF_STS= 11,
	CMD_STS = 12,
	TOTAL_FRMS_DN= 13,
	LOST_FRMS_RX= 14,
	TOTAL_FRMS_UP= 15,
	LOST_FRMS_TX= 16,
	TS_VID_ACTIVE =17,  // allow video to startup
} access_mode;

#define EXTRA															16  // randomly send more data on bulk pipe
//#define FWUP_DNLD_DBG							// don't turn in on unless it's necessary, not completely stable!
#define USB_SYSTEM_RESTART_VAL				0xa5
#define FW_UPGRADE_HDR_LEN					(12)
#define ATMEL_UPGRADE_HDR_LEN		(12-4) // starting address would be calc by atmel bootloader
#define USB_STM_UPGRADE_VAL				0x21  // stm32 upgrade cmd
#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
// these two are pertaining only to atmel operation
#define USB_FWM_BOOTUP_VAL					0xbe
#define USB_FWM_UPDATE_VAL					0xef
#define USB_STREAM_ON_VAL						0xe
#define USB_STREAM_IDX									0x1	// data instead comm interface
#define USB_QUERY_IDX										0xff
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
#ifdef  RADIO_TAISYNC
 #define RADIO_COMM_VAL								0x10  // using 5 bit out of 16 bit should be alright?
  // it should be safe to use wIndex alternatively instead pointer to interface index
  #define RADIO_STARTUP_IDX						0x2
  #define RADIO_DATA_TX_IDX 						0x3
  #define RADIO_DATA_RX_IDX						0x4
  #define RADIO_USR_TX_LEN		30 // ctl/sts radio payload byte length
  #define RADIO_USR_RX_LEN    30
  #define CTRL_SEND_POLLPERIOD    /*60500*/ /*54000*/ 48000  //us: 160000 = 1.5kbps
  #define CTRL_RECV_POLLPERIOD    /*70000*/ /*(70000/2)*/ /*(30000/4)*/ /*(27000/4)*/ (24000/4) //us: must be > 2*CTRL_SEND_POLLPERIOD
  #define RADIO_INFO_LEN  4 // for rx statistics
  #define CTRL_SEND_FIFODEPTH  8
  #define CTRL_RECV_FIFODEPTH  8
#endif

  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
 #ifdef MEDIA_ON_FLASH     //must also disable SRC_FRM_ENET
  #define USB_LOAD_MEDIA									0xb
  #define USB_LOADM_IDX								0x1	// data instead comm interface
  #define USB_LOADM_LEN								sizeof(int)  // media file byte length
  //#define DEBUG_FL
 #endif

#ifdef RFFE_PARAMS
 	typedef struct {
 		uint8_t  addr;
 		uint16_t data;
 	} dev_cfg;

 typedef union {
	 struct {
		 uint16_t chan_idx;
		 uint16_t pwr_att;
		 uint16_t tone_on; // tone generation, 1: turn on, 0: turn off
	 } params_tx;
	 struct {
		 dev_cfg *pregs_6612;
	 } params_rx;
 } rf_params;

 #define RF_TX_VAL								0x11
 #define RF_RX_VAL								0x12
#endif

#endif //__USB_TX_H__
