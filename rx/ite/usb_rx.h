#ifndef __USB_RX_H__
#define __USB_RX_H__

#include "type.h"
#include "rf2072_set.h"
typedef int bool;

#define true							1
#define false							0

#define DBG_USB_CTRL_TX	0 // turn on/off ctrl tx debug print,
#define DBG_USB_RX      1
#define DYNAMIC_VIDEO_CHANNEL_SCAN   0
#define RADIO_SI4463
#define SHMKEY_TX 								1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 								5678	 //rx shared memory key for IPC between lgdst/core
#define STC_HALF_RATE							(90000/2)  // divided by 2 because keep stc value on even boundary

#define CTRL_OUT								(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_IN									(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define USB_RQ									0x04
#define USB_DEV_VENDER							0x03eb
#define USB_DEV_PRODUCT			 				0x2404
#define USB_DEV_INTF							1  				// cdc comm intf included
#define USB_DEV_EP								1				// atmel CDC data ep 1 in
#define EP_DATA									(USB_DEV_EP | LIBUSB_ENDPOINT_IN)

#define FILE_NAME								 "NativeMedia-rec.ts"
#define FILE_LEN								12540164
#define UDP_PACKET_MAX 							1880
#define UDPOUT_PORT         					5558

//#define VIDEO_DUAL_BUFFER		// accommodate dual stream protection scheme
#ifdef VIDEO_DUAL_BUFFER
	  /*376*8/(1500*10^-6) approximate 2 mb/s,
  	to be 1333.333 ts pkts, take 251920=1340*188 to accommodate 1880 blk */
 	#define ONE_SEC_WORTHY	251920
 	#define SEQ_SCH_TOL					(100-1)
 	#define A_QUARTER_LESS		(3*ONE_SEC_WORTHY/4)
 	typedef struct {
	 	uint8_t *vid_sch_buff,
	 					*vid_sch_buff_e,
	 					*vid_sch_ptr;
		uint8_t *ptw_ts_b,
						*ptw_ts_e,
						*ptw_ts;
		bool  sch_buff_full,
					update_n;  // flag to tell whether if to upadte ext seq cnt next
		int buf_flag,
				ext_seq_cnt_n;
 	} ctx_dual_stream;
#endif
#define FRAME_SIZE_A							1880
#define FRAME_SIZE_V							307200
#define FRAME_SIZE_V2							(FRAME_SIZE_V*2)
#define ITERS									(FILE_LEN/FRAME_SIZE_A)
#define FRAME_BUFFS								5
#define servIP  								"127.0.0.1"
#define ctrl_port_base 							5560
#define ERR_CHK_BUFFS                   	    160
#define HOST_BUFFER_SIZE						(256-1) // max data len-1

enum TYPE {
	CMD0,	//w/o params
	CMD1,	//w single param
	ACS, 	//regs access
};

typedef struct {
	int8_t error;  // error status returned
	uint8_t access;
	uint8_t dcnt;	  // data count
	uint16_t addr;	  // first access address
	int8_t reserved[3];	// padded to align on 32 bit boundary
	uint8_t data[1];
} dev_access;

typedef struct {
	dev_access hdr;
	uint8_t data[HOST_BUFFER_SIZE];
} dAccess;           // dev access of ctrl xfer

typedef struct {
	uint16_t  wDir;  // usb direction
	uint16_t  wValue;
	uint16_t  wIndex;
} uTag;              // usb tags of ctrl xfer

typedef struct {
	uint16_t  type;
	uint16_t len;
	uTag   tag;
	uint8_t reserved[2]; // padded to align on 32 bit boundary
	dAccess access;
	int active;
	bool echo;
} ipcLgdst;

typedef enum{
	RF2072_RESET		= 0,//rf2072 reset
	IT913X_READ 		= 1,//i2c read
	IT913X_WRITE 		= 2,//i2c write
	RF2072_READ         = 3,//spi read
	RF2072_WRITE        = 4,//spi write
	IT913X_DOWNLOAD 	= 5,//i2c download firmware
	TS_VID_ACTIVE 		= 6,//ts stream

}access_mode;

#define USB_SYSTEM_RESTART_VAL					0xa5
#define USB_INIT_VID_SUBSYS								0xa1		// ite video subsystem init
#define USB_START_VID_SUBSYS							0xa2		// ite video subsystem start
#define FW_UPGRADE_HDR_LEN						(12)
#define ATMEL_UPGRADE_HDR_LEN					(12-4)  // starting address would be calc by atmel bootloader
#define USB_CPLD_UPGRADE_VAL					0x21  	// cpld upgrade cmd
//#define USB_FPGA_UPGRADE_VAL					0x22  	// fpga upgrade cmd
#define USB_FWM_BOOTUP_VAL						0xbe
#define USB_FWM_UPDATE_VAL						0xef
#define USB_STREAM_ON_VAL						0xe
#define USB_QUERY_IDX							0xff
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9

#define RF_RX_VAL								0x12

#ifdef  RADIO_SI4463

#define MAVLINK_USB_LEN							263//(30+6+2)
#define USE_MAVLINK								/*0*/ 1

#define RADIO_COMM_VAL							0x10    // using 5 bit out of 16 bit should be alright?
#define RADIO_STARTUP_IDX						0x2
#if USE_MAVLINK
  #define RADIO_MAVLEN_OUT_IDX				0x5
  #define RADIO_MAVLEN_IN_IDX				0x6
#endif
#define RADIO_DATA_TX_IDX 						0x3
#define RADIO_DATA_RX_IDX						0x4
#define RADIO_USR_TX_LEN						30 		// ctl/sts radio payload byte length
#define RADIO_USR_RX_LEN						30 		// ctl/sts radio payload byte length
#define RADIO_INFO_LEN  						4 		// gives usb pipe info
//#define USE_915MHZ
#ifdef USE_915MHZ
  #define CTRL_SEND_POLLPERIOD    				48000 	//us:
  #define CTRL_RECV_POLLPERIOD     				(48000/3) //us:
#else  // 869 mhz
  #define CTRL_SEND_POLLPERIOD    				150000 	//us:
  #define CTRL_RECV_POLLPERIOD     				(150000/3) //us:
#endif
#define CTRL_SEND_FIFODEPTH  				 	8
#define CTRL_RECV_FIFODEPTH  				 	8

#endif

#define USB_HOST_MSG_IDX						0x1		// data instead comm interface
#define USB_HOST_MSG_LEN						sizeof(dev_access)
#define EXTRA									16  	// randomly send more data on bulk pipe
#define USB_ITE_FW_VAL							0x6
#define ITE_FW_HDR_LEN							(4)
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
#define VIDEO_SETVCH_VAL								0x14
  #define VIDEO_SETVCH_IDX								0x2

extern bool stream_on ; // ctrl xfer access speed flag,
intmax_t get_file_size(const char* file_path);
void DieWithError(char *errorMessage) ;
void at_exit(int status);
void perror_exit(char*message ,int status);
int stream_block(unsigned char* pdata,int length) ;
int short_sleep(double sleep_time);
void sigint_handler(int signum);
bool open_ini(int *setting_rec);
bool update_ini(int *setting_rec);
uint32_t TEI_exam_pkt_ts(uint32_t size, uint8_t *tsbuf, uint8_t *retbuf);
void rffe_write_regs(dev_cfg* pregs, int size);
int cpld_firmware_update(int mode, const char*file_name);
#if (/*1*/0)
  int init_device(void);
#else
  int init_device(int argc,char **argv);
#endif
int init_rf2072(void);
int transfer_video(void);
uint32_t deinit_device(void);
int udpout_init(char* udpaddr);

#endif

