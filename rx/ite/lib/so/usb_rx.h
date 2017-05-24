#ifndef __USB_RX_H__
#define __USB_RX_H__

#include "type.h"
#include "rf2072_set.h"
typedef int bool;

#define RADIO_SI4463
#define SHMKEY_TX 								1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 								5678	 //rx shared memory key for IPC between lgdst/core
#define STC_HALF_RATE							(90000/2)

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

#define FRAME_SIZE_A							1880
#define FRAME_SIZE_V							307200
#define FRAME_SIZE_V2							(FRAME_SIZE_V*2)
#define ITERS									(FILE_LEN/FRAME_SIZE_A)
#define FRAME_BUFFS								5
#define TIMEOUT									1000
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
	uint8_t access;
	uint8_t dcnt;	  // data count
	uint16_t addr;	  // first access address
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
	int active;
	bool echo;
	uint16_t  type;
	uint16_t len;
	uTag   tag;
	dAccess access;
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

#define RADIO_COMM_VAL							0x10    // using 5 bit out of 16 bit should be alright?
#define RADIO_STARTUP_IDX						0x2
#define RADIO_DATA_TX_IDX 						0x3
#define RADIO_DATA_RX_IDX						0x4
#define RADIO_USR_TX_LEN						30 		// ctl/sts radio payload byte length
#define RADIO_USR_RX_LEN						30 		// ctl/sts radio payload byte length
#define RADIO_INFO_LEN  						4 		// gives usb pipe info
#define CTRL_SEND_POLLPERIOD    				24000 	//us: 160000 = 1.5kbps, 30000=8kbps
#define CTRL_RECV_POLLPERIOD     				(24000/3) //us: must be > 2*CTRL_SEND_POLLPERIOD
#define CTRL_SEND_FIFODEPTH  				 	8
#define CTRL_RECV_FIFODEPTH  				 	8

#endif

#define USB_HOST_MSG_IDX						0x1		// data instead comm interface
#define USB_HOST_MSG_LEN						sizeof(dev_access)
#define EXTRA									16  	// randomly send more data on bulk pipe
#define USB_ITE_FW_VAL							0x6
#define ITE_FW_HDR_LEN							(4)
#define LO_Frequency 							/*1583000*/ 1686000


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

