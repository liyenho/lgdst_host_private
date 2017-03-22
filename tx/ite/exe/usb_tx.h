//#define TEST_BITSTREAM
#define SRC_FRM_ENET
#define RADIO_SI4463
#define SI4463_CRYSTAL_32MHZ
#define RFFE_PARAMS
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
#define NON_NIOS

#define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define USB_RQ			0x04

typedef int bool;

#define HOST_BUFFER_SIZE						(256-1) // max data len-1
enum TYPE {
	CMD0,	/*w/o params*/
	CMD1,	/*w single param*/
	ACS, 	/*regs access*/
};
typedef struct {
	/*enum access_mode*/uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
	uint8_t data[1];
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

enum access_mode {
	RF2072_RESET=  0, // lastly added for rf2072 reset
	IT951X_READ = 1,
	IT951X_WRITE= 2,
	RF2072_READ=  3,
	RF2072_WRITE= 4,
	IT951X_DOWNLOAD = 5,
	TS_VID_ACTIVE = 6,
};

 #define CONF_BOARD_USB_RX
 #define CONFIG_RFFC_2072
 #ifdef CONF_BOARD_USB_RX
  //#define MEDIA_ON_FLASH
  #ifdef CONFIG_RFFC_2072
   #include "RFFC2072_set.h"
  #endif
  #include "error.h"
  #include "cmd.h"
 #endif

#define EXTRA															16  // randomly send more data on bulk pipe
//#define FWUP_DNLD_DBG							// don't turn in on unless it's necessary, not completely stable!
#define USB_SYSTEM_RESTART_VAL				0xa5
#define FW_UPGRADE_HDR_LEN					(12)
#define ATMEL_UPGRADE_HDR_LEN		(12-4) // starting address would be calc by atmel bootloader
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
//#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
//#define USB_FPGA_NEW_VAL							0x24  // fpga app image cmd
// these two are pertaining only to atmel operation
#define USB_FWM_BOOTUP_VAL					0xbe
#define USB_FWM_UPDATE_VAL					0xef
#define USB_STREAM_ON_VAL						0xe
#define USB_QUERY_IDX										0xff
#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
#ifdef  RADIO_SI4463
 //#define CTRL_RADIO_TEST
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

