//#define TEST_BITSTREAM
#define SRC_FRM_ENET
#define RADIO_SI4463
#define SI4463_CRYSTAL_32MHZ
#define RFFE_PARAMS
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
	// do not hard code unless it is spec not easy to change, null packet after each valid packet isn't that persistent, liyenho
#define STC_HALF_RATE	(90000/2)
#define TSTYPE   2/*0*/      //0 sn thinker stream
                         //1 yuneec dvb.ts type
                         //2 yuneec new camera type
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

typedef enum {
	RF2072_RESET=  0, // lastly added for rf2072 reset
	IT913X_READ = 1,
	IT913X_WRITE= 2,
	RF2072_READ=  3,
	RF2072_WRITE= 4,
	IT913X_DOWNLOAD = 5,
	TS_VID_ACTIVE = 6,
} access_mode;

 #define CONF_BOARD_USB_TX
 #define CONFIG_RFFC_2072
 #define SMS_DVBT2_DOWNLOAD //Kevin: Need comment out if SMS is broken
 #define RECV_SMS4470
#define DVBT  0
#define DVBT2  1
 #define MODE DVBT // start with dvbt first, liyenho
 //#define MODE DVBT2 // now try dvbt2 out, liyenho

#define USB_SYSTEM_RESTART_VAL				0xa5
#define FW_UPGRADE_HDR_LEN					(12)
#define ATMEL_UPGRADE_HDR_LEN		(12-4) // starting address would be calc by atmel bootloader
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
//#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
// these two are pertaining only to atmel operation
#define USB_FWM_BOOTUP_VAL					0xbe
#define USB_FWM_UPDATE_VAL					0xef
#define USB_STREAM_ON_VAL						0xe
#define USB_QUERY_IDX								0xff
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
#define RADIO_USR_RX_LEN		30 // ctl/sts radio payload byte length
#define RADIO_INFO_LEN  4 // gives usb pipe info
  #define CTRL_SEND_POLLPERIOD    /*30500*/ /*27000*/ 24000 //us: 160000 = 1.5kbps, 30000=8kbps
  #define CTRL_RECV_POLLPERIOD    /*70000*/ /*(30000/3)*/ /*(27000/3)*/ (24000/3) //us: must be > 2*CTRL_SEND_POLLPERIOD
#define CTRL_SEND_FIFODEPTH  8
#define CTRL_RECV_FIFODEPTH  8

#endif
//#ifdef CONF_BOARD_USB_RX
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
 #ifdef MEDIA_ON_FLASH	// this option should never be turn on...
  #define USB_LOAD_MEDIA									0xb
  #define USB_LOADM_IDX								0x1	// data instead comm interface
  #define USB_LOADM_LEN								sizeof(int)  // media file byte length
  //#define DEBUG_FL
 #endif
#ifdef CONF_BOARD_USB_TX
 #ifdef CONFIG_RFFC_2072
  #include "RFFC2072_set.h"
 #endif
 #include "error.h"
 #include "cmd.h"
#ifdef SMS_DVBT2_DOWNLOAD
  #define EXTRA												16  // randomly send more data on bulk pipe
  //#define SMS_FW_FNAME							"SMS4470_A2_DVBT2_MRC_Firmware_(2.0.0.89b).bin"
 #if (DVBT2==MODE)
  #define SMS_FW_FNAME							"SMS4470_A2_DVBT2_MRC_Firmware_(2.0.1.14).bin"
 #elif (DVBT==MODE)
  #define SMS_FW_FNAME							"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).bin"
 #endif
  //#define SMS_DATA_FNAME					"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.30).bin"
  #define SMS_DATA_FNAME					"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).bin"
  #define USB_SMS_FW_VAL						0x6
  #define SMS_FW_HDR_LEN					(12+4)
  #define USB_SMS_DATA_VAL					0x8
	//#define FWM_DNLD_DBG
	//#define DAT_DNLD_DBG
#endif
typedef enum
{
    SMSHOSTLIB_DEVMD_DVBT,
    SMSHOSTLIB_DEVMD_DVBH,
    SMSHOSTLIB_DEVMD_DAB_TDMB,
    SMSHOSTLIB_DEVMD_DAB_TDMB_DABIP,
    SMSHOSTLIB_DEVMD_DVBT_BDA,
    SMSHOSTLIB_DEVMD_ISDBT,
    SMSHOSTLIB_DEVMD_ISDBT_BDA,
    SMSHOSTLIB_DEVMD_CMMB,
    SMSHOSTLIB_DEVMD_RAW_TUNER,
    SMSHOSTLIB_DEVMD_FM_RADIO,
    SMSHOSTLIB_DEVMD_FM_RADIO_BDA,
    SMSHOSTLIB_DEVMD_ATSC,
    SMSHOSTLIB_DEVMD_ATV,
    SMSHOSTLIB_DEVMD_DVBT2,
    SMSHOSTLIB_DEVMD_DRM,
    SMSHOSTLIB_DEVMD_DVBT2_BDA,
    SMSHOSTLIB_DEVMD_MAX,
    SMSHOSTLIB_DEVMD_NONE = 0xFFFFFFFF
} SMSHOSTLIB_DEVICE_MODES_E;

#define Cmd_buildCommand(command, processor, chip)  (command + (uint16_t) (processor << 12) + (uint16_t) (chip << 12))
#endif

