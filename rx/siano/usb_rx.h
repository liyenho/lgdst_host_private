//#define TEST_BITSTREAM
#define SRC_FRM_ENET
#define RADIO_SI4463
#define SI4463_CRYSTAL_32MHZ
#define RFFE_PARAMS
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
	// do not hard code unless it is spec not easy to change, null packet after each valid packet isn't that persistent, liyenho
#define STC_HALF_RATE	(90000/2)  // divided by 2 because He'd want to keep stc value on even boundary
#define TSTYPE   2/*0*/      //0 sn thinker stream
                         //1 yuneec dvb.ts type
                         //2 yuneec new camera type
typedef int bool;

#define HOST_BUFFER_SIZE						(128-1) // max data len-1
enum TYPE {
	CMD0,	/*w/o params*/
	CMD1,	/*w single param*/
	ACS, 	/*regs access*/
};
typedef struct {
	/*enum access_mode*/uint16_t access;
	uint16_t dcnt;	// data count
	uint16_t addr;	// first access address, 12 bit
	uint16_t toflash; // request to burn to flash or not
	uint16_t data[1]; // only valid on lower half
} dev_access;
typedef struct {
	dev_access hdr;
	uint16_t data[HOST_BUFFER_SIZE];
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
/*
  bit 15-12: SPI Command defined as following
	  4'b0000: Read from SPI 8-bit Data Register
      4'b0010: Read from SPI 8-bit Data Register and
               Assert a Read Command from the same Address
      4'b0011: Read from SPI 8-bit Data Register and
               Assert a Read Command from (Address Register + 1)
      4'b0100: Read from SPI 12-bit Address Register

      4'b1000: Write to SPI 8-bit Data Register
      4'b1010: Write to SPI 8-bit Data Register and
               Assert a Write Command
      4'b1011: Write to SPI 8-bit Data Register and
               Assert a Write Command and then
               Increase Address Register by 1
      4'b1100: Write to SPI 12-bit Address Register
      4'b1101: Write to SPI 12-bit Address Register and
               Prepare data in SPI 8-bit Data Register
    bit 11-0: Data/Address Field (AD)
*/
enum access_mode {
	/*! \read data register current content */
	READ_CUR = 0,
	/*! \read addressed content from data register */
	READ_BY_ADDR,
	/*! \modify data register content */
	WRITE_CUR,
	/*! \modify addressed data content */
	WRITE_BY_ADDR,
	/*! \burst read at the same fifo address */
	BURST_FIFO_READ,
	/*! \burst write at the same fifo address */
	BURST_FIFO_WRITE,
	/*! \burst read at contiguous memory address */
	BURST_MEM_READ,
	/*! \burst write at contiguous memory address */
	BURST_MEM_WRITE,
	/*! \dvbt/2 signal detected from sms4470 @ rec */
	SIGNAL_DETECTED,
	/*! \dvbt/2 signal status from sms4470 @ rec */
	SIGNAL_STATUS,
	/*! \transmission status from sms4470 @ rec */
	TRANS_STATUS,
	/*! \reciever status from sms4470 @ rec */
	RECV_STATUS,
	/*! \locked status from sms4470 @ rec */
	LOCKED_STATUS,
};

 #define CONF_BOARD_USB_TX
 #define CONFIG_ADI_6612
 #define RF_RX_VAL												0x12
 #define SMS_DVBT2_DOWNLOAD //Kevin: Need comment out if SMS is broken
 #define RECV_SMS4470
#define DVBT  0
#define DVBT2  1
 #define MODE DVBT // start with dvbt first, liyenho

#define USB_SYSTEM_RESTART_VAL				0xa5
#define FW_UPGRADE_HDR_LEN					(12)
#define ATMEL_UPGRADE_HDR_LEN		(12-4) // starting address would be calc by atmel bootloader
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
// these two cmds  are for starting or stopping TS record
#define USB_SAVE_TS_VAL									0xf1
#define USB_UNSAVE_TS_VAL							0x1f
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
 #ifdef CONFIG_ADI_6612
  #include "ADRF6612_set.h"
 #endif
#ifdef SMS_DVBT2_DOWNLOAD
  #define EXTRA												16  // randomly send more data on bulk pipe
  //#define SMS_FW_FNAME							"SMS4470_A2_DVBT2_MRC_Firmware_(2.0.0.89b).h"
 #if (DVBT2==MODE)
  #define SMS_FW_FNAME							"SMS4470_A2_DVBT2_MRC_Firmware_(2.0.1.14).bin"
 #elif (DVBT==MODE)
  #define SMS_FW_FNAME							"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).bin"
 #endif
  //#define SMS_DATA_FNAME					"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.30).h"
  #define SMS_DATA_FNAME					"SMS4470_A2_DVBT_MRC_Firmware_(2.0.0.47).h"
  #define USB_SMS_FW_VAL						0x6
  #define SMS_FW_HDR_LEN					(12+4)
  #define USB_SMS_DATA_VAL					0x8
	//#define FWM_DNLD_DBG
	//#define DAT_DNLD_DBG
/* sms4470 comm header */
 typedef struct SmsMsgHdr_S
 {
	uint16_t 	msgType;
	uint8_t	 msgSrcId;
	uint8_t	 msgDstId;
	uint16_t	msgLength;	// Length is of the entire message, including header
	uint16_t	msgFlags;
 } /*SmsMsgHdr_ST*/sms_access;
  #define USB_SMS_MSG_IDX						0x0
  #define USB_SMS_MSG_LEN						sizeof(sms_access)
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
#endif
#endif

