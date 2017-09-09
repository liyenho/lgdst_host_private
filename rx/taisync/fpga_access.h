//#define TEST_BITSTREAM
#define SHMKEY_TX 								1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 								5678	 //rx shared memory key for IPC between lgdst/core
#define HOST_BUFFER_SIZE						(256-1) // max data len-1
typedef int bool;  // match definition in usb_core.h

enum TYPE {
	CMD0, /*w/o params*/
	CMD1, /*w single param*/
	ACS, 	/*regs access, only*/
};
typedef struct {
	uint8_t access;
	uint8_t dcnt;	// data count
	uint16_t addr;	// first access address
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
	uint16_t  type;
	uint16_t len;
	uTag   tag;
	dAccess access;
	int active;
	bool echo;
} ipcLgdst;

typedef enum {
	FIVE_MHZ,
	TEN_MHZ,
	TWENTY_MHZ,
	FORTY_MHZ,
} VCH_BW;

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

#define USB_ATMEL_VER_VAL								0x20
#define USB_BOOT_APP_VAL								0xa4
#define USB_STM_UPGRADE_VAL					0x21  // cpld upgrade cmd
#define USB_FPGA_UPGRADE_VAL					0x22  	// fpga upgrade cmd
#define USB_STREAM_ON_VAL								0xe
#define USB_STREAM_IDX									0x1	// data instead comm interface
#define USB_STREAM_LEN									0

// !range status monitor parameters
#define CTRL_BITS										2
#define CTRL_CTX_LEN									(5-1)
typedef enum {
	NEUTRAL = 0,
	LONG_RNG = 1,
	SHORT_RNG = 2,
	RESERVED = 3,
} BW_CTRL;

typedef struct {
	unsigned int bw_ctrl_bits;
	unsigned int ctrl_bits_ctx[CTRL_CTX_LEN];
	unsigned int errPerAcc; // checksum error accum
	unsigned int loop_cnt; // monitor pckt recv period
} ctrl_radio_stats;

#define RADIO_COMM_VAL									0x10
#define RADIO_STATS_IDX									0x5
#define RADIO_STATS_LEN									sizeof(ctrl_radio_stats)
#define HOP_ID_LEN										10
// pairing operation IDs
#define RADIO_PAIRID_IDX								0xa
#define RADIO_PAIR_LOCKED_IDX 							0xb
#define RADIO_PAIR_LOCKED_LEN							4 /*boolean*/
//Set Base Station GPS
#define BASE_GPS_IDX 									0xe
#define BASE_GPS_LEN									2*sizeof(float)


//Get RSSI Reading
#define RADIO_GET_RSSI_IDX								0x32
#define RADIO_SET_PWR_LVL_IDX							0x33
#define RADIO_SET_PWR_LVL_LEN							1

//command to enable/disable error correction on sent radio packets
#define SET_FEC_IDX										0x34
#define SET_FEC_LEN										sizeof(uint8_t)

#define RADIO_SET_REGIME_915_IDX						0x40
#define RADIO_SET_REGIME_869_IDX						0x41

#define USB_HOST_MSG_TX_VAL								0x5

#define USB_HOST_MSG_RX_VAL								0x9
  #define USB_HOST_MSG_IDX								0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN								sizeof(dev_access)


//Commands
#define pairID 		"pair-id"
#define pairLocked  "pair-locked"
#define locGPS 		"loc-gps"
#define RSSI 		"RSSI"
#define setFEC		"setFEC"
#define setCtrlPwr  "setCtrlPwr"
#define use915		"use915"
#define use869		"use869"

