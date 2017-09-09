#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
#define HOST_BUFFER_SIZE						(128-1) // max data len-1
typedef int bool;  // match definition in usb_core.h

#define LO_Frequency 							/*1583000*/ 1693000
	#define VID_CH_BW							/*6000*/10000
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

enum TYPE {
	CMD0, /*w/o params*/
	CMD1, /*w single param*/
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

#define USB_ATMEL_VER_VAL								0x20
#define USB_BOOT_APP_VAL								0xa4
#define USB_FPGA_DEF_VAL							0xa3
#define USB_STM_UPGRADE_VAL				0x21  // stm32 upgrade cmd
#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
#define USB_ATMEL_UPGRADE_VAL			0x23	// atmel upgrade cmd
#define USB_RX_TUNE_VAL						0xf
  #define USB_STREAM_IDX						0x1	// data instead comm interface
  #define USB_STREAM_LEN						0

// !Si446x status monitor parameters
#define CTRL_BITS				2
#define CTRL_CTX_LEN	(5-1)
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

#define RADIO_COMM_VAL								0x10
#define RADIO_STATS_IDX								0x5
#define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)
#define HOP_ID_LEN											10
// pairing operation IDs
#define RADIO_PAIRID_IDX							0xa
#define RADIO_PAIR_LOCKED_IDX 		    0xb
#define RADIO_PAIR_LOCKED_LEN		      4 /*boolean*/
//directional antenna selection
#define DRONE_GPS_IDX								0x10
#define DRONE_GPS_LEN								2*sizeof(float)
#define DRONE_YAW_IDX								0x11
#define DRONE_YAW_LEN								sizeof(float)
#define CAMERA_YAW_IDX								0x12
#define CAMERA_YAW_LEN								sizeof(float)
//Get RSSI Reading
#define RADIO_GET_RSSI_IDX		0x32

#define USB_HOST_MSG_TX_VAL						0x5
#define USB_HOST_MSG_RX_VAL						0x9
 #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
 #define USB_HOST_MSG_LEN						sizeof(dev_access)

#define DBG_BOOTSTRAP_BYPASS

#define RFFE_PARAMS
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
 #define RF_TX_FREQ_VAL				0x13
 #define RF_TX_ATTN_VAL				0x14
#endif