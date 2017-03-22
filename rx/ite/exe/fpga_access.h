//#define TEST_BITSTREAM
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
#define HOST_BUFFER_SIZE						(256-1) // max data len-1
typedef int bool;  // match definition in usb_core.h

enum TYPE {
	CMD0, /*w/o params*/
	CMD1, /*w single param*/
	ACS, 	/*regs access*/
};
typedef struct {
	/*enum access_mode*/uint8_t access;
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

#define USB_ATMEL_VER_VAL								0x20
#define USB_BOOT_APP_VAL								0xa4
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
#define USB_ATMEL_UPGRADE_VAL			0x23	// atmel upgrade cmd
#define USB_STREAM_OFF_VAL						0xa
#define USB_STREAM_ON_VAL						0xe
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
#define RADIO_CAL_IDX										0x1
#define RADIO_CAL_DONE_IDX 		 			0xc	// user query cmd index on cap value tuning process
#define RADIO_STATS_IDX								0x5
#define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)
#define RADIO_HOPLESS_IDX						0xd
#define HOP_ID_LEN											10
// pairing operation IDs
#define RADIO_PAIRID_IDX							0xa
#define RADIO_PAIR_LOCKED_IDX 		0xb
#define RADIO_PAIR_LOCKED_LEN		4 /*boolean*/
//
#define BASE_GPS_IDX 				0xe
#define BASE_GPS_LEN				2*sizeof(float)

typedef struct  {
	uint8_t  /*MODEM_PEND*/RSSI_COMP; /*compensation offset*/
	uint8_t  MODEM_STATUS;
	uint8_t  CURR_RSSI;
	uint8_t  LATCH_RSSI;
	uint8_t  ANT1_RSSI;
	uint8_t  ANT2_RSSI;
	uint16_t  AFC_FREQ_OFFSET;
} si446x_get_modem_status;

#define RADIO_MODEM_IDX								0x6
#define RADIO_MODEM_LEN							sizeof(si446x_get_modem_status)

 #define USB_HOST_MSG_TX_VAL						0x5

#define USB_HOST_MSG_RX_VAL						0x9
  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)

// !Si446x RF Channel definitions in 25.5 mhz range divided into 14 bands of 3.43 mhz wide,
// each band contains 50 hopping channels in 70 khz separation, all calculations are based
// upon 32 mhz reference crystal
const uint8_t RF_FREQ_CONTROL_INTE_BAND0_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0B, 0x24, 0x7A, 0x08, 0xF6}; // 902.285 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND0_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND1_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0x04, 0x7A, 0x08, 0xF6}; // 904.035 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND1_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND2_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0C, 0xE4, 0x7A, 0x08, 0xF6}; // 905.785 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND2_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND3_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0D, 0xC4, 0x7A, 0x08, 0xF6}; // 907.535 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND3_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND4_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0E, 0xA4, 0x7A, 0x08, 0xF6}; // 909.285 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND4_3[] = {0x11, 0x20, 0x03, 0x30,  0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND5_6[] = {0x11, 0x40, 0x06, 0x00, 0x37, 0x0F, 0x84, 0x7A, 0x08, 0xF6}; // 911.035 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND5_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND6_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x08, 0x64, 0x7A, 0x08, 0xF6}; // 912.785 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND6_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND7_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x09, 0x44, 0x7A, 0x08, 0xF6}; // 914.535 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND7_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND8_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0A, 0x24, 0x7A, 0x08, 0xF6}; // 916.250 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND8_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND9_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0x04, 0x7A, 0x08, 0xF6}; // 918.035 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND9_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND10_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0B, 0xE4, 0x7A, 0x08, 0xF6}; // 919.785 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND10_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND11_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0C, 0xC4, 0x7A, 0x08, 0xF6}; // 921.535 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND11_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND12_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0D, 0xA4, 0x7A, 0x08, 0xF6}; // 923.285 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND12_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};
const uint8_t RF_FREQ_CONTROL_INTE_BAND13_6[] = {0x11, 0x40, 0x06, 0x00, 0x38, 0x0E, 0x84, 0x7A, 0x08, 0xF6}; // 925.035 mhz
  const uint8_t RF_MODEM_AFC_LIMITER_BAND13_3[] = {0x11, 0x20, 0x03, 0x30, 0x25, 0x16, 0xC0};

// tabulate all avaialble ctrl rf bands
const uint8_t *chtbl_ctrl_rdo[/*14*2*/] = {
	RF_FREQ_CONTROL_INTE_BAND0_6,
	 RF_MODEM_AFC_LIMITER_BAND0_3,
	RF_FREQ_CONTROL_INTE_BAND1_6,
	 RF_MODEM_AFC_LIMITER_BAND1_3,
	RF_FREQ_CONTROL_INTE_BAND2_6,
	 RF_MODEM_AFC_LIMITER_BAND2_3,
	RF_FREQ_CONTROL_INTE_BAND3_6,
	 RF_MODEM_AFC_LIMITER_BAND3_3,
	RF_FREQ_CONTROL_INTE_BAND4_6,
	 RF_MODEM_AFC_LIMITER_BAND4_3,
	RF_FREQ_CONTROL_INTE_BAND5_6,
	 RF_MODEM_AFC_LIMITER_BAND5_3,
	RF_FREQ_CONTROL_INTE_BAND6_6,
	 RF_MODEM_AFC_LIMITER_BAND6_3,
	RF_FREQ_CONTROL_INTE_BAND7_6,
	 RF_MODEM_AFC_LIMITER_BAND7_3,
	RF_FREQ_CONTROL_INTE_BAND8_6,
	 RF_MODEM_AFC_LIMITER_BAND8_3,
	RF_FREQ_CONTROL_INTE_BAND9_6,
	 RF_MODEM_AFC_LIMITER_BAND9_3,
	RF_FREQ_CONTROL_INTE_BAND10_6,
	 RF_MODEM_AFC_LIMITER_BAND10_3,
	RF_FREQ_CONTROL_INTE_BAND11_6,
	 RF_MODEM_AFC_LIMITER_BAND11_3,
	RF_FREQ_CONTROL_INTE_BAND12_6,
	 RF_MODEM_AFC_LIMITER_BAND12_3,
	RF_FREQ_CONTROL_INTE_BAND13_6,
	 RF_MODEM_AFC_LIMITER_BAND13_3,
} ;

#define RADIO_CHSEL_IDX								0x7
#define RADIO_CHSEL_LEN0							sizeof(RF_FREQ_CONTROL_INTE_BAND0_6)
#define RADIO_CHSEL_LEN1							sizeof(RF_MODEM_AFC_LIMITER_BAND0_3)

const uint8_t *chtbl_ctrl_len[/*2*/] = {
	RADIO_CHSEL_LEN0,
	RADIO_CHSEL_LEN1,
} ;

#define RADIO_TEMP_IDX									0x8
#define RADIO_TEMP_LEN								sizeof(int16_t)
#define RADIO_CTUNE_IDX								0x9
#define RADIO_CTUNE_LEN							sizeof(uint8_t)

 #include "RFFC2072_set.h"
