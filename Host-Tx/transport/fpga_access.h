//#define TEST_BITSTREAM
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
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
#define HOST_BUFFER_SIZE						(128-1) // max data len-1
typedef int bool;  // match definition in usb_core.h

enum TYPE {
	CMD0, /*w/o params*/
	CMD1, /*w single param*/
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
	uint16_t  type;
	uint16_t len;
	uTag   tag;
	dAccess access;
	int active;
	bool echo;
} ipcLgdst;

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

#define USB_ATMEL_VER_VAL								0x20
#define USB_BOOT_APP_VAL								0xa4
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
#define USB_FPGA_NEW_VAL							0x24  // fpga app image cmd
#define USB_ATMEL_UPGRADE_VAL			0x23	// atmel upgrade cmd
#define USB_STREAM_OFF_VAL						0xa
#define USB_STREAM_ON_VAL						0xe
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
#define RADIO_CAL_IDX										0x1
#define RADIO_CAL_DONE_IDX 		 			0xc	// user query cmd index on cap value tuning process
#define RADIO_STATS_IDX								0x5
#define RADIO_STATS_LEN							sizeof(ctrl_radio_stats)
#define RADIO_HOPLESS_IDX							0xd
#define HOP_ID_LEN									10
// pairing operation IDs
#define RADIO_PAIRID_IDX							0xa
#define RADIO_PAIR_LOCKED_IDX 					    0xb
#define RADIO_PAIR_LOCKED_LEN		  			    4 /*boolean*/
//directional antenna selection
#define DRONE_GPS_IDX								0x10
#define DRONE_GPS_LEN								2*sizeof(float)
#define DRONE_YAW_IDX								0x11
#define DRONE_YAW_LEN								sizeof(float)
#define CAMERA_YAW_IDX								0x12
#define CAMERA_YAW_LEN								sizeof(float)
#define RADIO_ANT_QUERY_IDX							0x13

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

#define DBG_BOOTSTRAP_BYPASS
#define DBG_CTRL_PAIRING
#ifdef DBG_CTRL_PAIRING
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
#endif

#define RADIO_TEMP_IDX									0x8
#define RADIO_TEMP_LEN								sizeof(int16_t)
#define RADIO_CTUNE_IDX								0x9
#define RADIO_CTUNE_LEN							sizeof(uint8_t)

typedef struct Short_Statistics_S
{
    //Statistics parameters
    uint32_t IsDemodLocked;
    uint32_t InBandPwr;
    uint32_t BER;
    uint32_t SNR;
    uint32_t TotalTSPackets;
    uint32_t ErrorTSPackets;
} Short_Statistics_ST;

typedef struct TRANSMISSION_STATISTICS_S
{
	uint32_t Frequency;				//!< Frequency in Hz
	uint32_t Bandwidth;				//!< Bandwidth in MHz
	uint32_t TransmissionMode;		//!< FFT mode carriers in Kilos
	uint32_t GuardInterval;			//!< Guard Interval from SMSHOSTLIB_GUARD_INTERVALS_ET
	uint32_t CodeRate;				//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET
	uint32_t LPCodeRate;				//!< Low Priority Code Rate from SMSHOSTLIB_CODE_RATE_ET
	uint32_t Hierarchy;				//!< Hierarchy from SMSHOSTLIB_HIERARCHY_ET
	uint32_t Constellation;			//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET

	// DVB-H TPS parameters
	uint32_t CellId;					//!< TPS Cell ID in bits 15..0, bits 31..16 zero; if set to 0xFFFFFFFF cell_id not yet recovered
	uint32_t DvbhSrvIndHP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	uint32_t DvbhSrvIndLP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	uint32_t IsDemodLocked;			//!< 0 - not locked, 1 - locked
}TRANSMISSION_STATISTICS_ST;

typedef struct RECEPTION_STATISTICS_S
{
	uint32_t IsRfLocked;				//!< 0 - not locked, 1 - locked
	uint32_t IsDemodLocked;			//!< 0 - not locked, 1 - locked
	uint32_t IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	uint32_t ModemState;				//!< from SMSHOSTLIB_DVB_MODEM_STATE_ET
	int32_t  SNR;						//!< dB
	uint32_t BER;						//!< Post Viterbi BER [1E-5]
	uint32_t BERErrorCount;			//!< Number of erroneous SYNC bits.
	uint32_t BERBitCount;				//!< Total number of SYNC bits.
	uint32_t TS_PER;					//!< Transport stream PER, 0xFFFFFFFF indicate N/A
	uint32_t MFER;					//!< DVB-H frame error rate in percentage, 0xFFFFFFFF indicate N/A, valid only for DVB-H
	int32_t  RSSI;					//!< dBm
	int32_t  InBandPwr;				//!< In band power in dBM
	int32_t  CarrierOffset;			//!< Carrier Offset in bin/1024
	uint32_t ErrorTSPackets;			//!< Number of erroneous transport-stream packets
	uint32_t TotalTSPackets;			//!< Total number of transport-stream packets

	int32_t  RefDevPPM;
	int32_t  FreqDevHz;

	int32_t  MRC_SNR;					//!< dB //in non MRC application: maximum dvbt cnt
	int32_t  MRC_RSSI;				//!< dBm
	int32_t  MRC_InBandPwr;			//!< In band power in dBM, in Non MRC application: dvbt buffer max count. Should be less than 345*188

	uint32_t ErrorTSPacketsAfterReset;			//!< Number of erroneous transport-stream packets from the last reset
	uint32_t TotalTSPacketsAfterReset;			//!< Total number of transport-stream packets from the last reset
}RECEPTION_STATISTICS_ST;

 #include "ADRF6612_set.h"
 typedef union {
	 struct {
		 uint16_t chan_idx;
		 uint16_t pwr_att;
	 } params_tx;
	 struct {
		 dev_cfg *pregs_6612;
	 } params_rx;
 } rf_params;
 #define RF_TX_FREQ_VAL				0x13
 #define RF_TX_ATTN_VAL				0x14
