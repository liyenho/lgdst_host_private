//#define TEST_BITSTREAM
#define SRC_FRM_ENET
#define RADIO_SI4463
#define SI4463_CRYSTAL_32MHZ
#define RFFE_PARAMS
//#define PES_HDR_PROT
//#define PES_FRM_PROT
#define SHMKEY_TX 1234	 //tx shared memory key for IPC between lgdst/core
#define SHMKEY_RX 5678	 //rx shared memory key for IPC between lgdst/core
#define NON_NIOS
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
};

 #define CONF_BOARD_USB_RX
 #ifdef CONF_BOARD_USB_RX
  //#define MEDIA_ON_FLASH
 #endif

#define EXTRA															16  // randomly send more data on bulk pipe
//#define FWUP_DNLD_DBG							// don't turn in on unless it's necessary, not completely stable!
#define USB_SYSTEM_RESTART_VAL				0xa5
#define FW_UPGRADE_HDR_LEN					(12)
#define ATMEL_UPGRADE_HDR_LEN		(12-4) // starting address would be calc by atmel bootloader
#define USB_CPLD_UPGRADE_VAL				0x21  // cpld upgrade cmd
#define USB_FPGA_UPGRADE_VAL				0x22  // fpga upgrade cmd
#define USB_FPGA_NEW_VAL							0x24  // fpga app image cmd
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
#ifdef PES_FRM_PROT
	// to encode them into scarmbling control field (2 bits)
  #define PAT																		(0<<(2+4))
   #define PAT_PID														0x0
  #define PMT																	(1<<(2+4))
   #define PMT_PID														/*0x1*/ 0x100
  #define VIDEO																(2<<(2+4))
   #define VIDEO_PID													0x200 /*0x40*/
  #define AUDIO																(3<<(2+4))
   #define AUDIO_PID												0x201
  #ifdef REC
  	#define ERR_CHK_BUFFS									160	// in case of huge I frame size
  #endif
#endif

  #define USB_HOST_MSG_IDX						0x1	// data instead comm interface
  #define USB_HOST_MSG_LEN						sizeof(dev_access)
 #ifdef MEDIA_ON_FLASH     //must also disable SRC_FRM_ENET
  #define USB_LOAD_MEDIA									0xb
  #define USB_LOADM_IDX								0x1	// data instead comm interface
  #define USB_LOADM_LEN								sizeof(int)  // media file byte length
  //#define DEBUG_FL
 #endif
 	typedef struct {
 		uint8_t  addr;
 		uint16_t data;
 	} dev_cfg;

#ifdef RFFE_PARAMS
 typedef union {
	 struct {
		 uint16_t chan_idx;
		 uint16_t pwr_att;
	 } params_tx;
	 struct {
		 dev_cfg *pregs_6612;
	 } params_rx;
 } rf_params;
 #define RF_TX_VAL								0x11
 #define RF_TX_NIOS_DONE			0x16
 #define RF_RX_VAL								0x12
#endif
