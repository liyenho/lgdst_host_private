#ifndef __MODULATOR_TYPE_H__
#define __MODULATOR_TYPE_H__

/* define NULL, true, flase */

//#include <linux/stddef.h>

/* define bool */

//#include <linux/types.h>

/* include Linux delay */

//#include <linux/delay.h>


#define EagleUser_INTERNAL	 				1
#define IT9510User_INTERNAL					1
#define IT9133Rx							1
#define IT9510_MAX_BIT  

#define Tuner_Afatech_OMEGA                 0x38
#define Tuner_Afatech_OMEGA_LNA_Config_1	0x51 // 0x50~0x5f reserved for OMEGA use
#define Tuner_Afatech_OMEGA_LNA_Config_2    0x52
#define Tuner_Afatech_OMEGA_V2              0x60 // 0x60~0x6f reserved for OMEGA V2 use
#define Tuner_Afatech_OMEGA_V2_LNA_Config_1	0x61 
#define Tuner_Afatech_OMEGA_V2_LNA_Config_2 0x62
#define Tuner_Afatech_AF9007                0xFF

#define Eagle_MAX_BIT               		8
#define IQ_TABLE_NROW 						92
#define Command_REG_DEMOD_READ          	0x0000
#define Command_REG_DEMOD_WRITE         	0x0001
#define Command_REG_EEPROM_READ         	0x0004
#define Command_REG_EEPROM_WRITE        	0x0005
#define Command_IR_GET                  	0x0018
#define Command_QUERYINFO               	0x0022
#define Command_BOOT                    	0x0023
#define Command_REBOOT                  	0x0023
#define Command_SCATTER_WRITE           	0x0029
#define Command_GENERIC_READ            	0x002A
#define Command_GENERIC_WRITE           	0x002B
//#define NULL   								0

/**
 * The type of handle.
 */
typedef void* Handle;


/**
 * The type defination of 8-bits unsigned type.
 */
//typedef unsigned char uint8_t;


/**
 * The type defination of 16-bits unsigned type.
 */
//typedef unsigned short uint16_t;


/**
 * The type defination of 32-bits unsigned type.
 */
//typedef unsigned long uint32_t;


/**
 * The type defination of 16-bits signed type.
 */
typedef short Short;


/**
 * The type defination of 32-bits signed type.
 */
typedef long Long;


/**
 * The type defination of ValueSet.
 */
typedef struct {
	uint32_t frequency;      /**  */
	int  	 dAmp;			  /**  */
	int  	 dPhi;	
} IQtable;

typedef struct {
	IQtable *ptrIQtableEx;
	uint16_t	tableGroups;		//Number of IQtable groups;
	uint32_t	tableVersion;		//tableVersion;
	int			outputGain;	
	uint16_t	c1DefaultValue;
	uint16_t	c2DefaultValue;
	uint16_t	c3DefaultValue;
} CalibrationInfo;

typedef struct {
	uint32_t startFrequency;      /**  */
	int  i;			  /**  */
	int  q;	
} DCtable;

typedef struct {
	DCtable *ptrDCtable;
	DCtable *ptrOFStable;
	uint16_t tableGroups;		//Number of IQtable groups;	
} DCInfo;
/**
 * The type defination of Bool
 */

typedef enum {
	False = 0,
	True = 1
} Bool;

//typedef enum {False=0, True} bool;


typedef struct {
	uint8_t segmentType;           /** 0:Firmware download 1:Rom copy 2:Direct command */
	uint32_t segmentLength;
} Segment;


/**
 * The type defination of Bandwidth.
 */
typedef enum {
	Bandwidth_6M = 0,           /** Signal bandwidth is 6MHz */
	Bandwidth_7M,               /** Signal bandwidth is 7MHz */
	Bandwidth_8M,               /** Signal bandwidth is 8MHz */
	Bandwidth_5M,               /** Signal bandwidth is 5MHz */
	Bandwidth_4M = 40,          /** Signal bandwidth is 4MHz */
	Bandwidth_3M = 30,          /** Signal bandwidth is 3MHz */
	Bandwidth_2_5M = 25 ,       /** Signal bandwidth is 2.5MHz */
	Bandwidth_2M = 20,          /** Signal bandwidth is 2MHz */
	Bandwidth_1_5M = 15,        /** Signal bandwidth is 1.5MHz */
	Bandwidth_1M = 10,          /** Signal bandwidth is 1MHz */
	Bandwidth_UNKNOW = 0xFF			/** Signal bandwidth is UNKNOW */
} Bandwidth;

/**
 * The type defination of TransmissionMode.
 */
typedef enum {
	TransmissionMode_2K = 0,    /** OFDM frame consists of 2048 different carriers (2K FFT mode, ISDBT Mode 1) */
	TransmissionMode_8K = 1,    /** OFDM frame consists of 8192 different carriers (8K FFT mode, ISDBT Mode 3) */
	TransmissionMode_4K = 2     /** OFDM frame consists of 4096 different carriers (4K FFT mode, ISDBT Mode 2) */
} TransmissionModes;


/**
 * The type defination of Interval.
 */
typedef enum {
	Interval_1_OVER_32 = 0,     /** Guard interval is 1/32 of symbol length */
	Interval_1_OVER_16,         /** Guard interval is 1/16 of symbol length */
	Interval_1_OVER_8,          /** Guard interval is 1/8 of symbol length  */
	Interval_1_OVER_4           /** Guard interval is 1/4 of symbol length  */
} Interval;


/**
 * The type defination of Priority.
 */
typedef enum {
	Priority_HIGH = 0,          /** DVB-T and DVB-H - identifies high-priority stream */
	Priority_LOW                /** DVB-T and DVB-H - identifies low-priority stream  */
} Priority;


/**
 * The type defination of CodeRate.
 */
typedef enum {
	CodeRate_1_OVER_2 = 0,      /** Signal uses FEC coding ratio of 1/2 */
	CodeRate_2_OVER_3,          /** Signal uses FEC coding ratio of 2/3 */
	CodeRate_3_OVER_4,          /** Signal uses FEC coding ratio of 3/4 */
	CodeRate_5_OVER_6,          /** Signal uses FEC coding ratio of 5/6 */
	CodeRate_7_OVER_8,          /** Signal uses FEC coding ratio of 7/8 */
	CodeRate_NONE               /** None, NXT doesn't have this one     */
} CodeRate;


/**
 * TPS Hierarchy and Alpha value.
 */
typedef enum {
	Hierarchy_NONE = 0,         /** Signal is non-hierarchical        */
	Hierarchy_ALPHA_1,          /** Signalling format uses alpha of 1 */
	Hierarchy_ALPHA_2,          /** Signalling format uses alpha of 2 */
	Hierarchy_ALPHA_4           /** Signalling format uses alpha of 4 */
} Hierarchy;

/**
 * The defination of SubchannelType.
 */
typedef enum {
	SubchannelType_AUDIO = 0,           /** Signal in subchannel is audio format          */
	SubchannelType_VIDEO = 1,           /** Signal in subchannel is video format          */
	SubchannelType_PACKET = 3,          /** Signal in subchannel is packet format         */
	SubchannelType_ENHANCEPACKET = 4    /** Signal in subchannel is enhance packet format */
} SubchannelType;


/**
 * The defination of ProtectionLevel.
 */
typedef enum {
	ProtectionLevel_NONE = 0x00,    /** The protection level of subchannel is none     */
	ProtectionLevel_PL1 = 0x01,     /** The protection level of subchannel is level 1  */
	ProtectionLevel_PL2 = 0x02,     /** The protection level of subchannel is level 2  */
	ProtectionLevel_PL3 = 0x03,     /** The protection level of subchannel is level 3  */
	ProtectionLevel_PL4 = 0x04,     /** The protection level of subchannel is level 4  */
	ProtectionLevel_PL5 = 0x05,     /** The protection level of subchannel is level 5  */
	ProtectionLevel_PL1A = 0x1A,    /** The protection level of subchannel is level 1A */
	ProtectionLevel_PL2A = 0x2A,    /** The protection level of subchannel is level 2A */
	ProtectionLevel_PL3A = 0x3A,    /** The protection level of subchannel is level 3A */
	ProtectionLevel_PL4A = 0x4A,    /** The protection level of subchannel is level 4A */
	ProtectionLevel_PL1B = 0x1B,    /** The protection level of subchannel is level 1B */
	ProtectionLevel_PL2B = 0x2B,    /** The protection level of subchannel is level 2B */
	ProtectionLevel_PL3B = 0x3B,    /** The protection level of subchannel is level 3B */
	ProtectionLevel_PL4B = 0x4B     /** The protection level of subchannel is level 4B */
} ProtectionLevel;


/**
 * The defination of SubchannelModulation. This structure is used to
 * represent subchannel modulation when device is operate in T-DMB/DAB mode.
 *
 */
typedef struct {
	uint8_t subchannelId;                  /** The ID of subchannel.                                                 */
	uint16_t subchannelSize;                /** The size of subchannel.                                               */
	uint16_t bitRate;                       /** The bit rate of subchannel.                                           */
	uint8_t transmissionMode;              /** The transmission mode of subchannel, possible values are: 1, 2, 3, 4. */
	ProtectionLevel protectionLevel;    /** The protection level of subchannel.                                   */
	SubchannelType subchannelType;      /** The type of subchannel                                                */
	uint8_t conditionalAccess;             /** If a conditional access exist                                         */
	uint8_t tiiPrimary;                    /** TII primary                                                           */
	uint8_t tiiCombination;                /** TII combination                                                       */
} SubchannelModulation;


/**
 * The type defination of IpVersion.
 */
typedef enum {
	IpVersion_IPV4 = 0,         /** The IP version if IPv4 */
	IpVersion_IPV6 = 1          /** The IP version if IPv6 */
} IpVersion;

/**
 * The type defination of Ip.
 */
typedef struct {
	IpVersion version;          /** The version of IP. See the defination of IpVersion.                                               */
	Priority  priority;          /** The priority of IP. See the defination of Priority.                                               */
	Bool      cache;                 /** True: IP datagram will be cached in device's buffer. Fasle: IP datagram will be transfer to host. */
	uint8_t   address[16];           /** The byte array to store IP address.                                                               */
} Ip;


/**
 * The type defination of Platform.
 * Mostly used is in DVB-H standard
 */
typedef struct {
	uint32_t platformId;           /** The ID of platform.                                    */
	char iso639LanguageCode[3]; /** The ISO 639 language code for platform name.           */
	uint8_t platformNameLength;    /** The length of platform name.                           */
	char platformName[32];      /** The char array to store platform name.                 */
	uint16_t bandwidth;             /** The operating channel bandwith of this platform.       */
	uint32_t frequency;            /** The operating channel frequency of this platform.      */
	uint8_t* information;          /** The extra information about this platform.             */
	uint16_t informationLength;     /** The length of information.                             */
	Bool hasInformation;        /** The flag to indicate if there exist extra information. */
	IpVersion ipVersion;        /** The IP version of this platform.                       */
} Platform;


/**
 * The type defination of Label.
 */
typedef struct {
	uint8_t charSet;
	uint16_t charFlag;
	uint8_t string[16];
} Label;


/**
 * The type defination of Ensemble.
 */
typedef struct {
	uint16_t ensembleId;
	Label ensembleLabel;
	uint8_t totalServices;
} Ensemble;


/**
 * The type defination of Service.
 * Mostly used is in T-DMB standard
 */
typedef struct {
	uint8_t serviceType;       /** Service Type(P/D): 0x00: Program, 0x80: Data */
	uint32_t serviceId;
	uint32_t frequency;
	Label serviceLabel;
	uint8_t totalComponents;
} Service;


/**
 * The type defination of Service Component.
 */
typedef struct {
	uint8_t serviceType;           /** Service Type(P/D): 0x00: Program, 0x80: Data         */
	uint32_t serviceId;            /** Service ID                                           */
	uint16_t componentId;           /** Stream audio/data is subchid, packet mode is SCId    */
	uint8_t componentIdService;    /** Component ID within Service                          */
	Label componentLabel;       /** The label of component. See the defination of Label. */
	uint8_t language;              /** Language code                                        */
	uint8_t primary;               /** Primary/Secondary                                    */
	uint8_t conditionalAccess;     /** Conditional Access flag                              */
	uint8_t componentType;         /** Component Type (A/D)                                 */
	uint8_t transmissionId;        /** Transmission Mechanism ID                            */
} Component;

/**
 * The type defination of Target.
 */
typedef enum {
	SectionType_MPE = 0,        /** Stands for MPE data.                                         */
	SectionType_SIPSI,          /** Stands for SI/PSI table, but don't have to specify table ID. */
	SectionType_TABLE           /** Stands for SI/PSI table.                                     */
} SectionType;


/**
 * The type defination of FrameRow.
 */
typedef enum {
	FrameRow_256 = 0,           /** There should be 256 rows for each column in MPE-FEC frame.  */
	FrameRow_512,               /** There should be 512 rows for each column in MPE-FEC frame.  */
	FrameRow_768,               /** There should be 768 rows for each column in MPE-FEC frame.  */
	FrameRow_1024               /** There should be 1024 rows for each column in MPE-FEC frame. */
} FrameRow;


/**
 * The type defination of Pid.
 *
 * In DVB-T mode, only value is valid. In DVB-H mode,
 * as sectionType = SectionType_SIPSI: only value is valid.
 * as sectionType = SectionType_TABLE: both value and table is valid.
 * as sectionType = SectionType_MPE: except table all other fields is valid.
 */
typedef struct {
	uint8_t table;                 /** The table ID. Which is used to filter specific SI/PSI table.                                  */
	uint8_t duration;              /** The maximum burst duration. It can be specify to 0xFF if user don't know the exact value.     */
	FrameRow frameRow;          /** The frame row of MPE-FEC. It means the exact number of rows for each column in MPE-FEC frame. */
	SectionType sectionType;    /** The section type of pid. See the defination of SectionType.                                   */
	Priority priority;          /** The priority of MPE data. Only valid when sectionType is set to SectionType_MPE.              */
	IpVersion version;          /** The IP version of MPE data. Only valid when sectionType is set to SectionType_MPE.            */
	Bool cache;                 /** True: MPE data will be cached in device's buffer. Fasle: MPE will be transfer to host.        */
	uint16_t value;                 /** The 13 bits Packet ID.                                                                        */
} Pid;


/**
 * The type defination of ValueSet.
 */
typedef struct {
	uint32_t address;      /** The address of target register */
	uint8_t value;         /** The value of target register   */
} ValueSet;

/**
 * The type defination of tuner group.
 */
typedef struct {
	uint16_t tunerId;       /** The id of tuner */
	uint8_t groupIndex;    /** The index of group */
} TunerGroup;


/**
 * The type defination of Datetime.
 */
typedef struct {
	uint32_t mjd;              /** The mjd of datetime           */
	uint8_t configuration;     /** The configuration of datetime */
	uint8_t hours;             /** The hours of datetime         */
	uint8_t minutes;           /** The minutes of datetime       */
	uint8_t seconds;           /** The seconds of datetime       */
	uint16_t milliseconds;      /** The milli seconds of datetime */
} Datetime;


/**
 * The type defination of Interrupt.
 */
typedef enum {
	Interrupt_NONE      = 0x0000,   /** No interrupt. */
	Interrupt_SIPSI     = 0x0001,
	Interrupt_DVBH      = 0x0002,
	Interrupt_DVBT      = 0x0004,
	Interrupt_PLATFORM  = 0x0008,
	Interrupt_VERSION   = 0x0010,
	Interrupt_FREQUENCY = 0x0020,
	Interrupt_SOFTWARE1 = 0x0040,
	Interrupt_SOFTWARE2 = 0x0080,
	Interrupt_FIC       = 0x0100,
	Interrupt_MSC       = 0x0200,
	Interrupt_MCISI     = 0x0400
} Interrupt;


/**
 * The type defination of Multiplier.
 */
typedef enum {
	Multiplier_1X = 0,
	Multiplier_2X
} Multiplier;


/**
 * The type defination of StreamType.
 */
typedef enum {
	StreamType_NONE = 0,            /** Invalid (Null) StreamType                */
	StreamType_DVBT_DATAGRAM = 3,   /** DVB-T mode, store data in device buffer  */
	StreamType_DVBT_PARALLEL,       /** DVB-T mode, output via paralle interface */
	StreamType_DVBT_SERIAL,         /** DVB-T mode, output via serial interface  */
} StreamType;

/**
 * The type defination of StreamType.
 */
typedef enum {
	INTERFACE_UNUSE = 0, 
	PARALLEL_TS_INPUT=4,		/** input via paralle interface */
	SERIAL_TS_INPUT,			/** input via serial interface  */
} TsInterface;

/**
 * The type defination of StreamType.
 */
typedef enum {
	Architecture_NONE = 0,      /** Inavalid (Null) Architecture.                                    */
	Architecture_DCA,           /** Diversity combine architecture. Only valid when chip number > 1. */
	Architecture_PIP            /** Picture in picture. Only valid when chip number > 1.             */
} Architecture;

/**
 * The type defination of ClockTable.
 */
typedef struct {
	uint32_t crystalFrequency;     /** The frequency of crystal. */
	uint32_t adcFrequency;         /** The frequency of ADC.     */
} ClockTable;


/**
 * The type defination of BandTable.
 */
typedef struct {
	uint32_t minimum;          /** The minimum frequency of this band */
	uint32_t maximum;          /** The maximum frequency of this band */
} BandTable;


/**
 * The type defination of MeanTable.
 */
typedef struct {
	uint32_t mean;
	uint32_t errorCount;
} MeanTable;


/**
 * The type defination of Polarity.
 */
typedef enum {
	Polarity_NORMAL = 0,
	Polarity_INVERSE
} Polarity;


/**
 * The type defination of Processor.
 */
typedef enum {
	Processor_LINK = 0,
	Processor_OFDM = 8
} Processor;



/**
 * The type defination of Constellation.
 */
typedef enum {

	Constellation_QPSK = 0,     /** Signal uses QPSK constellation  */
	Constellation_16QAM,        /** Signal uses 16QAM constellation */
	Constellation_64QAM         /** Signal uses 64QAM constellation */
} Constellation;

/**
 * The defination of ChannelInformation.
 */
typedef struct {
	uint32_t frequency;                    /** Channel frequency in KHz.                                */
	TransmissionModes transmissionMode; /** Number of carriers used for OFDM signal                  */
	Constellation constellation;        /** Constellation scheme (FFT mode) in use                   */
	Interval interval;                  /** Fraction of symbol length used as guard (Guard Interval) */
	Priority priority;                  /** The priority of stream                                   */
	CodeRate highCodeRate;              /** FEC coding ratio of high-priority stream                 */
	CodeRate lowCodeRate;               /** FEC coding ratio of low-priority stream                  */
	Hierarchy hierarchy;                /** Hierarchy levels of OFDM signal                          */
	Bandwidth bandwidth;
} ChannelModulation;


/**
 * The type defination of Statistic.
 */
typedef struct {
	Bool signalPresented;       /** Signal is presented.                                                                         */
	Bool signalLocked;          /** Signal is locked.                                                                            */
	uint8_t signalQuality;         /** Signal quality, from 0 (poor) to 100 (good).                                                 */
	uint8_t signalStrength;        /** Signal strength from 0 (weak) to 100 (strong).                                               */
} Statistic;

typedef struct _TPS{

	uint8_t highCodeRate;
	uint8_t lowCodeRate;
	uint8_t transmissionMode;
	uint8_t constellation; 
	uint8_t interval;
	uint16_t cellid;

} TPS, *pTPS;

/**
 * The type defination of Statistic.
 */
typedef struct {
	uint16_t abortCount;
	uint32_t postVitBitCount;
	uint32_t postVitErrorCount;
	/** float point */
	uint32_t softBitCount;
	uint32_t softErrorCount;
	uint32_t preVitBitCount;
	uint32_t preVitErrorCount;
	uint8_t snr;
} ChannelStatistic;


//------------------------
/**
 * The type defination of Demodulator.
 */
typedef struct {
	Handle userData;
	Handle driver;
} Demodulator;

/**
 * General tuner opening function
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
typedef uint32_t (*OpenTuner) (
		Demodulator*    demodulator
		);


/**
 * General tuner closing function
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
typedef uint32_t (*CloseTuner) (
		Demodulator*    demodulator
		);


/**
 * General tuner setting function
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
typedef uint32_t (*SetTuner) (
		Demodulator*    demodulator,
		uint16_t            bandwidth,
		uint32_t           frequency
		);
/**
 * The type defination of TunerDescription
 */
typedef struct {
	OpenTuner       openTunerFunc;
	CloseTuner      closeTunerFunc;
	SetTuner        setTunerFunc;
	ValueSet*       tunerScriptTable;
	uint16_t*           tunerScriptSetsTable;
	uint8_t            tunerAddress;
	uint8_t            registerAddressLength;
	uint32_t           ifFrequency;
	Bool            inversion;
	uint16_t            tunerId;
} TunerDescription;
//--------------------------

typedef enum {
	UNUSED = 0,
	GPIOH1 = 0xD8AE,           //r_eagle_reg_top_gpioh1_i
	GPIOH2 = 0xD8B6,           //r_eagle_reg_top_gpioh2_i
	GPIOH3 = 0xD8B2,           //r_eagle_reg_top_gpioh3_i
	GPIOH4 = 0xD8BE,           //r_eagle_reg_top_gpioh4_i
	GPIOH5 = 0xD8BA,           //r_eagle_reg_top_gpioh5_i
	GPIOH6 = 0xD8C6,           //r_eagle_reg_top_gpioh6_i
	GPIOH7 = 0xD8C2,           //r_eagle_reg_top_gpioh7_i
	GPIOH8 = 0xD8CE            //r_eagle_reg_top_gpioh8_i

} GPIO;


typedef struct _SystemConfig{
	GPIO	resetSlave;
	GPIO	rfEnable;
	GPIO	loClk;
	GPIO	loData;
	GPIO	loLe;
	GPIO	lnaPowerDown;
	GPIO	irDa;
	GPIO	uvFilter;
	GPIO	chSelect3;
	GPIO	chSelect2;
	GPIO	chSelect1;
	GPIO	chSelect0;
	GPIO	powerDownSlave;
	GPIO	uartTxd;
	GPIO	muxSelect;
	GPIO	lnaGain;
	GPIO	intrEnable;
	GPIO	dataRateCheck;
	GPIO	slaveLock;
	GPIO	filter_1;
	GPIO	filter_2;
	GPIO	uartRxd;
	GPIO	eepromSwitch;
	GPIO	filter_1n;
	GPIO	filter_2n;
} SystemConfig, *pSystemConfig;

/**
 * The data structure of Eagle
 */
typedef struct {
	/** Basic structure */
	Handle userData;
	uint8_t busId;
	uint8_t i2cAddr;
	uint8_t* firmwareCodes;
	Segment* firmwareSegments;
	uint16_t*  firmwarePartitions;
	uint16_t* scriptSets;
	ValueSet* scripts;
	TsInterface tsInterfaceType;
	uint16_t bandwidth;
	uint32_t frequency;    
	Bool booted;
	uint8_t slaveIICAddr;  
	ChannelModulation channelModulation;
	CalibrationInfo calibrationInfo;
	DCInfo dcInfo;
} Modulator;



/**
 * The data structure of Ganymede
 */
typedef struct {
	/** Basic structure */
	Handle userData;
	Handle driver;
	uint8_t* firmwareCodes;
	Segment* firmwareSegments;
	uint16_t* firmwarePartitions;
	uint16_t* scriptSets;
	ValueSet* scripts;
	uint32_t crystalFrequency;
	uint32_t adcFrequency;
	StreamType streamType;
	uint16_t bandwidth;
	uint32_t frequency;
	uint32_t fcw;
	Statistic statistic;
	uint8_t hostInterface;
	Bool booted;
	Bool initialized;
	uint8_t pidCounter;
	uint8_t demodAddr;       /** Demodulator I2C Address */
	Bool Clkout_en;       /** Clock output enable */
	ChannelStatistic channelStatistic;
	Modulator *modulator;
} DefaultDemodulator;
//--------------------------


typedef enum {
	filter = 0,	
	LayerB = 1,
	LayerA = 2,
	LayerAB = 3
} TransportLayer;


/**
 * The type defination of Constellation.
 */
typedef enum {
	DownSampleRate_21_OVER_1 = 0,		/** Signal uses FEC coding ratio of 21/1						*/
	DownSampleRate_21_OVER_2,			/** Signal uses FEC coding ratio of 21/2						*/
	DownSampleRate_21_OVER_3,			/** Signal uses FEC coding ratio of 21/3						*/
	DownSampleRate_21_OVER_4,			/** Signal uses FEC coding ratio of 21/4						*/
	DownSampleRate_21_OVER_5,			/** Signal uses FEC coding ratio of 21/5						*/
	DownSampleRate_21_OVER_6,			/** Signal uses FEC coding ratio of 21/6						*/
} DownSampleRate;

typedef enum {
	DVBT = 1,
	ISDBT
} OutputMode;

typedef enum {
	NullPacketModeDisable = 0,
	SequentialMode = 1,
	NormalMode
} NullPacketMode;

typedef enum {
	PcrModeDisable = 0,
	PcrMode1 = 1,
	PcrMode2,
	PcrMode3
} PcrMode;

typedef enum {
	ARIB_STD_B31 = 0,					/** System based on this specification							*/
	ISDB_TSB							/** System for ISDB-TSB											*/
} SystemIdentification;


typedef struct {
	Constellation	constellation;      /** Constellation scheme (FFT mode) in use						*/
	CodeRate		codeRate;		    /** FEC coding ratio of high-priority stream					*/
} TMCC;

typedef struct {
	uint32_t		frequency;              /** Channel frequency in KHz.									*/
	Bandwidth	bandwidth;
	TransmissionModes transmissionMode; /** Number of carriers used for OFDM signal						*/
	Interval	interval;               /** Fraction of symbol length used as guard (Guard Interval)	*/
	TMCC		layerA;
	TMCC		layerB;	
	Bool		isPartialReception;
} ISDBTModulation;

typedef struct _TMCCINFO{
	TMCC					layerA;
	TMCC					layerB;	
	Bool					isPartialReception;
	SystemIdentification	systemIdentification;
} TMCCINFO, *pTMCCINFO;


typedef struct {
	PcrMode pcrMode;
	uint16_t bandwidth;
	ChannelModulation channelModulation;
	ISDBTModulation   isdbtModulation;
	OutputMode outputMode; //0:un 1:DVB 2:isdbt; 
	uint32_t   packetTimeJitter_ps;
	uint32_t	pcrExtJitter;
	int		positive; //-1:- / 1:+
} PCRCALINFO;

/**
 * The data structure of IT9510
 */
typedef struct {
	/** Basic structure */
	Handle userData;
	uint8_t busId;
	uint8_t i2cAddr;
	uint8_t* firmwareCodes;
	Segment* firmwareSegments;
	uint16_t*  firmwarePartitions;
	uint16_t* scriptSets;
	ValueSet* scripts;
	TsInterface tsInterfaceType;
	uint16_t bandwidth;
	uint32_t frequency;    
	Bool booted;
	uint8_t slaveIICAddr;  
	ChannelModulation channelModulation;
	CalibrationInfo calibrationInfo;
	DCInfo dcInfo;
	ISDBTModulation   isdbtModulation;
	OutputMode outputMode; //0:un 1:DVB 2:isdbt; 
	NullPacketMode nullPacketMode;
	PcrMode pcrMode;
	Bool isExtLo;
	PCRCALINFO	pcrCalInfo;
} IT9510INFO;


extern const uint8_t IT9510_bitMask[8];
#define IT9510_REG_MASK(pos, len)                (IT9510_bitMask[len-1] << pos)
#define IT9510_REG_CLEAR(temp, pos, len)         (temp & (~IT9510_REG_MASK(pos, len)))
#define IT9510_REG_CREATE(val, temp, pos, len)   ((val << pos) | (IT9510_REG_CLEAR(temp, pos, len)))
#define IT9510_REG_GET(value, pos, len)          ((value & IT9510_REG_MASK(pos, len)) >> pos)

extern const uint8_t Eagle_bitMask[8];
#define REG_MASK(pos, len)                (Eagle_bitMask[len-1] << pos)
#define REG_CLEAR(temp, pos, len)         (temp & (~REG_MASK(pos, len)))
#define REG_CREATE(val, temp, pos, len)   ((val << pos) | (REG_CLEAR(temp, pos, len)))
#define REG_GET(value, pos, len)          ((value & REG_MASK(pos, len)) >> pos)
#define LOWBYTE(w)      				  ((uint8_t)((w) & 0xff))
#define HIGHBYTE(w)     				  ((uint8_t)((w >> 8) & 0xff))
#define OMEGA_NORMAL                      0x00
#define OMEGA_LNA_Config_1                0x01
#define OMEGA_LNA_Config_2                0x02
#define OMEGA_LNA_Config_3                0x03
#define OMEGA_LNA_Config_4				  0x04
#define OMEGA_LNA_Config_5				  0x05
#endif
