#include "standard.h"
#include "cmd.h"
#include "user.h"
#include "tuner.h"
#include "firmware.h"
#include "firmware_V2.h"
#include "firmware_V2I.h"
#include "firmware_V2W.h"

/*
   User_delay to 100*User_delay

*/

#if ((DVB_V2_LL_VERSION3<1)||(DVB_V2_OFDM_VERSION3<1))
#error Firmware version too old.  Please update Firmware version.
#endif

//#if ((OFDM_VERSION3 < 14)||(LL_VERSION3 < 14))
//#error Firmware version too old.  Please update Firmware version.
//#endif


#ifndef Firmware_FORMAT_VER1
#define Firmware_FORMAT_VER1 			1
#define Firmware_FORMAT_VER2 			0
#endif

#define Standard_MAX_BIT                8
#define Standard_MAX_CLOCK              12
#define Standard_MAX_BAND               3


uint8_t Chip2_I2c_address = User_Chip2_I2C_ADDRESS;
//uint8_t Pre_SQI[2];
const uint8_t Standard_bitMask[Standard_MAX_BIT] = {
	0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF
};

static ClockTable Standard_clockTable[Standard_MAX_CLOCK] =
{
	{0,0}
};

const BandTable Standard_bandTable[Standard_MAX_BAND] =
{
	{    30000,     300000     },      /** VHF    30MHz ~ 300MHz*/
	{    300000,    1000000    },      /** UHF    300MHz ~ 1000MHz*/
	{   1670000,    1680000    }       /** L-BAND */
};

const int Standard_PrefValues[3][5] =
{
      {-93, -91, -90, -89, -88},       /** QPSK    CodeRate 1/2 ~ 7/8 Pref Values*/
      {-87, -85, -84, -83, -82},       /** 16QAM   CodeRate 1/2 ~ 7/8 Pref Values*/
      {-82, -80, -78, -77, -76},       /** 64QAM   CodeRate 1/2 ~ 7/8 Pref Values*/
};/** NorDig 3.4.4.6 & Table 3.4*/

const uint32_t  Standard_CNRequiredValues[3][5] =
{
      {51, 69, 79, 89, 97},       /** QPSK    CodeRate 1/2 ~ 7/8 CN Required Values*/
      {108, 131, 146, 156, 160},  /** 16QAM   CodeRate 1/2 ~ 7/8 CN Required Values*/
      {165, 187, 202, 216, 225},  /** 64QAM   CodeRate 1/2 ~ 7/8 CN Required Values*/
};
const uint32_t  Standard_HierarchicalCNRequiredValues[2][3][4] =    /**[16 or 64 QAM][CodeRate][a & H/L]   */
{
    {/**{HP(a=2),  HP(a=4),   LP(a=2), LP(a=4) } */
		{68, 58, 150, 195},		/** QPSK in 16QAM   CodeRate 1/2 CN Required Values*/
		{91, 79, 172, 214},		/** QPSK in 16QAM   CodeRate 2/3 CN Required Values*/
		{104, 91, 184, 225},	/** QPSK in 16QAM   CodeRate 3/4 CN Required Values*/

	},
	{/**{HP(a=1),  HP(a=2),   LP(a=1), LP(a=2) } */
		{109, 85, 167, 185},	/** QPSK in 64QAM   CodeRate 1/2 CN Required Values*/
		{141, 110, 191, 212},		/** QPSK in 64QAM   CodeRate 2/3 CN Required Values*/
		{157, 128, 209, 236},	/** QPSK in 64QAM   CodeRate 3/4 CN Required Values*/
	},
};

/** local functions */


uint32_t Standard_divider (
		Demodulator*    demodulator,
	  uint32_t           a,
	  uint32_t           b,
	  uint32_t           x
) {
	uint32_t answer = 0;
	uint32_t c = 0;
	uint32_t i = 0;

	if (a > b) {
		c = a / b;
		a = a - c * b;
	}

	for (i = 0; i < x; i++) {
		if (a >= b) {
			answer += 1;
			a-=b;
		}
		a <<= 1;
		answer <<= 1;
	}

	answer = (c << (Long) x) + answer;

	return (answer);
}


uint32_t Standard_computeCrystal (
		Demodulator*    demodulator,
	  Long            crystalFrequency,   /** Crystal frequency (Hz) */
	OUT uint32_t*          crystal
) {
	uint32_t   error = Error_NO_ERROR;

	*crystal = (Long) Standard_divider (demodulator, (uint32_t) crystalFrequency, 1000000ul, 19ul);

	return (error);
}


uint32_t Standard_computeAdc (
		Demodulator*    demodulator,
	  Long            adcFrequency,       /** ADC frequency (Hz) */
	 uint32_t*          adc
)
{
	uint32_t   error = Error_NO_ERROR;

	*adc = (Long) Standard_divider (demodulator, (uint32_t) adcFrequency, 1000000ul, 19ul);

	return (error);
}


uint32_t Standard_computeFcw (
		Demodulator*    demodulator,
	  Long            adcFrequency,       /** ADC frequency (Hz)    */
	  Long            ifFrequency,        /** IF frequency (Hz)     */
	  Booll            inversion,          /** RF spectrum inversion */
	 uint32_t*          fcw
) {
	uint32_t error = Error_NO_ERROR;
	Long ifFreq;
	Long adcFreq;
	Long adcFreqHalf;
	Long adcFreqSample;
	Long invBfs;
	Long controlWord;
	uint8_t adcMultiplier;

	adcFreq = adcFrequency;
	ifFreq = ifFrequency;
	adcFreqHalf = adcFreq / 2;

	if (inversion == True)
		ifFreq = -1 * ifFreq;

	adcFreqSample = ifFreq;

	if (adcFreqSample >= 0)
		invBfs = 1;
	else {
		invBfs = -1;
		adcFreqSample = adcFreqSample * -1;
	}

	while (adcFreqSample > adcFreqHalf)
		adcFreqSample = adcFreqSample - adcFreq;

	/** Sample, spectrum at positive frequency */
	if(adcFreqSample >= 0)
		invBfs = invBfs * -1;
	else {
		invBfs = invBfs * 1;
		adcFreqSample = adcFreqSample * (-1);       /** Absolute value */
	}

	controlWord = (Long) Standard_divider (demodulator, (uint32_t) adcFreqSample, (uint32_t) adcFreq, 23ul);

	if (invBfs == -1) {
		controlWord *= -1;
	}

	/** Get ADC multiplier */
	error = Standard_readRegister (demodulator, 0, Processor_OFDM, adcx2, &adcMultiplier);
	if (error) goto exit;

	if (adcMultiplier == 1) {
		controlWord /= 2;
	}

	*fcw = controlWord & 0x7FFFFF;

exit :
	return (error);
}


uint32_t Standard_programFcw (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Long            shift,          /** Hz */
	  uint32_t           adcFrequency    /** Hz */
)
{
	uint32_t error = Error_NO_ERROR;
	uint32_t fcw;
	Long fcwShift;
	uint8_t temp0;
	uint8_t temp1;
	uint8_t temp2;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	/** Get shift freq */
	fcwShift = (shift * 8 * 1024 + (Long) adcFrequency / (2 * 1024)) / (Long) adcFrequency * 1024;

	fcw  = (uint32_t) ((Long) it9130->fcw + fcwShift);

	temp0 = (uint8_t) (fcw  & 0x000000FF);
	temp1 = (uint8_t) ((fcw & 0x0000FF00) >> 8);
	temp2 = (uint8_t) ((fcw & 0x007F0000) >> 16);

	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_bfs_fcw_7_0, temp0);
	if (error) goto exit;
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_bfs_fcw_15_8, temp1);
	if (error) goto exit;
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_bfs_fcw_22_16, temp2);
	if (error) goto exit;

exit :
	return (error);
}


uint32_t Standard_maskDcaOutput (
		Demodulator*    demodulator
) {
	uint32_t error = Error_NO_ERROR;
	uint8_t i;
	Booll dcaValid = False;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if ((it9130->chipNumber > 1) && ((it9130->architecture == Architecture_DCA) ))
		dcaValid = True;

	if (dcaValid == True) {
		for (i = 0; i < it9130->chipNumber; i++) {
			error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_dca_upper_out_en, reg_dca_upper_out_en_pos, reg_dca_upper_out_en_len, 0);
			if (error) goto exit;
		}
		User_delay ( 5);
	}

exit :
	return (error);
}


uint32_t Standard_sendCommand (
		Demodulator*    demodulator,
	 uint16_t            command,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           writeBufferLength,
	  uint8_t*           writeBuffer,
	  uint32_t           readBufferLength,
	 uint8_t*           readBuffer
) {
	uint32_t error = Error_NO_ERROR;

	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (Cmd_sendCommand != NULL) {
		error = Cmd_sendCommand ( command, chip, processor, writeBufferLength, writeBuffer, readBufferLength, readBuffer);
	}
	return (error);
}

uint32_t Standard_selectBandwidth(
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint16_t            bandwidth,          /** KHz              */
	  uint32_t           adcFrequency        /** Hz, ex: 20480000 */
	) {
	uint32_t error = Error_NO_ERROR;
	uint32_t BWerror = Error_NO_ERROR;
	uint32_t coeff1_2048Nu = 0;
	uint32_t coeff1_4096Nu = 0;
	uint32_t coeff1_8191Nu = 0;
	uint32_t coeff1_8192Nu = 0;
	uint32_t coeff1_8193Nu = 0;
	uint32_t coeff2_2k = 0;
	uint32_t coeff2_4k = 0;
	uint32_t coeff2_8k = 0;
	uint16_t bfsfcw_fftindex_ratio = 0;
	uint16_t fftindex_bfsfcw_ratio = 0;

	uint8_t temp0;
	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;

	uint8_t buffer[36];
	uint8_t bw = 0xff;
	uint8_t adcMultiplier;
	IT9130* it9130;
	uint32_t adc = 0;
	uint32_t fcw = 0;
	uint8_t clk_strap;

	it9130 = (IT9130*)demodulator;


	error = Standard_readRegisterBits(demodulator, chip, Processor_LINK, r_io_mux_pwron_clk_strap, io_mux_pwron_clk_strap_pos, io_mux_pwron_clk_strap_len, &clk_strap);
	if (error) goto exit;

	if (bandwidth == 5000){
		if (clk_strap == 1){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP1_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 3;
			adcFrequency = 20250000;
		}
	}
	else if (bandwidth == 6000) {
		if (clk_strap == 1){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP1_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 0;
			adcFrequency = 20250000;
		}
	}
	else if (bandwidth == 7000) {
		if (clk_strap == 1){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP1_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 1;
			adcFrequency = 20250000;
		}
	}
	else if (bandwidth == 8000) {
		if (clk_strap == 1){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP1_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 2;
			adcFrequency = 20250000;
		}
	}
	else if (bandwidth == 2000) {
		if (clk_strap == 0){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP0_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 3;
			adcFrequency = 30000000;
			error = Standard_writeRegister(demodulator, chip, Processor_OFDM, p_reg_mpeg_full_speed, 0x01);
			//printf("*** 2MHz Mode -- Strap GPIOH1 to 1.  Current strapping = %d ***", clk_strap);
		}
	}
	else if (bandwidth == 2500) {
		if (clk_strap == 0){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP0_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 3;
			adcFrequency = 24000000;
			error = Standard_writeRegister(demodulator, chip, Processor_OFDM, p_reg_mpeg_full_speed, 0x01);
			//printf("*** 2MHz Mode -- Strap GPIOH1 to 1.  Current strapping = %d ***", clk_strap);
		}
	}
	else if (bandwidth == 3000) {
		if (clk_strap == 0){
			adcFrequency = 0;
			BWerror = Error_CLK_STRAP0_INVALID_BW;
			//goto exit;
		}
		else{
			bw = 3;
			adcFrequency = 20000000;
			error = Standard_writeRegister(demodulator, chip, Processor_OFDM, p_reg_mpeg_full_speed, 0x01);
			//printf("*** 3MHz Mode -- Strap GPIOH1 to 1 Current strapping = %d ***", clk_strap);
		}
	}
	else if (bandwidth == 4000) {
		bw = 3;
		if (clk_strap == 1) {
			adcFrequency = 15000000;
			error = Standard_writeRegister(demodulator, chip, Processor_OFDM, p_reg_mpeg_full_speed, 0x01);
		}
		else {
			adcFrequency = 25312500;
		}
	}
	else {
		BWerror = Error_INVALID_BW;
		//goto exit;
	}

	it9130->adcFrequency = adcFrequency;

	error = Standard_writeRegisterBits(demodulator, chip, Processor_OFDM, g_reg_bw, reg_bw_pos, reg_bw_len, bw);
	if (error) goto exit;



	/** Compute ADC and load them to device */
	error = Standard_computeAdc(demodulator, (Long)it9130->adcFrequency, &adc);
	if (error) goto exit;

	buffer[0] = (uint8_t)(adc & 0x000000FF);
	buffer[1] = (uint8_t)((adc & 0x0000FF00) >> 8);
	buffer[2] = (uint8_t)((adc & 0x00FF0000) >> 16);
	error = Standard_writeRegisters(demodulator, chip, Processor_OFDM, p_reg_f_adc_7_0, 3, buffer);
	if (error) goto exit;

	/** Compute FCW and load them to device */
	error = Standard_computeFcw(demodulator, (Long)it9130->adcFrequency, (Long)it9130->tunerDescription->ifFrequency, it9130->tunerDescription->inversion, &fcw);
	if (error) goto exit;
	it9130->fcw = fcw;

	buffer[0] = (uint8_t)(fcw & 0x000000FF);
	buffer[1] = (uint8_t)((fcw & 0x0000FF00) >> 8);
	buffer[2] = (uint8_t)((fcw & 0x007F0000) >> 16);
	error = Standard_writeRegisters(demodulator, chip, Processor_OFDM, bfs_fcw_7_0, bfs_fcw_22_16 - bfs_fcw_7_0 + 1, buffer);
	if (error) goto exit;



	/** Program CFOE */
	if (adcFrequency == 20156250) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02449b5c;
			coeff1_4096Nu = 0x01224dae;
			coeff1_8191Nu = 0x00912b60;
			coeff1_8192Nu = 0x009126d7;
			coeff1_8193Nu = 0x0091224e;
			coeff2_2k = 0x01224dae;
			coeff2_4k = 0x009126d7;
			coeff2_8k = 0x0048936b;
			bfsfcw_fftindex_ratio = 0x0387;
			fftindex_bfsfcw_ratio = 0x0122;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02b8ba6e;
			coeff1_4096Nu = 0x015c5d37;
			coeff1_8191Nu = 0x00ae340d;
			coeff1_8192Nu = 0x00ae2e9b;
			coeff1_8193Nu = 0x00ae292a;
			coeff2_2k = 0x015c5d37;
			coeff2_4k = 0x00ae2e9b;
			coeff2_8k = 0x0057174e;
			bfsfcw_fftindex_ratio = 0x02f1;
			fftindex_bfsfcw_ratio = 0x015c;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x032cd980;
			coeff1_4096Nu = 0x01966cc0;
			coeff1_8191Nu = 0x00cb3cba;
			coeff1_8192Nu = 0x00cb3660;
			coeff1_8193Nu = 0x00cb3007;
			coeff2_2k = 0x01966cc0;
			coeff2_4k = 0x00cb3660;
			coeff2_8k = 0x00659b30;
			bfsfcw_fftindex_ratio = 0x0285;
			fftindex_bfsfcw_ratio = 0x0196;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03a0f893;
			coeff1_4096Nu = 0x01d07c49;
			coeff1_8191Nu = 0x00e84567;
			coeff1_8192Nu = 0x00e83e25;
			coeff1_8193Nu = 0x00e836e3;
			coeff2_2k = 0x01d07c49;
			coeff2_4k = 0x00e83e25;
			coeff2_8k = 0x00741f12;
			bfsfcw_fftindex_ratio = 0x0234;
			fftindex_bfsfcw_ratio = 0x01d0;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20187500) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x0243b546;
			coeff1_4096Nu = 0x0121daa3;
			coeff1_8191Nu = 0x0090f1d9;
			coeff1_8192Nu = 0x0090ed51;
			coeff1_8193Nu = 0x0090e8ca;
			coeff2_2k = 0x0121daa3;
			coeff2_4k = 0x0090ed51;
			coeff2_8k = 0x004876a9;
			bfsfcw_fftindex_ratio = 0x0388;
			fftindex_bfsfcw_ratio = 0x0122;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02b7a654;
			coeff1_4096Nu = 0x015bd32a;
			coeff1_8191Nu = 0x00adef04;
			coeff1_8192Nu = 0x00ade995;
			coeff1_8193Nu = 0x00ade426;
			coeff2_2k = 0x015bd32a;
			coeff2_4k = 0x00ade995;
			coeff2_8k = 0x0056f4ca;
			bfsfcw_fftindex_ratio = 0x02f2;
			fftindex_bfsfcw_ratio = 0x015c;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x032b9761;
			coeff1_4096Nu = 0x0195cbb1;
			coeff1_8191Nu = 0x00caec30;
			coeff1_8192Nu = 0x00cae5d8;
			coeff1_8193Nu = 0x00cadf81;
			coeff2_2k = 0x0195cbb1;
			coeff2_4k = 0x00cae5d8;
			coeff2_8k = 0x006572ec;
			bfsfcw_fftindex_ratio = 0x0286;
			fftindex_bfsfcw_ratio = 0x0196;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x039f886f;
			coeff1_4096Nu = 0x01cfc438;
			coeff1_8191Nu = 0x00e7e95b;
			coeff1_8192Nu = 0x00e7e21c;
			coeff1_8193Nu = 0x00e7dadd;
			coeff2_2k = 0x01cfc438;
			coeff2_4k = 0x00e7e21c;
			coeff2_8k = 0x0073f10e;
			bfsfcw_fftindex_ratio = 0x0235;
			fftindex_bfsfcw_ratio = 0x01d0;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20250000) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x0241eb3b;
			coeff1_4096Nu = 0x0120f59e;
			coeff1_8191Nu = 0x00907f53;
			coeff1_8192Nu = 0x00907acf;
			coeff1_8193Nu = 0x0090764b;
			coeff2_2k = 0x0120f59e;
			coeff2_4k = 0x00907acf;
			coeff2_8k = 0x00483d67;
			bfsfcw_fftindex_ratio = 0x038b;
			fftindex_bfsfcw_ratio = 0x0121;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02b580ad;
			coeff1_4096Nu = 0x015ac057;
			coeff1_8191Nu = 0x00ad6597;
			coeff1_8192Nu = 0x00ad602b;
			coeff1_8193Nu = 0x00ad5ac1;
			coeff2_2k = 0x015ac057;
			coeff2_4k = 0x00ad602b;
			coeff2_8k = 0x0056b016;
			bfsfcw_fftindex_ratio = 0x02f4;
			fftindex_bfsfcw_ratio = 0x015b;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03291620;
			coeff1_4096Nu = 0x01948b10;
			coeff1_8191Nu = 0x00ca4bda;
			coeff1_8192Nu = 0x00ca4588;
			coeff1_8193Nu = 0x00ca3f36;
			coeff2_2k = 0x01948b10;
			coeff2_4k = 0x00ca4588;
			coeff2_8k = 0x006522c4;
			bfsfcw_fftindex_ratio = 0x0288;
			fftindex_bfsfcw_ratio = 0x0195;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x039cab92;
			coeff1_4096Nu = 0x01ce55c9;
			coeff1_8191Nu = 0x00e7321e;
			coeff1_8192Nu = 0x00e72ae4;
			coeff1_8193Nu = 0x00e723ab;
			coeff2_2k = 0x01ce55c9;
			coeff2_4k = 0x00e72ae4;
			coeff2_8k = 0x00739572;
			bfsfcw_fftindex_ratio = 0x0237;
			fftindex_bfsfcw_ratio = 0x01ce;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20583333) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02388f54;
			coeff1_4096Nu = 0x011c47aa;
			coeff1_8191Nu = 0x008e2846;
			coeff1_8192Nu = 0x008e23d5;
			coeff1_8193Nu = 0x008e1f64;
			coeff2_2k = 0x011c47aa;
			coeff2_4k = 0x008e23d5;
			coeff2_8k = 0x004711ea;
			bfsfcw_fftindex_ratio = 0x039a;
			fftindex_bfsfcw_ratio = 0x011c;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02aa4598;
			coeff1_4096Nu = 0x015522cc;
			coeff1_8191Nu = 0x00aa96bb;
			coeff1_8192Nu = 0x00aa9166;
			coeff1_8193Nu = 0x00aa8c12;
			coeff2_2k = 0x015522cc;
			coeff2_4k = 0x00aa9166;
			coeff2_8k = 0x005548b3;
			bfsfcw_fftindex_ratio = 0x0300;
			fftindex_bfsfcw_ratio = 0x0155;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x031bfbdc;
			coeff1_4096Nu = 0x018dfdee;
			coeff1_8191Nu = 0x00c7052f;
			coeff1_8192Nu = 0x00c6fef7;
			coeff1_8193Nu = 0x00c6f8bf;
			coeff2_2k = 0x018dfdee;
			coeff2_4k = 0x00c6fef7;
			coeff2_8k = 0x00637f7b;
			bfsfcw_fftindex_ratio = 0x0293;
			fftindex_bfsfcw_ratio = 0x018e;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x038db21f;
			coeff1_4096Nu = 0x01c6d910;
			coeff1_8191Nu = 0x00e373a3;
			coeff1_8192Nu = 0x00e36c88;
			coeff1_8193Nu = 0x00e3656d;
			coeff2_2k = 0x01c6d910;
			coeff2_4k = 0x00e36c88;
			coeff2_8k = 0x0071b644;
			bfsfcw_fftindex_ratio = 0x0240;
			fftindex_bfsfcw_ratio = 0x01c7;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20416667) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x023d337f;
			coeff1_4096Nu = 0x011e99c0;
			coeff1_8191Nu = 0x008f515a;
			coeff1_8192Nu = 0x008f4ce0;
			coeff1_8193Nu = 0x008f4865;
			coeff2_2k = 0x011e99c0;
			coeff2_4k = 0x008f4ce0;
			coeff2_8k = 0x0047a670;
			bfsfcw_fftindex_ratio = 0x0393;
			fftindex_bfsfcw_ratio = 0x011f;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02afd765;
			coeff1_4096Nu = 0x0157ebb3;
			coeff1_8191Nu = 0x00abfb39;
			coeff1_8192Nu = 0x00abf5d9;
			coeff1_8193Nu = 0x00abf07a;
			coeff2_2k = 0x0157ebb3;
			coeff2_4k = 0x00abf5d9;
			coeff2_8k = 0x0055faed;
			bfsfcw_fftindex_ratio = 0x02fa;
			fftindex_bfsfcw_ratio = 0x0158;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03227b4b;
			coeff1_4096Nu = 0x01913da6;
			coeff1_8191Nu = 0x00c8a518;
			coeff1_8192Nu = 0x00c89ed3;
			coeff1_8193Nu = 0x00c8988e;
			coeff2_2k = 0x01913da6;
			coeff2_4k = 0x00c89ed3;
			coeff2_8k = 0x00644f69;
			bfsfcw_fftindex_ratio = 0x028d;
			fftindex_bfsfcw_ratio = 0x0191;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03951f32;
			coeff1_4096Nu = 0x01ca8f99;
			coeff1_8191Nu = 0x00e54ef7;
			coeff1_8192Nu = 0x00e547cc;
			coeff1_8193Nu = 0x00e540a2;
			coeff2_2k = 0x01ca8f99;
			coeff2_4k = 0x00e547cc;
			coeff2_8k = 0x0072a3e6;
			bfsfcw_fftindex_ratio = 0x023c;
			fftindex_bfsfcw_ratio = 0x01cb;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20480000) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x023b6db7;
			coeff1_4096Nu = 0x011db6db;
			coeff1_8191Nu = 0x008edfe5;
			coeff1_8192Nu = 0x008edb6e;
			coeff1_8193Nu = 0x008ed6f7;
			coeff2_2k = 0x011db6db;
			coeff2_4k = 0x008edb6e;
			coeff2_8k = 0x00476db7;
			bfsfcw_fftindex_ratio = 0x0396;
			fftindex_bfsfcw_ratio = 0x011e;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02adb6db;
			coeff1_4096Nu = 0x0156db6e;
			coeff1_8191Nu = 0x00ab7312;
			coeff1_8192Nu = 0x00ab6db7;
			coeff1_8193Nu = 0x00ab685c;
			coeff2_2k = 0x0156db6e;
			coeff2_4k = 0x00ab6db7;
			coeff2_8k = 0x0055b6db;
			bfsfcw_fftindex_ratio = 0x02fd;
			fftindex_bfsfcw_ratio = 0x0157;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03200000;
			coeff1_4096Nu = 0x01900000;
			coeff1_8191Nu = 0x00c80640;
			coeff1_8192Nu = 0x00c80000;
			coeff1_8193Nu = 0x00c7f9c0;
			coeff2_2k = 0x01900000;
			coeff2_4k = 0x00c80000;
			coeff2_8k = 0x00640000;
			bfsfcw_fftindex_ratio = 0x028f;
			fftindex_bfsfcw_ratio = 0x0190;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x03924925;
			coeff1_4096Nu = 0x01c92492;
			coeff1_8191Nu = 0x00e4996e;
			coeff1_8192Nu = 0x00e49249;
			coeff1_8193Nu = 0x00e48b25;
			coeff2_2k = 0x01c92492;
			coeff2_4k = 0x00e49249;
			coeff2_8k = 0x00724925;
			bfsfcw_fftindex_ratio = 0x023d;
			fftindex_bfsfcw_ratio = 0x01c9;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20500000) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x023adeff;
			coeff1_4096Nu = 0x011d6f80;
			coeff1_8191Nu = 0x008ebc36;
			coeff1_8192Nu = 0x008eb7c0;
			coeff1_8193Nu = 0x008eb34a;
			coeff2_2k = 0x011d6f80;
			coeff2_4k = 0x008eb7c0;
			coeff2_8k = 0x00475be0;
			bfsfcw_fftindex_ratio = 0x0396;
			fftindex_bfsfcw_ratio = 0x011d;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02ad0b99;
			coeff1_4096Nu = 0x015685cc;
			coeff1_8191Nu = 0x00ab4840;
			coeff1_8192Nu = 0x00ab42e6;
			coeff1_8193Nu = 0x00ab3d8c;
			coeff2_2k = 0x015685cc;
			coeff2_4k = 0x00ab42e6;
			coeff2_8k = 0x0055a173;
			bfsfcw_fftindex_ratio = 0x02fd;
			fftindex_bfsfcw_ratio = 0x0157;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x031f3832;
			coeff1_4096Nu = 0x018f9c19;
			coeff1_8191Nu = 0x00c7d44b;
			coeff1_8192Nu = 0x00c7ce0c;
			coeff1_8193Nu = 0x00c7c7ce;
			coeff2_2k = 0x018f9c19;
			coeff2_4k = 0x00c7ce0c;
			coeff2_8k = 0x0063e706;
			bfsfcw_fftindex_ratio = 0x0290;
			fftindex_bfsfcw_ratio = 0x0190;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x039164cb;
			coeff1_4096Nu = 0x01c8b266;
			coeff1_8191Nu = 0x00e46056;
			coeff1_8192Nu = 0x00e45933;
			coeff1_8193Nu = 0x00e45210;
			coeff2_2k = 0x01c8b266;
			coeff2_4k = 0x00e45933;
			coeff2_8k = 0x00722c99;
			bfsfcw_fftindex_ratio = 0x023e;
			fftindex_bfsfcw_ratio = 0x01c9;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20625000) {
		if (bandwidth == 5000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02376948;
			coeff1_4096Nu = 0x011bb4a4;
			coeff1_8191Nu = 0x008ddec1;
			coeff1_8192Nu = 0x008dda52;
			coeff1_8193Nu = 0x008dd5e3;
			coeff2_2k = 0x011bb4a4;
			coeff2_4k = 0x008dda52;
			coeff2_8k = 0x0046ed29;
			bfsfcw_fftindex_ratio = 0x039c;
			fftindex_bfsfcw_ratio = 0x011c;
		}
		else if (bandwidth == 6000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x02a8e4bd;
			coeff1_4096Nu = 0x0154725e;
			coeff1_8191Nu = 0x00aa3e81;
			coeff1_8192Nu = 0x00aa392f;
			coeff1_8193Nu = 0x00aa33de;
			coeff2_2k = 0x0154725e;
			coeff2_4k = 0x00aa392f;
			coeff2_8k = 0x00551c98;
			bfsfcw_fftindex_ratio = 0x0302;
			fftindex_bfsfcw_ratio = 0x0154;
		}
		else if (bandwidth == 7000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x031a6032;
			coeff1_4096Nu = 0x018d3019;
			coeff1_8191Nu = 0x00c69e41;
			coeff1_8192Nu = 0x00c6980c;
			coeff1_8193Nu = 0x00c691d8;
			coeff2_2k = 0x018d3019;
			coeff2_4k = 0x00c6980c;
			coeff2_8k = 0x00634c06;
			bfsfcw_fftindex_ratio = 0x0294;
			fftindex_bfsfcw_ratio = 0x018d;
		}
		else if (bandwidth == 8000 && BWerror == Error_NO_ERROR) {
			coeff1_2048Nu = 0x038bdba6;
			coeff1_4096Nu = 0x01c5edd3;
			coeff1_8191Nu = 0x00e2fe02;
			coeff1_8192Nu = 0x00e2f6ea;
			coeff1_8193Nu = 0x00e2efd2;
			coeff2_2k = 0x01c5edd3;
			coeff2_4k = 0x00e2f6ea;
			coeff2_8k = 0x00717b75;
			bfsfcw_fftindex_ratio = 0x0242;
			fftindex_bfsfcw_ratio = 0x01c6;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 20000000) {
		if (bandwidth == 3000 && BWerror == Error_NO_ERROR) {//YUMIN for 3MHz
			coeff1_2048Nu = 0x02492492;
			coeff1_4096Nu = 0x01249249;
			coeff1_8191Nu = 0x00924db7;
			coeff1_8192Nu = 0x00924925;
			coeff1_8193Nu = 0x00924492;
			coeff2_2k = 0x01249129;
			coeff2_4k = 0x00924925;
			coeff2_8k = 0x00492492;
			bfsfcw_fftindex_ratio = 0x0380;
			fftindex_bfsfcw_ratio = 0x0125;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 25312500) {
		if (bandwidth == 4000 && BWerror == Error_NO_ERROR) {//YUMIN for 4MHz (GPIOH1 = 1)
			coeff1_2048Nu = 0x01ce55c9;
			coeff1_4096Nu = 0x00e72ae4;
			coeff1_8191Nu = 0x0073990f;
			coeff1_8192Nu = 0x00739572;
			coeff1_8193Nu = 0x007391d6;
			coeff2_2k = 0x00e72ae4;
			coeff2_4k = 0x00739572;
			coeff2_8k = 0x0039cab9;
			bfsfcw_fftindex_ratio = 0x046e;
			fftindex_bfsfcw_ratio = 0x00e7;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 15000000) {
		if (bandwidth == 4000 && BWerror == Error_NO_ERROR) {//YUMIN for 4MHz (GPIOH1 = 0)
			coeff1_2048Nu = 0x030c30c3;
			coeff1_4096Nu = 0x01861862;
			coeff1_8191Nu = 0x00c31249;
			coeff1_8192Nu = 0x00c30c31;
			coeff1_8193Nu = 0x00c30619;
			coeff2_2k = 0x01861862;
			coeff2_4k = 0x00c30c31;
			coeff2_8k = 0x00618618;
			bfsfcw_fftindex_ratio = 0x02a0;
			fftindex_bfsfcw_ratio = 0x0186;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 24000000) {
		if (bandwidth == 2500 && BWerror == Error_NO_ERROR) {//YUMIN for 2.5MHz
			coeff1_2048Nu = 0x01e79e7a;
			coeff1_4096Nu = 0x00f3cf3d;
			coeff1_8191Nu = 0x0079eb6e;
			coeff1_8192Nu = 0x0079e79e;
			coeff1_8193Nu = 0x0079e3cf;
			coeff2_2k = 0x00f3cf3d;
			coeff2_4k = 0x0079e79e;
			coeff2_8k = 0x003cf3cf;
			bfsfcw_fftindex_ratio = 0x0433;
			fftindex_bfsfcw_ratio = 0x00f4;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else if (adcFrequency == 30000000) {
		if (bandwidth == 2000 && BWerror == Error_NO_ERROR) {//YUMIN for 2MHz
			coeff1_2048Nu = 0x01861862;
			coeff1_4096Nu = 0x00c30c31;
			coeff1_8191Nu = 0x00618925;
			coeff1_8192Nu = 0x00618618;
			coeff1_8193Nu = 0x0061830c;
			coeff2_2k = 0x00c30c31;
			coeff2_4k = 0x00618618;
			coeff2_8k = 0x0030c30c;
			bfsfcw_fftindex_ratio = 0x0540;
			fftindex_bfsfcw_ratio = 0x00c3;
		}
		else {
			//error = Error_INVALID_BW;
			//goto exit;
		}
	}
	else {
		error = Error_INVALID_XTAL_FREQ;
		goto exit;
	}


	/** Get ADC multiplier */
	error = Standard_readRegister(demodulator, 0, Processor_OFDM, adcx2, &adcMultiplier);
	if (error) goto exit;

	if (adcMultiplier == 1) {
		coeff1_2048Nu /= 2;
		coeff1_4096Nu /= 2;
		coeff1_8191Nu /= 2;
		coeff1_8192Nu /= 2;
		coeff1_8193Nu /= 2;
		coeff2_2k /= 2;
		coeff2_4k /= 2;
		coeff2_8k /= 2;
	}

	/** Write coeff1_2048Nu */
	/** Get Byte0 */
	temp0 = (uint8_t)(coeff1_2048Nu & 0x000000FF);
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff1_2048Nu & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff1_2048Nu & 0x00FF0000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff1_2048Nu & 0x03000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_2048_coeff1_25_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_2048_coeff1_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_2048_coeff1_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_2048_coeff1_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Write coeff2_2k */
	/** Get Byte0 */
	temp0 = (uint8_t)((coeff2_2k & 0x000000FF));
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff2_2k & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff2_2k & 0x00FF0000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff2_2k & 0x01000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_2k_coeff2_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_2k_coeff2_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_2k_coeff2_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_2k_coeff2_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Write coeff1_8191Nu */
	/** Get Byte0 */
	temp0 = (uint8_t)((coeff1_8191Nu & 0x000000FF));
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff1_8191Nu & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff1_8191Nu & 0x00FFC000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff1_8191Nu & 0x03000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_8191_coeff1_25_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_8191_coeff1_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_8191_coeff1_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_8191_coeff1_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;
	/** Write coeff1_8192Nu */
	/** Get Byte0 */
	temp0 = (uint8_t)(coeff1_8192Nu & 0x000000FF);
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff1_8192Nu & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff1_8192Nu & 0x00FFC000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff1_8192Nu & 0x03000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_8192_coeff1_25_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_8192_coeff1_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_8192_coeff1_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_8192_coeff1_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Write coeff1_8193Nu */
	/** Get Byte0 */
	temp0 = (uint8_t)((coeff1_8193Nu & 0x000000FF));
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff1_8193Nu & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff1_8193Nu & 0x00FFC000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff1_8193Nu & 0x03000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_8193_coeff1_25_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_8193_coeff1_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_8193_coeff1_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_8193_coeff1_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;
	/** Write coeff2_8k */
	/** Get Byte0 */
	temp0 = (uint8_t)((coeff2_8k & 0x000000FF));
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff2_8k & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff2_8k & 0x00FF0000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff2_8k & 0x01000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_8k_coeff2_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_8k_coeff2_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_8k_coeff2_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_8k_coeff2_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Write coeff1_4096Nu */
	/** Get Byte0 */
	temp0 = (uint8_t)(coeff1_4096Nu & 0x000000FF);
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff1_4096Nu & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff1_4096Nu & 0x00FF0000) >> 16);
	/** Get Byte3[1:0] */
	/** Bit[7:2] will be written soon and so don't have to care them */
	temp3 = (uint8_t)((coeff1_4096Nu & 0x03000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_4096_coeff1_25_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_4096_coeff1_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_4096_coeff1_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_4096_coeff1_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Write coeff2_4k */
	/** Get Byte0 */
	temp0 = (uint8_t)((coeff2_4k & 0x000000FF));
	/** Get Byte1 */
	temp1 = (uint8_t)((coeff2_4k & 0x0000FF00) >> 8);
	/** Get Byte2 */
	temp2 = (uint8_t)((coeff2_4k & 0x00FF0000) >> 16);
	/** Get Byte3 */
	temp3 = (uint8_t)((coeff2_4k & 0x01000000) >> 24);

	/** Big endian to make 8051 happy */
	buffer[cfoe_NS_4k_coeff2_24 - cfoe_NS_2048_coeff1_25_24] = temp3;
	buffer[cfoe_NS_4k_coeff2_23_16 - cfoe_NS_2048_coeff1_25_24] = temp2;
	buffer[cfoe_NS_4k_coeff2_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[cfoe_NS_4k_coeff2_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Get Byte0 */
	temp0 = (uint8_t)(bfsfcw_fftindex_ratio & 0x00FF);
	/** Get Byte1 */
	temp1 = (uint8_t)((bfsfcw_fftindex_ratio & 0xFF00) >> 8);

	/** Big endian to make 8051 happy */
	buffer[bfsfcw_fftindex_ratio_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[bfsfcw_fftindex_ratio_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	/** Get Byte0 */
	temp0 = (uint8_t)(fftindex_bfsfcw_ratio & 0x00FF);
	/** Get Byte1 */
	temp1 = (uint8_t)((fftindex_bfsfcw_ratio & 0xFF00) >> 8);

	/** Big endian to make 8051 happy */
	buffer[fftindex_bfsfcw_ratio_15_8 - cfoe_NS_2048_coeff1_25_24] = temp1;
	buffer[fftindex_bfsfcw_ratio_7_0 - cfoe_NS_2048_coeff1_25_24] = temp0;

	error = Standard_writeRegisters(demodulator, chip, Processor_OFDM, cfoe_NS_2048_coeff1_25_24, 36, buffer);


	if (error) goto exit;

exit:
	if (BWerror)
		error = BWerror;
	return (error);
}

uint32_t Standard_setFrequency (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint32_t           frequency
) {
	uint32_t error = Error_NO_ERROR;
	uint8_t band;
	uint8_t i;
	IT9130* it9130;
	it9130 = (IT9130*) demodulator;

	/** Clear easy mode flag first */
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, Training_Mode, 0x00);
	if (error) goto exit;

	/** Clear empty_channel_status lock flag */
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, empty_channel_status, 0x00);
	if (error) goto exit;

	/** Clear MPEG2 lock flag */
	error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, r_mp2if_sync_byte_locked, mp2if_sync_byte_locked_pos, mp2if_sync_byte_locked_len, 0x00);
	if (error) goto exit;

	/** Determine frequency band */
	band = 0xFF;
	for (i = 0; i < Standard_MAX_BAND; i++) {
		if ((frequency >= Standard_bandTable[i].minimum) && (frequency <= Standard_bandTable[i].maximum)) {
			band = i;
			break;
		}
	}
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, FreBand, band);
	if (error) goto exit;
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, var_pre_FreBand, band);
	if (error) goto exit;

	if(it9130->tunerDescription->tunerId >= 0x63 && it9130->tunerDescription->tunerId <= 0x65){
		error = Standard_writeRegister (demodulator, chip, Processor_OFDM, 0x019b, band);
		if (error) goto exit;
	}


//--------------------------
	if (it9130->tunerDescription->setTuner != NULL) {
		if ((it9130->busId != Bus_I2M) && (it9130->busId != Bus_I2U)) {
			if (it9130->chipNumber > 1 && chip == 0) {
				error = it9130->tunerDescription->setTuner (demodulator, chip, it9130->bandwidth[chip], frequency + 100);
				if (error) goto exit;
			} else if (it9130->chipNumber > 1 && chip == 1) {
				error = it9130->tunerDescription->setTuner (demodulator, chip, it9130->bandwidth[chip], frequency - 100);
				if (error) goto exit;
			} else {
				error = it9130->tunerDescription->setTuner (demodulator, chip, it9130->bandwidth[chip], frequency);
				if (error) goto exit;
			}
		}
	}

	/** Trigger ofsm */
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, trigger_ofsm, 0);
	if (error) goto exit;

	it9130->frequency[chip] = frequency;

exit :
	return (error);
}


uint32_t Standard_loadFirmware (
		Demodulator*    demodulator,
	  uint8_t*           firmwareCodes,
	  Segment*        firmwareSegments,
	  uint16_t*           firmwarePartitions
) {
	uint32_t error = Error_NO_ERROR;
	uint32_t beginPartition = 0;
	uint32_t endPartition = 0;
	uint32_t version;
	uint32_t firmwareLength;
	uint8_t* firmwareCodesPointer;
	uint16_t command;
	uint32_t i;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	/** Set I2C master clock speed. */
	error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_one_cycle_counter_tuner, User_I2C_SPEED);
	if (error) goto exit;

	firmwareCodesPointer = firmwareCodes;

#if (Firmware_FORMAT_VER1 == 1)
	beginPartition = 0;
	endPartition = firmwarePartitions[0];
#elif (Firmware_FORMAT_VER1 == 2)
	if (it9130->tunerDescription->tunerId == 0x23 || it9130->tunerDescription->tunerId == 0x2D) {
		beginPartition = firmwarePartitions[0];
		endPartition = firmwarePartitions[0] + firmwarePartitions[1];

		for (i = 0; i < beginPartition; i++) {
			firmwareLength = firmwareSegments[i].segmentLength;
			firmwareCodesPointer += firmwareLength;
		}
	} else {
		beginPartition = 0;
		endPartition = firmwarePartitions[0];
	}
#elif (Firmware_FORMAT_VER1 == 3)
	TunerGroup tunerGroup;
	Bool found = False;
	uint8_t groupIndex;

	/** Find the tuner group */
	for (i = 0; i < Firmware_TUNERGROUPLENGTH; i++) {
		tunerGroup = Firmware_tunerGroups[i];
		if (it9130->tunerDescription->tunerId == tunerGroup.tunerId) {
			found = True;
			break;
		}
	}

	/** Calculate begin/end partition */
	if (found) {
		groupIndex = tunerGroup.groupIndex;
	} else {
		groupIndex = 0;
	}
	for (i = 0; i < groupIndex; i++) {
		beginPartition += firmwarePartitions[i];
	}
	endPartition = beginPartition + firmwarePartitions[groupIndex];

	/** Calculate start pointer of firmware code */
	for (i = 0; i < beginPartition; i++) {
		firmwareLength = firmwareSegments[i].segmentLength;
		firmwareCodesPointer += firmwareLength;
	}
#endif

	for (i = beginPartition; i < endPartition; i++) {
		firmwareLength = firmwareSegments[i].segmentLength;
	printf("downloading firmware partition[%d]...(%d)\n", i,firmwareLength);
		if (firmwareSegments[i].segmentType == 0) {
			/** Dwonload firmware */
			error = Standard_sendCommand (demodulator, Command_FW_DOWNLOAD_BEGIN, 0, Processor_LINK, 0, NULL, 0, NULL);
			if (error) goto exit;
			if (Cmd_loadFirmware != NULL) {
				error = Cmd_loadFirmware ( firmwareLength, firmwareCodesPointer);
			}
			if (error) goto exit;
			error = Standard_sendCommand (demodulator, Command_FW_DOWNLOAD_END, 0, Processor_LINK, 0, NULL, 0, NULL);
			if (error) goto exit;
		} else if (firmwareSegments[i].segmentType == 1) {
			/** Copy firmware */
			error = Standard_sendCommand (demodulator, Command_SCATTER_WRITE, 0, Processor_LINK, firmwareLength, firmwareCodesPointer, 0, NULL);
			if (error) goto exit;
		} else {
			/** Direct write firmware */
			command = (uint16_t) (firmwareCodesPointer[0] << 8) + (uint16_t) firmwareCodesPointer[1];
			error = Standard_sendCommand (demodulator, command, 0, Processor_LINK, firmwareLength - 2, firmwareCodesPointer + 2, 0, NULL);
			if (error) goto exit;
		}
		firmwareCodesPointer += firmwareLength;
	}
	/** Boot */
	error = Standard_sendCommand (demodulator, Command_BOOT, 0, Processor_LINK, 0, NULL, 0, NULL);
	if (error) goto exit;

	User_delay ( 10);
	/** Check if firmware is running */
	version = 0;
	error = Standard_getFirmwareVersion (demodulator, Processor_LINK, &version);
	if (error) goto exit;
	if (version == 0)
		error = Error_BOOT_FAIL;

exit :
	return (error);
}


uint32_t Standard_loadScript (
		Demodulator*    demodulator,
	  StreamType      streamType,
	  uint16_t*           scriptSets,
	  ValueSet*       scripts,
	  uint16_t*           tunerScriptSets,
	  ValueSet*       tunerScripts
) {
	uint32_t error = Error_NO_ERROR;
	uint16_t beginScript;
	uint16_t endScript;
	uint8_t i, value1 = 0, value2 = 0, supportRelay = 0, chipNumber = 0, bufferLens = 1;
	uint16_t j;
	uint8_t buffer[20] = {0,};
	uint32_t tunerAddr, tunerAddrTemp;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;


	/** Querry SupportRelayCommandWrite **/
	error = Standard_readRegister (demodulator, 0, Processor_OFDM, 0x004D, &supportRelay);
	if (error) goto exit;

	if (supportRelay && it9130->chipNumber == 2)
		chipNumber = 1;
	else
		chipNumber = it9130->chipNumber;

	/** Enable RelayCommandWrite **/
	if (supportRelay) {
		error = Standard_writeRegister (demodulator, 0, Processor_OFDM, 0x004E, 1);
		if (error) goto exit;
	}

	if ((scriptSets[0] != 0) && (scripts != NULL)) {
		beginScript = 0;
		endScript = scriptSets[0];

		for (i = 0; i < chipNumber; i++) {
			/** Load OFSM init script */
			for (j = beginScript; j < endScript; j++) {
				tunerAddr = tunerAddrTemp = scripts[j].address;
				buffer[0] = scripts[j].value;

				while (j < endScript && bufferLens < 20) {
					tunerAddrTemp += 1;
					if (tunerAddrTemp != scripts[j+1].address)
						break;

					buffer[bufferLens] = scripts[j+1].value;
					bufferLens ++;
					j ++;
				}
		printf("loading OFSM script chunk[%d]...(%d)\n", j, bufferLens);
				error = Standard_writeRegisters (demodulator, i, Processor_OFDM, tunerAddr, bufferLens, buffer);
				if (error) goto exit;
				bufferLens = 1;
			}
		}
	}

	/** Distinguish chip type */
	error = Standard_readRegister (demodulator, 0, Processor_LINK, chip_version_7_0, &value1);
	if (error) goto exit;

	error = Standard_readRegister (demodulator, 0, Processor_LINK, prechip_version_7_0, &value2);
	if (error) goto exit;

	if ((tunerScriptSets[0] != 0) && (tunerScripts != NULL)) {
		if (tunerScriptSets[1] == tunerScriptSets[0] && !(value1 == 0xF8 && value2 == 0xEA)) {
			/** New version */
			beginScript = tunerScriptSets[0];
			endScript = tunerScriptSets[0] + tunerScriptSets[1];
		} else {
			/** Old version */
			beginScript = 0;
			endScript = tunerScriptSets[0];
		}

		for (i = 0; i < chipNumber; i++) {
			/** Load tuner init script */
			for (j = beginScript; j < endScript; j++) {
				tunerAddr = tunerAddrTemp = tunerScripts[j].address;
				buffer[0] = tunerScripts[j].value;

				while (j < endScript && bufferLens < 20) {
					tunerAddrTemp += 1;
					if (tunerAddrTemp != tunerScripts[j+1].address)
						break;

					buffer[bufferLens] = tunerScripts[j+1].value;
					bufferLens ++;
					j ++;
				}
		printf("loading Tuner script chunk[%d]...(%d)\n", j, bufferLens);
				error = Standard_writeRegisters (demodulator, i, Processor_OFDM, tunerAddr, bufferLens, buffer);
				if (error) goto exit;
				bufferLens = 1;
			}
		}
	}

	/** Disable RelayCommandWrite **/
	if (supportRelay) {
		error = Standard_writeRegister (demodulator, 0, Processor_OFDM, 0x004E, 0);
		if (error) goto exit;
	}

exit :
	return (error);
}

/** end of local functions */

uint32_t Standard_writeRegister (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	  uint8_t            value
) {
	return (Standard_writeRegisters (demodulator, chip, processor, registerAddress, 1, &value));
}


uint32_t Standard_writeRegisters (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	  uint8_t            bufferLength,
	  uint8_t*           buffer
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t registerAddressLength;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
			registerAddressLength = 2;
	}
	/*if (Cmd_writeRegisters != NULL)*/ {
		error = Cmd_writeRegisters ( chip, processor, registerAddress, registerAddressLength, bufferLength, buffer);
	}

	return (error);
}

uint32_t Standard_writeRegisterBits (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	  uint8_t            position,
	  uint8_t            length,
	  uint8_t            value
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t registerAddressLength;
	IT9130* it9130;
	uint8_t temp;

	it9130 = (IT9130*) demodulator;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
		registerAddressLength = 2;
	}
	if (length == 8) {
		/*if (Cmd_writeRegisters != NULL)*/ {
			error = Cmd_writeRegisters (chip, processor, registerAddress, registerAddressLength, 1, &value);
		}
	} else {
		/*if (Cmd_readRegisters != NULL)*/ {
			error = Cmd_readRegisters ( chip, processor, registerAddress, registerAddressLength, 1, &temp);
			if (error) goto exit;
		}

		temp = (uint8_t)REG_CREATE (value, temp, position, length);

		/*if (Cmd_writeRegisters != NULL)*/ {
			error = Cmd_writeRegisters ( chip, processor, registerAddress, registerAddressLength, 1, &temp);
			if (error) goto exit;
		}
	}
exit:

	return (error);
}

uint32_t Standard_readRegister (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	 uint8_t*           value
) {
	return (Standard_readRegisters (demodulator, chip, processor, registerAddress, 1, value));
}


uint32_t Standard_readRegisters (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	  uint8_t            bufferLength,
	 uint8_t*           buffer
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t registerAddressLength;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
		registerAddressLength = 2;
	}
	/*if (Cmd_readRegisters != NULL)*/ {
		error = Cmd_readRegisters ( chip, processor, registerAddress, registerAddressLength, bufferLength, buffer);
	}

	return (error);
}

uint32_t Standard_readRegisterBits (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Processor       processor,
	  uint32_t           registerAddress,
	  uint8_t            position,
	  uint8_t            length,
	  uint8_t*           value
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp = 0;
	uint8_t registerAddressLength;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
		registerAddressLength = 2;
	}
	/*if (Cmd_readRegisters != NULL) */{
		error = Cmd_readRegisters ( chip, processor, registerAddress, registerAddressLength, 1, &temp);
	}
	if (error) goto exit;

	if (length == 8) {
		*value = temp;
	} else {
		temp = REG_GET (temp, position, length);
		*value = temp;
	}


exit :
	return (error);
}

uint32_t Standard_getFirmwareVersion (
		Demodulator*    demodulator,
	  Processor       processor,
	 uint32_t*          version
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t writeBuffer[1] = {0,};
	uint8_t readBuffer[4] = {0,};
	uint8_t value = 0;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;
	if ((it9130->busId == Bus_I2M) || (it9130->busId == Bus_I2U)) {
		*version = 0xFFFFFFFF;
		goto exit;
	}
	/** Check chip version */
	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0,&value);
	if (error) goto exit;

	if (value == 0xF8 || User_MAX_PKT_SIZE > 9) {
		/** Old version */
		writeBuffer[0] = 1;
		error = Standard_sendCommand (demodulator, Command_QUERYINFO, 0, processor, 1, writeBuffer, 4, readBuffer);
		if (error) goto exit;
	} else {
		/** New version */
		error = Standard_sendCommand (demodulator, Command_FW_DOWNLOAD_END, 0, Processor_LINK, 0, NULL, 0, NULL);
		if (error == 0x01000009) { /* Boot code*/
			readBuffer[0] = readBuffer[1] = readBuffer[2] = readBuffer[3] = 0;
			error = 0;
		} else if (error == 0x010000FA) { /* Firmware code*/
			if (processor == Processor_LINK)
			{
				error = Standard_readRegisters (demodulator, 0, Processor_LINK, link_version_31_24, 1, readBuffer);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_LINK, link_version_23_16, 1, readBuffer + 1);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_LINK, link_version_15_8, 1, readBuffer + 2);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_LINK, link_version_7_0, 1, readBuffer + 3);
				if (error) goto exit;
			}
			else
			{
				error = Standard_readRegisters (demodulator, 0, Processor_OFDM, ofdm_version_31_24, 1, readBuffer);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_OFDM, ofdm_version_23_16, 1, readBuffer + 1);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_OFDM, ofdm_version_15_8, 1, readBuffer + 2);
				if (error) goto exit;

				error = Standard_readRegisters (demodulator, 0, Processor_OFDM, ofdm_version_7_0, 1, readBuffer + 3);
				if (error) goto exit;
			}
		} else /* error */
			goto exit;
	}

	*version = (uint32_t) (((uint32_t) readBuffer[0] << 24) + ((uint32_t) readBuffer[1] << 16) + ((uint32_t) readBuffer[2] << 8) + (uint32_t) readBuffer[3]);

exit :

	return (error);
}


uint32_t Standard_getPostVitBer (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint32_t*          postErrorCount,  /** 24 bits */
	 uint32_t*          postBitCount,    /** 16 bits */
	 uint16_t*           abortCount
) {
	uint32_t error = Error_NO_ERROR;
	uint32_t errorCount;
	uint32_t bitCount;
	uint8_t buffer[7];
	uint16_t abort;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	*postErrorCount = 0;
	*postBitCount  = 0;

	error = Standard_readRegisters (demodulator, chip, Processor_OFDM, rsd_abort_packet_cnt_7_0, r_rsd_packet_unit_15_8 - rsd_abort_packet_cnt_7_0 + 1, buffer);
	if (error) goto exit;

	abort = ((uint16_t) buffer[rsd_abort_packet_cnt_15_8 - rsd_abort_packet_cnt_7_0] << 8) + buffer[rsd_abort_packet_cnt_7_0 - rsd_abort_packet_cnt_7_0];
	errorCount = ((uint32_t) buffer[rsd_bit_err_cnt_23_16 - rsd_abort_packet_cnt_7_0] << 16) + ((uint32_t) buffer[rsd_bit_err_cnt_15_8 - rsd_abort_packet_cnt_7_0] << 8) + buffer[rsd_bit_err_cnt_7_0 - rsd_abort_packet_cnt_7_0];
	bitCount = ((uint32_t) buffer[r_rsd_packet_unit_15_8 - rsd_abort_packet_cnt_7_0] << 8) + buffer[r_rsd_packet_unit_7_0 - rsd_abort_packet_cnt_7_0];
	if (bitCount == 0) {
		/*error = Error_RSD_PKT_CNT_0;*/
		*postErrorCount = 1;
		*postBitCount = 2;
		*abortCount = 1000;
		goto exit;
	}

	*abortCount = abort;
	bitCount = bitCount;
	if (bitCount == 0) {
		*postErrorCount = 1;
		*postBitCount  = 2;
	} else {
		*postErrorCount = errorCount ;
		*postBitCount  = bitCount * 204 * 8;
	}

exit :
	return (error);
}


uint32_t Standard_getRfAgcGain (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           rfAgc
) {
	uint32_t   error = Error_NO_ERROR;

	/** get rf_agc_control */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, r_reg_aagc_rf_gain, reg_aagc_rf_gain_pos, reg_aagc_rf_gain_len, rfAgc);

	return (error);
}


uint32_t Standard_getIfAgcGain (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           ifAgc
) {
	uint32_t error = Error_NO_ERROR;

	/** get if_agc_control */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, r_reg_aagc_if_gain, reg_aagc_if_gain_pos, reg_aagc_if_gain_len, ifAgc);

	return (error);
}


uint32_t Standard_getSignalQuality (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           quality
) {
	uint32_t error = Error_NO_ERROR;
	Booll     locked;

    error = Standard_isMpeg2Locked (demodulator, chip, &locked);
	if (error) goto exit;

	if (locked != True)
        *quality = 0;
    else
	    error = Standard_readRegister (demodulator, chip, Processor_OFDM, signal_quality, quality);
exit:
	return (error);
}


uint32_t Standard_getSignalQualityIndication (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           quality
) {
	uint32_t error = Error_NO_ERROR;
    uint8_t SQI = 0;
	ChannelModulation    channelModulation;
	uint8_t CNrec = 0;
	uint8_t BER_SQI = 0;
	uint32_t postErrorCount=0;
    uint32_t postBitCount=0;
    uint16_t  abortCount=0;

	uint32_t invBER = 0;
	int CNrel_ten = 0;

	IT9130* it9130;
	Booll          locked;
	it9130 = (IT9130*) demodulator;


    error = Standard_isMpeg2Locked (demodulator, chip, &locked);
	if (error) goto exit;

	if (locked != True){
		SQI = 0;
        *quality = 0;
		goto exit;
	}

	error= Standard_getChannelModulation(demodulator, chip, &channelModulation);
    if (error) goto exit;

	Standard_getSNR (demodulator, chip, &CNrec);
	if(CNrec>15)
		CNrec = CNrec - 2;

	if(channelModulation.hierarchy == 0){
		if(channelModulation.priority == Priority_HIGH)
			CNrel_ten = CNrec*10 - Standard_CNRequiredValues[channelModulation.constellation][channelModulation.highCodeRate];
		else
		   CNrel_ten = CNrec*10 - Standard_CNRequiredValues[channelModulation.constellation][channelModulation.lowCodeRate];
	}else{//Hierarchical mode

		if(channelModulation.constellation == 1){ //16QAM
			if(channelModulation.priority == Priority_HIGH)
				CNrel_ten = CNrec*10 - Standard_HierarchicalCNRequiredValues[0][channelModulation.highCodeRate][channelModulation.hierarchy-2];
			else
				CNrel_ten = CNrec*10 - Standard_HierarchicalCNRequiredValues[0][channelModulation.lowCodeRate][channelModulation.hierarchy];

		}else if(channelModulation.constellation == 2){ //64QAM
			if(channelModulation.priority == Priority_HIGH)
				CNrel_ten = CNrec*10 - Standard_HierarchicalCNRequiredValues[1][channelModulation.highCodeRate][channelModulation.hierarchy-1];
			else
				CNrel_ten = CNrec*10 - Standard_HierarchicalCNRequiredValues[1][channelModulation.lowCodeRate][channelModulation.hierarchy+1];

		}else
			goto exit;
	}
		error = Standard_getPostVitBer (demodulator, chip, &postErrorCount, &postBitCount, &abortCount);
	if (error) {
		goto exit;
	} else {

		if(postErrorCount!=0)
			invBER = postBitCount/postErrorCount;
		else
			invBER = 10000001;
    }

	if(invBER < 1000)
		BER_SQI = 0;
	else if(invBER >= 1000 && invBER < 10000000)
	{
		if(invBER < 1245) //3.0
			BER_SQI = 20 ;
		else if (invBER >= 1245 && invBER < 1567) //3.1
			BER_SQI = 22 ;
		else if (invBER >= 1567 && invBER < 1973) //3.2
			BER_SQI = 24 ;
		else if (invBER >= 1973 && invBER < 2484) //3.3
			BER_SQI = 26 ;
		else if (invBER >= 2484 && invBER < 3127) //3.4
			BER_SQI = 28 ;
		else if (invBER >= 3127 && invBER < 3936) //3.5
			BER_SQI = 30 ;
		else if (invBER >= 3936 && invBER < 4955) //3.6
			BER_SQI = 32 ;
		else if (invBER >= 4955 && invBER < 6238) //3.7
			BER_SQI = 34 ;
		else if (invBER >= 6238 && invBER < 7853) //3.8
			BER_SQI = 36 ;
		else if (invBER >= 7853 && invBER < 9886) //3.9
			BER_SQI = 38 ;
		else if (invBER >= 9886 && invBER < 12446) //4.0
			BER_SQI = 40 ;
		else if (invBER >= 12446 && invBER < 15668) //4.1
			BER_SQI = 42 ;
		else if (invBER >= 15668 && invBER < 19725) //4.2
			BER_SQI = 44 ;
		else if (invBER >= 19725 && invBER < 24832) //4.3
			BER_SQI = 46 ;
		else if (invBER >= 24832 && invBER < 31261) //4.4
			BER_SQI = 48 ;
		else if (invBER >= 31261 && invBER < 39356) //4.5
			BER_SQI = 50 ;
		else if (invBER >= 39356 && invBER < 49546) //4.6
			BER_SQI = 52 ;
		else if (invBER >= 49546 && invBER < 62374) //4.7
			BER_SQI = 54 ;
		else if (invBER >= 62374 && invBER < 78524) //4.8
			BER_SQI = 56 ;
		else if (invBER >= 78524 && invBER < 98856) //4.9
			BER_SQI = 58 ;
		else if (invBER >= 98856 && invBER < 124452) //5.0
			BER_SQI = 60 ;
		else if (invBER >= 124452 && invBER < 156676) //5.1
			BER_SQI = 62 ;
		else if (invBER >= 156676 && invBER < 197243) //5.2
			BER_SQI = 64 ;
		else if (invBER >= 197243 && invBER < 248314) //5.3
			BER_SQI = 66 ;
		else if (invBER >= 248314 && invBER < 312609) //5.4
			BER_SQI = 68 ;
		else if (invBER >= 312609 && invBER < 393551) //5.5
			BER_SQI = 70 ;
		else if (invBER >= 393551 && invBER < 495451) //5.6
			BER_SQI = 72 ;
		else if (invBER >= 495451 && invBER < 623735) //5.7
			BER_SQI = 74 ;
		else if (invBER >= 623735 && invBER < 785237) //5.8
			BER_SQI = 76 ;
		else if (invBER >= 785237 && invBER < 988554) //5.9
			BER_SQI = 78 ;
		else if (invBER >= 988554 && invBER < 1244515) //6.0
			BER_SQI = 80 ;
		else if (invBER >= 1244515 && invBER < 1566751) //6.1
			BER_SQI = 82 ;
		else if (invBER >= 1566751 && invBER < 1972422) //6.2
			BER_SQI = 84 ;
		else if (invBER >= 1972422 && invBER < 2483135) //6.3
			BER_SQI = 86 ;
		else if (invBER >= 2483135 && invBER < 3126081) //6.5
			BER_SQI = 88 ;
		else if (invBER >= 3126081 && invBER < 3935502) //6.6
			BER_SQI = 90 ;
		else if (invBER >= 3935502 && invBER < 4954502) //6.7
			BER_SQI = 92 ;
		else if (invBER >= 4954502 && invBER < 6237347) //6.8
			BER_SQI = 94 ;
		else if (invBER >= 6237347 && invBER < 7852361) //6.9
			BER_SQI = 96 ;
		else if (invBER >= 7852361 && invBER < 9885534) //7.0
			BER_SQI = 98 ;
		else
			BER_SQI = 100 ;
	}
	else if(invBER > 10000000)
		BER_SQI = 100;

	if(CNrel_ten<-70)
		SQI = 0;
	else if(-70 <= CNrel_ten && CNrel_ten < 30)
	{
		SQI = (uint8_t)((( (CNrel_ten -30) + 100) * BER_SQI)/100);
	}
	else if(CNrel_ten >= 30)
		SQI = (uint8_t)BER_SQI;


	if(SQI){
		if(SQI == 0 || SQI == 100)
			it9130->Pre_SQI[chip] = SQI;
			SQI = (it9130->Pre_SQI[chip] + SQI)/2;
	}
			it9130->Pre_SQI[chip] = SQI;

exit:
    *quality = SQI;
	return (error);
}

uint32_t Standard_getSignalStrengthIndication (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           strength
) {
	uint32_t error = Error_NO_ERROR;
	uint8_t temp,LNAGain_offset;
	ChannelModulation    channelModulation;
	int Prec,Pref,Prel;

	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister (demodulator, chip, Processor_OFDM, var_p_inband, &temp);

	if (error) goto exit;

	error= Standard_getChannelModulation(demodulator, chip, &channelModulation);
    if (error) goto exit;

	if(it9130->tunerDescription->tunerId== 0x63){ //IT9163
		if(channelModulation.frequency > 700000)
			LNAGain_offset = 4;
		else
			LNAGain_offset = 7;
	}else{
		if(channelModulation.frequency < 300000)
			LNAGain_offset = 7;  //VHF
		else
			LNAGain_offset = 14; //UHF
	}
    Prec = (temp - 100 ) - LNAGain_offset;

    if(channelModulation.priority == Priority_HIGH)
       Pref = Standard_PrefValues[channelModulation.constellation][channelModulation.highCodeRate];
    else
       Pref = Standard_PrefValues[channelModulation.constellation][channelModulation.lowCodeRate];

     Prel = Prec - Pref;

     if(Prel<-15){
       *strength = 0;
     }else if((-15 <= Prel)&&(Prel < 0)){
       *strength = (uint8_t)((2 * (Prel + 15))/3);
     }else if((0 <= Prel)&&(Prel < 20)){
       *strength = (uint8_t)( 4 * Prel + 10);
     }else if((20 <= Prel)&&(Prel < 35)){
       *strength = ((uint8_t)(2 * (Prel - 20))/3 + 90);
     }else if(Prel >= 35){
       *strength = 100;
     }

exit :
	return (error);
}
uint32_t Standard_getSignalStrength (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 uint8_t*           strength
) {
	uint32_t error = Error_NO_ERROR;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;


	error = Standard_readRegister (demodulator, chip, Processor_OFDM, var_signal_strength, strength);



	return (error);
}

uint32_t Standard_getSignalStrengthDbm (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 Long*           strengthDbm           /** DBm*/
)
{
	uint32_t error = Error_NO_ERROR;
	uint8_t temp;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister (demodulator, chip, Processor_OFDM, var_p_inband, &temp);
	if (error) goto exit;
	*strengthDbm = (Long) (temp - 100);

exit :
	return (error);
}


uint32_t Standard_initialize (
		Demodulator*    demodulator,
	  uint8_t            chipNumber,
	  uint16_t            sawBandwidth,
	  StreamType      streamType,
	  Architecture    architecture
) {
	uint32_t error = Error_NO_ERROR;

	uint32_t crystal = 0;
	uint32_t adc = 0;
	uint32_t fcw = 0;
	uint8_t buffer[4];
	uint32_t version = 0;
	uint8_t chip_version = 0;
	uint16_t* tunerScriptSets = NULL;
	ValueSet* tunerScripts = NULL;
	uint8_t i;
	IT9130 *it9130;

	uint32_t chip_Type;
	uint8_t var[2];
//	uint8_t temp = 0;

	it9130 = (IT9130*) demodulator;

	it9130->chipNumber = chipNumber;
	it9130->options = 0x0000;
	it9130->fcw = 0x00000000;
	it9130->frequency[0] = 642000;
	it9130->frequency[1] = 642000;
	it9130->initialized = False;
	it9130->hostInterface[0] = 0;


	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];

	it9130->busId = Bus_I2C;
	if (it9130->busId == 0xFFFF) {
		goto exit;
	}

	it9130->tunerDescription = &tunerDescription;
	if (it9130->tunerDescription->tunerId == 0xFFFF) {
		goto exit;
	} else if(chip_Type==0x9135 || chip_Type==0x9175){
		Standard_clockTable[0].crystalFrequency = 12000;
		Standard_clockTable[0].adcFrequency = 20250000;
		Standard_clockTable[1].crystalFrequency = 20480;
		Standard_clockTable[1].adcFrequency = 20480000;
	}
	error = Standard_getFirmwareVersion (demodulator, Processor_LINK, &version);
	if (error) goto exit;

	if (version != 0) {
		it9130->booted = True;
		//it9130->alreadyBoot = False;
	} else {
		it9130->booted = False;
		//it9130->alreadyBoot = False;
	}


	if(chip_Type==0x9135 && chip_version == 2){

		if(it9130->tunerDescription->tunerId == 0x64 || it9130->tunerDescription->tunerId == 0x65){
			it9130->firmwareCodes = FirmwareV2I_codes;
			it9130->firmwareSegments = FirmwareV2I_segments;
			it9130->firmwarePartitions = FirmwareV2I_partitions;
			it9130->scriptSets = FirmwareV2I_scriptSets;
			it9130->scripts = FirmwareV2I_scripts;
		}else if(it9130->tunerDescription->tunerId == 0x63){
			it9130->firmwareCodes = FirmwareV2W_codes;
			it9130->firmwareSegments = FirmwareV2W_segments;
			it9130->firmwarePartitions = FirmwareV2W_partitions;
			it9130->scriptSets = FirmwareV2W_scriptSets;
			it9130->scripts = FirmwareV2W_scripts;

		}else{
			it9130->firmwareCodes = FirmwareV2_codes;
			it9130->firmwareSegments = FirmwareV2_segments;
			it9130->firmwarePartitions = FirmwareV2_partitions;
			it9130->scriptSets = FirmwareV2_scriptSets;
			it9130->scripts = FirmwareV2_scripts;
       	}

	}else{
		it9130->firmwareCodes = Firmware_codes;
		it9130->firmwareSegments = Firmware_segments;
		it9130->firmwarePartitions = Firmware_partitions;
		it9130->scriptSets = Firmware_scriptSets;
		it9130->scripts = Firmware_scripts;
	}
	if(it9130->tunerDescription == NULL)
	{
		tunerScriptSets = 0;
		tunerScripts = NULL;
	}else{
		tunerScriptSets = it9130->tunerDescription->tunerScriptSets;
		tunerScripts = it9130->tunerDescription->tunerScript;
	}


	it9130->crystalFrequency = 12000;//Standard_clockTable[i].crystalFrequency;
	it9130->adcFrequency = 20250000;//Standard_clockTable[i].adcFrequency;

	it9130->dataReady = False;


	/** Write secondary I2C address to device */
	/** Enable or disable clock out for 2nd chip power saving */
	if (it9130->chipNumber > 1) {
		error = Standard_writeRegister (demodulator, 0, Processor_LINK, second_i2c_address, Chip2_I2c_address);
		if (error) goto exit;

        error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_top_clkoen, 1);
		if (error) goto exit;

	} else {
		error = Standard_writeRegister (demodulator, 0, Processor_LINK, second_i2c_address, 0x00);
		if (error) goto exit;
		error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_top_clkoen, 0);
		if (error) goto exit;
	}
	/** Load firmware */
	if (it9130->firmwareCodes != NULL) {
		if (it9130->booted == False) {
			error = Standard_loadFirmware (demodulator, it9130->firmwareCodes, it9130->firmwareSegments, it9130->firmwarePartitions);
			if (error) goto exit;
			it9130->booted = True;
		}
	}
	/** Set I2C master clock 100k in order to support tuner I2C. */
	error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_one_cycle_counter_tuner, 0x1a);
	if (error) goto exit;


	for (i = 0; i < it9130->chipNumber; i++) {
		error = Standard_writeRegister(demodulator, i, Processor_OFDM, 0xEC4C, 0x68);
		if (error) goto exit;

		User_delay ( 10);
	}

	if(chip_Type==0x9135 && chip_version == 1){
		/** Open tuner */
		for (i = 0; i < it9130->chipNumber; i++) {

			/** Set 0xD827 to 0 as open drain for tuner i2c */
			error = Standard_writeRegister (demodulator, i ,  Processor_LINK, p_reg_top_padodpu, 0);
			if (error) goto exit;

			/** Set 0xD829 to 0 as push pull for tuner AGC */
			error = Standard_writeRegister (demodulator, i ,  Processor_LINK, p_reg_top_agc_od, 0);
			if (error) goto exit;
		}
		if (it9130->tunerDescription->openTuner != NULL) {
			if ((it9130->busId != Bus_I2M) && (it9130->busId != Bus_I2U)) {
				for (i = 0; i < it9130->chipNumber; i++) {
					error = it9130->tunerDescription->openTuner (demodulator, i);
					if (error) goto exit;
				}
			}
		}
	}
	for (i = 0; i < it9130->chipNumber; i++) {
		/** Tell firmware the type of tuner. */
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_link_ofsm_dummy_15_8, (uint8_t) it9130->tunerDescription->tunerId);
		if (error) goto exit;
	}
	/** Initialize OFDM */
	if (it9130->booted == True) {
		for (i = 0; i < it9130->chipNumber; i++) {
			/** Set read-update bit to 1 for constellation */
			error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_feq_read_update, reg_feq_read_update_pos, reg_feq_read_update_len, 1);
			if (error) goto exit;

			/** Enable FEC Monitor */
			error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_fec_vtb_rsd_mon_en, fec_vtb_rsd_mon_en_pos, fec_vtb_rsd_mon_en_len, 1);
			if (error) goto exit;
		}

		/** Compute ADC and load them to device */
		error = Standard_computeCrystal (demodulator, (Long) it9130->crystalFrequency * 1000, &crystal);
		if (error) goto exit;

		buffer[0] = (uint8_t) (crystal & 0x000000FF);
		buffer[1] = (uint8_t) ((crystal & 0x0000FF00) >> 8);
		buffer[2] = (uint8_t) ((crystal & 0x00FF0000) >> 16);
		buffer[3] = (uint8_t) ((crystal & 0xFF000000) >> 24);
		for (i = 0; i < it9130->chipNumber; i++) {
			error = Standard_writeRegisters (demodulator, i, Processor_OFDM, crystal_clk_7_0, 4, buffer);
			if (error) goto exit;
		}
	}

	/** Load script */
	if (it9130->scripts != NULL) {
		error = Standard_loadScript (demodulator, streamType, it9130->scriptSets, it9130->scripts, tunerScriptSets, tunerScripts);
		if (error) goto exit;
	}

	if((chip_Type==0x9135 && chip_version == 2) || chip_Type==0x9175){
		for (i = 0; i< it9130->chipNumber; i++) {
			error = Standard_writeRegister (demodulator, i ,  Processor_OFDM, trigger_ofsm, 0x01);
			if (error) goto exit;
		}
		/** Open tuner */
		for (i = 0; i < it9130->chipNumber; i++) {

			/** Set 0xD827 to 0 as open drain for tuner i2c */
			error = Standard_writeRegister (demodulator, i ,  Processor_LINK, p_reg_top_padodpu, 0);
			if (error) goto exit;
			/** Set 0xD829 to 0 as push pull for tuner AGC */
			error = Standard_writeRegister (demodulator, i ,  Processor_LINK, p_reg_top_agc_od, 0);
			if (error) goto exit;

		}
		if (it9130->tunerDescription->openTuner != NULL) {
			if ((it9130->busId != Bus_I2M) && (it9130->busId != Bus_I2U)) {
				for (i = 0; i < it9130->chipNumber; i++) {
					error = it9130->tunerDescription->openTuner (demodulator, i);
					if (error) goto exit;
				}
			}
		}
	}

	/** Set the desired stream type */
	error = Standard_setStreamType (demodulator, streamType);
	if (error) goto exit;
	/** Set the desired architecture type */
	error = Standard_setArchitecture (demodulator, architecture);
	if (error) goto exit;

	for (i = 0; i< it9130->chipNumber; i++) {
		/** Set H/W MPEG2 locked detection **/
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_top_lock3_out, 1);
		if (error) goto exit;
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_top_padmiscdrsr, 1);
		if (error) goto exit;
		/** Set registers for driving power 0xD830 **/
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_top_padmiscdr2, 0);
		if (error) goto exit;
		/** enhance the performance while using DIP crystals **/
		error = Standard_writeRegister (demodulator, i, Processor_OFDM, 0xEC57, 0);
		if (error) goto exit;
		error = Standard_writeRegister (demodulator, i, Processor_OFDM, 0xEC58, 0);
		if (error) goto exit;
		/** Set ADC frequency multiplier **/
		error = Standard_setMultiplier(demodulator, Multiplier_2X);
		if (error) goto exit;

		if(it9130->tunerDescription->tunerId >= 0x63 && it9130->tunerDescription->tunerId <= 0x65){

			if (it9130->tunerDescription->tunerId != 0x63){
				error = Standard_initDetectTable(demodulator, i);
				if (error) goto exit;
			}
			error = Standard_initDecryptTable(demodulator, i);
			if (error) goto exit;
		}
		/** Set registers for driving power 0xD831 **/
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_top_padmiscdr4, 0);
		if (error) goto exit;

		/** Set registers for driving power 0xD832 **/
		error = Standard_writeRegister (demodulator, i, Processor_LINK, p_reg_top_padmiscdr8, 0);
		if (error) goto exit;
   }

	it9130->initialized = True;
exit:

	return (error);
}

uint32_t Standard_finalize (
		Demodulator*    demodulator
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t i;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (it9130->tunerDescription->closeTuner != NULL) {
		for (i = 0; i < it9130->chipNumber; i++) {
			error = it9130->tunerDescription->closeTuner ( i);
		}
	}


	return (error);
}

uint32_t Standard_isTpsLocked (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 Booll*           locked
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp;

	*locked = False;

	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, p_fd_tpsd_lock, fd_tpsd_lock_pos, fd_tpsd_lock_len, &temp);
	if (error) goto exit;
	if (temp) *locked = True;

exit :

	return (error);
}


uint32_t Standard_isMpeg2Locked (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 Booll*           locked
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp;
	uint8_t lock = 0;


	*locked = False;
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, r_mp2if_sync_byte_locked, mp2if_sync_byte_locked_pos, mp2if_sync_byte_locked_len, &temp);
	if (error) goto exit;
	if (temp) *locked = True;
exit :

	return (error);
}

uint32_t Standard_isLocked (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 Booll*           locked
) {
	uint32_t error = Error_NO_ERROR;

	uint16_t emptyLoop = 0;
	uint16_t tpsLoop = 0;
	uint16_t mpeg2Loop = 0;
	uint8_t channels[2];
	uint8_t begin;
	uint8_t end;
	uint8_t i;
	uint8_t emptyChannel = 1;
	uint8_t tpsLocked = 0;

	IT9130* it9130;


	Booll retry = True;
	it9130 = (IT9130*) demodulator;
check_lock:

	*locked = False;

	if ((it9130->architecture == Architecture_DCA) || (it9130->architecture == Architecture_DCAEx)){
		begin = 0;
		end = it9130->chipNumber;
	} else {
		begin = chip;
		end = begin + 1;
	}

	for (i = begin; i < end; i++) {
		it9130->statistic[i].signalPresented = False;
		it9130->statistic[i].signalLocked = False;
		it9130->statistic[i].signalQuality = 0;
		it9130->statistic[i].signalStrength = 0;
	}

	channels[0] = 2;
	channels[1] = 2;
	while (emptyLoop < 40) {
		for (i = begin; i < end; i++) {
			error = Standard_readRegister (demodulator, i, Processor_OFDM, empty_channel_status, &channels[i]);
			if (error) goto exit;
		}
		if ((channels[0] == 1) || (channels[1] == 1)) {
			emptyChannel = 0;
			break;
		}
		if ((channels[0] == 2) && (channels[1] == 2)) {
			emptyChannel = 1;
			goto exit;
		}
		User_delay ( 25);
		emptyLoop++;
	}

	if (emptyChannel == 1) goto exit;

	while (tpsLoop < 50) {
		for (i = begin; i < end; i++) {
			/** TPS check */
			error = Standard_isTpsLocked (demodulator, i, &it9130->statistic[i].signalPresented);
			if (error) goto exit;
			if (it9130->statistic[i].signalPresented == True) {
				tpsLocked = 1;
				break;
			}
		}

		if (tpsLocked == 1) break;

		User_delay ( 25);
		tpsLoop++;
	}

	if (tpsLocked == 0) goto exit;

	while (mpeg2Loop < 40) {
		if ((it9130->architecture == Architecture_DCA) || (it9130->architecture == Architecture_DCAEx)) {
			error = Standard_isMpeg2Locked (demodulator, 0, &it9130->statistic[0].signalLocked);
			if (error) goto exit;
			if (it9130->statistic[0].signalLocked == True) {
				for (i = begin; i < end; i++) {
					it9130->statistic[i].signalQuality = 80;
					it9130->statistic[i].signalStrength = 80;
				}
				*locked = True;
				break;
			}
		} else {
   			error = Standard_isMpeg2Locked (demodulator, chip, &it9130->statistic[chip].signalLocked);
			if (error) goto exit;
			if (it9130->statistic[chip].signalLocked == True) {
				it9130->statistic[chip].signalQuality = 80;
				it9130->statistic[chip].signalStrength = 80;
				*locked = True;
				break;
			}
		}
		User_delay ( 25);
		mpeg2Loop++;
	}
	for (i = begin; i < end; i++) {
		it9130->statistic[i].signalQuality = 0;
		it9130->statistic[i].signalStrength = 20;
	}

exit:

	if(*locked == False && retry == True){

        retry=False;
        mpeg2Loop = 0;
	    tpsLoop = 0;
        emptyLoop = 0;
		emptyChannel = 1;
    	tpsLocked = 0;
        // Clear empty_channel_status lock flag
    	error = Standard_writeRegister (demodulator, chip,Processor_OFDM, empty_channel_status, 0x00);
    	if (error) goto exit;

        //Trigger ofsm /
    	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, trigger_ofsm, 0);
    	if (error) goto exit;
        goto check_lock;
    }


	return (error);
}

uint32_t Standard_reset (
		Demodulator*    demodulator
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t value;
	uint8_t i;
	uint8_t j;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	for (i = 0; i < it9130->chipNumber; i++) {

		/** Enable OFDM reset */
		error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, I2C_reg_ofdm_rst_en, reg_ofdm_rst_en_pos, reg_ofdm_rst_en_len, 0x01);
		if (error) goto exit;

		/** Start reset mechanism */
		value = 0x00;
		error = Standard_writeRegisters (demodulator, i, Processor_OFDM, RESET_STATE, 1, &value);
		if (error) goto exit;

		/** Clear ofdm reset */
		for (j = 0; j < 150; j++) {
			error = Standard_readRegisterBits (demodulator, i, Processor_OFDM, I2C_reg_ofdm_rst, reg_ofdm_rst_pos, reg_ofdm_rst_len, &value);
			if (error) goto exit;
			if (value) break;
			User_delay (10);
		}

		if (j == 150) {
			error = Error_RESET_TIMEOUT;
			goto exit;
		}

		error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, I2C_reg_ofdm_rst, reg_ofdm_rst_pos, reg_ofdm_rst_len, 0);
		if (error) goto exit;

		/** Disable OFDM reset */
		error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, I2C_reg_ofdm_rst_en, reg_ofdm_rst_en_pos, reg_ofdm_rst_en_len, 0x00);
		if (error) goto exit;
	}

exit :

	return (error);
}


uint32_t Standard_getChannelModulation (
		Demodulator*    demodulator,
	  uint8_t                    chip,
	 ChannelModulation*      channelModulation
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp;
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	/** Get constellation type */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_const, reg_tpsd_const_pos, reg_tpsd_const_len, &temp);
	if (error) goto exit;
	channelModulation->constellation = (Constellation) temp;

	/** Get TPS hierachy and alpha value */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_hier, reg_tpsd_hier_pos, reg_tpsd_hier_len, &temp);
	if (error) goto exit;
	channelModulation->hierarchy = (Hierarchy)temp;

	/** Get high/low priority */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_dec_pri, reg_dec_pri_pos, reg_dec_pri_len, &temp);
	if (error) goto exit;
	if (temp)
		channelModulation->priority = Priority_HIGH;
	else
		channelModulation->priority = Priority_LOW;

	/** Get high code rate */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_hpcr, reg_tpsd_hpcr_pos, reg_tpsd_hpcr_len, &temp);
	if (error) goto exit;
	channelModulation->highCodeRate = (CodeRate) temp;

	/** Get low code rate */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_lpcr, reg_tpsd_lpcr_pos, reg_tpsd_lpcr_len, &temp);
	if (error) goto exit;
	channelModulation->lowCodeRate  = (CodeRate) temp;

	/** Get guard interval */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_gi, reg_tpsd_gi_pos, reg_tpsd_gi_len, &temp);
	if (error) goto exit;
	channelModulation->interval = (Interval) temp;

	/** Get FFT mode */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_tpsd_txmod, reg_tpsd_txmod_pos, reg_tpsd_txmod_len, &temp);
	if (error) goto exit;
	channelModulation->transmissionMode = (TransmissionModes) temp;

	/** Get bandwidth */
	error = Standard_readRegisterBits (demodulator, chip, Processor_OFDM, g_reg_bw, reg_bw_pos, reg_bw_len, &temp);
	if (error) goto exit;
	channelModulation->bandwidth = (Bandwidth) temp;

	/** Get frequency */
	channelModulation->frequency = it9130->frequency[chip];

exit :

	return (error);
}

uint32_t Standard_acquireChannel (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint16_t            bandwidth,
	  uint32_t           frequency
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t begin;
	uint8_t end;
	uint8_t i;

	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	if (it9130->architecture == Architecture_DCA) {
		begin = 0;
		end = it9130->chipNumber;
	} else {
		begin = chip;
		end = begin + 1;
	}


	for (i = begin; i < end; i++) {
		error = Standard_selectBandwidth (demodulator, i, bandwidth, it9130->adcFrequency);
		if (error) goto exit;
		it9130->bandwidth[i] = bandwidth;
	}
	/*
	if ((it9130->architecture == Architecture_DCAEx) && (chip == 1)){

	}
	else{
		error = Standard_maskDcaOutput(demodulator);
	}
	*/

	if (it9130->architecture != Architecture_DCAEx) {
		error = Standard_maskDcaOutput(demodulator );
		if (error) goto exit;
	}


	/** Set frequency */
	for (i = begin; i < end; i++) {
		error = Standard_setFrequency (demodulator, i, frequency);
		if (error) goto exit;
	}
exit :

	return (error);
}


uint32_t Standard_setStreamType (
		Demodulator*    demodulator,
	  StreamType      streamType
) {
	uint32_t error = Error_NO_ERROR;

//	uint32_t warning = Error_NO_ERROR;
	IT9130* it9130;
	uint8_t i;
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t var[2];

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];


	/** Enable DVB-T interrupt if next stream type is StreamType_DVBT_DATAGRAM */
	if (streamType == StreamType_DVBT_DATAGRAM) {
		for (i = 0; i < it9130->chipNumber; i++) {
			error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_dvbt_inten, reg_dvbt_inten_pos, reg_dvbt_inten_len, 1);
			if (error) goto exit;
			if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_mpeg_full_speed, reg_mpeg_full_speed_pos, reg_mpeg_full_speed_len, 0);
				if (error) goto exit;
			} else {
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_mpeg_full_speed, reg_mpeg_full_speed_pos, reg_mpeg_full_speed_len, 1);
				if (error) goto exit;
			}
		}
	}

	/** Enable DVB-T mode */
	for (i = 0; i < it9130->chipNumber; i++) {
		error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_dvbt_en, reg_dvbt_en_pos, reg_dvbt_en_len, 1);
		if (error) goto exit;
	}

	/** Pictor & Orion & Omega & Omega_LNA*/
    if (it9130->tunerDescription->tunerId == 0x35 || it9130->tunerDescription->tunerId == 0x39 || it9130->tunerDescription->tunerId == 0x3A || chip_Type==0x9135 || chip_Type==0x9175) {
		/** Enter sub mode */
		switch (streamType) {
			case StreamType_DVBT_DATAGRAM :
				error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
				if (error) goto exit;
				error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 0);
				if (error) goto exit;
				/** Fix current leakage */
				error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
				if (error) goto exit;
				error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 0);
				if (error) goto exit;
				break;
			case StreamType_DVBT_PARALLEL :
				for (i = 0; i < it9130->chipNumber; i++) {
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
					if (error) goto exit;
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 1);
					if (error) goto exit;

					/** HostB interface is enabled */
					error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
					if (error) goto exit;
					error = Standard_writeRegisterBits ( demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
					if (error) goto exit;
				}
				break;
			case StreamType_DVBT_SERIAL :
				for (i = 0; i < it9130->chipNumber; i++) {
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 1);
					if (error) goto exit;

					/** HostB interface is enabled */
					error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 1);
					if (error) goto exit;
				}
				break;
			default:
				error = Error_INVALID_STREAM_TYPE;
				break;
		}
		error = User_mpegConfig ();

		it9130->streamType = streamType;

		goto exit;
	} else {
		/** Enter sub mode */
		switch (streamType) {
			case StreamType_DVBT_DATAGRAM :
				if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
					error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
					if (error) goto exit;
					error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 0);
					if (error) goto exit;
					/** Fix current leakage */
					if (it9130->chipNumber > 1) {
						if (it9130->hostInterface[0]) {
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
							if (error) goto exit;
						} else {
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 1);
							if (error) goto exit;
						}
					} else {
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 1);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
						if (error) goto exit;
					}
				} else {
					error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
					if (error) goto exit;
					error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 0);
					if (error) goto exit;

					/** Fix current leakage */
					if (it9130->chipNumber > 1) {
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 0);
						if (error) goto exit;
					} else {
						if (it9130->hostInterface[0]) {
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
							if (error) goto exit;
						} else {
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 1);
							if (error) goto exit;
						}
					}
				}
				break;
			case StreamType_DVBT_PARALLEL :
				for (i = 0; i < it9130->chipNumber; i++) {
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
					if (error) goto exit;
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 1);
					if (error) goto exit;

					if (i == 0) {
						if (it9130->hostInterface[0]) {
							/** HostA interface is enabled */
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 1);
							if (error) goto exit;
						} else {
							/** HostB interface is enabled */
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
							if (error) goto exit;
						}
					} else {
						/** HostA interface is enabled */
						error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 0);
						if (error) goto exit;
						error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_par_mode, reg_top_hosta_mpeg_par_mode_pos, reg_top_hosta_mpeg_par_mode_len, 1);
						if (error) goto exit;
					}
				}
				break;
			case StreamType_DVBT_SERIAL :
				for (i = 0; i < it9130->chipNumber; i++) {
					error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 1);
					if (error) goto exit;

					if (i == 0) {
						if (it9130->hostInterface[0]) {
							/** HostA interface is enabled */
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 1);
							if (error) goto exit;
						} else {
							/** HostB interface is enabled */
							error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 1);
							if (error) goto exit;
						}
					} else {
						/** HostA interface is enabled */
						error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hosta_mpeg_ser_mode, reg_top_hosta_mpeg_ser_mode_pos, reg_top_hosta_mpeg_ser_mode_len, 1);
						if (error) goto exit;
					}
				}
				break;

			default:
				error = Error_INVALID_STREAM_TYPE;
				break;
		}
	}
	error = User_mpegConfig ();

	it9130->streamType = streamType;

exit :

	return (error);
}


uint32_t Standard_setArchitecture (
		Demodulator*    demodulator,
	  Architecture    architecture
) {
	uint32_t error = Error_NO_ERROR;

	uint16_t frameSize;
	uint8_t packetSize;
	uint8_t buffer[2];
	uint8_t standAlone[2];
	uint8_t upperChip[2];
	uint8_t upperHost[2];
	uint8_t lowerChip[2];
	uint8_t lowerHost[2];
	uint8_t dcaEnable[2];
	uint8_t phaseLatch[2];
	uint8_t fpgaLatch[2];
	uint8_t i;
	Booll pipValid = False;
	IT9130* it9130;
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t var[2];

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];

	if ((architecture == Architecture_DCA) || (architecture == Architecture_DCAEx)) {
		for (i = 0; i < it9130->chipNumber; i++) {
			standAlone[i] = 0;
			upperChip[i] = 0;
			upperHost[i] = 0;
			lowerChip[i] = 0;
			lowerHost[i] = 0;
			dcaEnable[i] = 1;
			phaseLatch[i] = 0;
			fpgaLatch[i] = 0;
		}
		if (it9130->chipNumber == 1) {
			standAlone[0] = 1;
			dcaEnable[0] = 0;
		} else {
            /** Pictor & Orion & Omega*/

			if (it9130->tunerDescription->tunerId == 0x35 || it9130->tunerDescription->tunerId == 0x39 || it9130->tunerDescription->tunerId == 0x3A) {
				upperChip[it9130->chipNumber - 1] = 1;
				upperHost[0] = 1;
				lowerChip[0] = 1;
				lowerHost[it9130->chipNumber - 1] = 1;
				phaseLatch[0] = 0;
				phaseLatch[it9130->chipNumber - 1] = 0;
				fpgaLatch[0] = 0;
				fpgaLatch[it9130->chipNumber - 1] = 0;
			} else if(chip_Type == 0x9135 || chip_Type==0x9175){
				upperChip[it9130->chipNumber - 1] = 1;
				upperHost[0] = 1;

				lowerChip[0] = 1;
				lowerHost[it9130->chipNumber - 1] = 1;

				phaseLatch[0] = 1;
				phaseLatch[it9130->chipNumber - 1] = 1;

				//fpgaLatch[0] = 0x44;
				//fpgaLatch[it9130->chipNumber - 1] = 0x44;
				fpgaLatch[0] = 0x55;
				fpgaLatch[it9130->chipNumber - 1] = 0x55;
			} else {
				upperChip[it9130->chipNumber - 1] = 1;
				upperHost[0] = 1;
				lowerChip[0] = 1;
				lowerHost[it9130->chipNumber - 1] = 1;
				phaseLatch[0] = 1;
				phaseLatch[it9130->chipNumber - 1] = 1;
				fpgaLatch[0] = 0x77;
				fpgaLatch[it9130->chipNumber - 1] = 0x77;
			}

		}
	} else {
		for (i = 0; i < it9130->chipNumber; i++) {
			standAlone[i] = 1;
			upperChip[i] = 0;
			upperHost[i] = 0;
			lowerChip[i] = 0;
			lowerHost[i] = 0;
			dcaEnable[i] = 0;
			phaseLatch[i] = 0;
			fpgaLatch[i] = 0;
		}
	}

	if (it9130->initialized == True) {
		error = Standard_maskDcaOutput (demodulator );
		if (error) goto exit;
	}


	/** Pictor & Orion & Omega & Omega_LNA*/
    if (it9130->tunerDescription->tunerId == 0x35 || it9130->tunerDescription->tunerId == 0x39 || it9130->tunerDescription->tunerId == 0x3A || chip_Type == 0x9135 || chip_Type==0x9175) {
		for (i = it9130->chipNumber; i > 0; i--) {
			/** Set dca_upper_chip */
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_upper_chip, reg_dca_upper_chip_pos, reg_dca_upper_chip_len, upperChip[i - 1]);
			if (error) goto exit;
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, upperHost[i - 1]);
			if (error) goto exit;

			/** Set dca_lower_chip */
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_lower_chip, reg_dca_lower_chip_pos, reg_dca_lower_chip_len, lowerChip[i - 1]);
			if (error) goto exit;
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, lowerHost[i - 1]);
			if (error) goto exit;

			/** Set phase latch */
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_platch, reg_dca_platch_pos, reg_dca_platch_len, phaseLatch[i - 1]);
			if (error) goto exit;

			/** Set fpga latch */
			error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_fpga_latch, reg_dca_fpga_latch_pos, reg_dca_fpga_latch_len, fpgaLatch[i - 1]);
			if (error) goto exit;
		}
	} else {
	    /** Set upper chip first in order to avoid I/O conflict */
	    for (i = it9130->chipNumber; i > 0; i--) {
		    /** Set dca_upper_chip */
		    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_upper_chip, reg_dca_upper_chip_pos, reg_dca_upper_chip_len, upperChip[i - 1]);
		    if (error) goto exit;
		    if (i == 1) {
			    if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
				    if (it9130->hostInterface[0]) {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_upper, reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, upperHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, 0);
					    if (error) goto exit;
				    } else {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, upperHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_upper, reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, 0);
					    if (error) goto exit;
				    }
			    } else {
				    if (it9130->hostInterface[0]) {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, upperHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_upper, reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, 0);
					    if (error) goto exit;
				    } else {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_upper, reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, upperHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits ( demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, 0);
					    if (error) goto exit;
				    }
			    }
		    } else {
			    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_upper, reg_top_hostb_dca_upper_pos, reg_top_hostb_dca_upper_len, upperHost[i - 1]);
			    if (error) goto exit;
			    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_upper, reg_top_hosta_dca_upper_pos, reg_top_hosta_dca_upper_len, 0);
			    if (error) goto exit;
		    }

		    /** Set dca_lower_chip */
		    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_lower_chip, reg_dca_lower_chip_pos, reg_dca_lower_chip_len, lowerChip[i - 1]);
		    if (error) goto exit;
		    if (i == 1) {
			    if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
				    if (it9130->hostInterface[0]) {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_lower, reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, lowerHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, 0);
					    if (error) goto exit;
				    } else {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, lowerHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_lower, reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, 0);
					    if (error) goto exit;
				    }
			    } else {
				    if (it9130->hostInterface[0]) {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, lowerHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_lower, reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, 0);
					    if (error) goto exit;
				    } else {
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_lower, reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, lowerHost[i - 1]);
					    if (error) goto exit;
					    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, 0);
					    if (error) goto exit;
				    }
			    }
		    } else {
			    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hostb_dca_lower, reg_top_hostb_dca_lower_pos, reg_top_hostb_dca_lower_len, lowerHost[i - 1]);
			    if (error) goto exit;
			    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_top_hosta_dca_lower, reg_top_hosta_dca_lower_pos, reg_top_hosta_dca_lower_len, 0);
			    if (error) goto exit;
		    }

		    /** Set phase latch */
		    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_platch, reg_dca_platch_pos, reg_dca_platch_len, phaseLatch[i - 1]);
		    if (error) goto exit;

		    /** Set fpga latch */
		    error = Standard_writeRegisterBits (demodulator, i - 1, Processor_OFDM, p_reg_dca_fpga_latch, reg_dca_fpga_latch_pos, reg_dca_fpga_latch_len, fpgaLatch[i - 1]);
		    if (error) goto exit;
	    }
	}

	for (i = 0; i < it9130->chipNumber; i++) {
		/** Set stand alone */
		error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_dca_stand_alone, reg_dca_stand_alone_pos, reg_dca_stand_alone_len, standAlone[i]);
		if (error) goto exit;

		/** Set DCA enable */
		error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_dca_en, reg_dca_en_pos, reg_dca_en_len, dcaEnable[i]);
		if (error) goto exit;
	}


	if (it9130->initialized == True) {
		for (i = 0; i < it9130->chipNumber; i++) {
			error = Standard_writeRegister (demodulator, i, Processor_OFDM, trigger_ofsm, 0);
			if (error) goto exit;
		}
	}


	if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
		frameSize = User_USB20_FRAME_SIZE_DW;
		packetSize = (uint8_t) (User_USB20_MAX_PACKET_SIZE / 4);

		if (it9130->busId == Bus_USB11) {
			frameSize   = User_USB11_FRAME_SIZE_DW;
			packetSize = (uint8_t) (User_USB11_MAX_PACKET_SIZE / 4);
		}

		if ((it9130->chipNumber > 1) && (architecture == Architecture_PIP))
			pipValid = True;

		/** Reset EP4 */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2_sw_rst, reg_mp2_sw_rst_pos, reg_mp2_sw_rst_len, 1);
		if (error) goto exit;

		/** Reset EP5 */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if2_sw_rst, reg_mp2if2_sw_rst_pos, reg_mp2if2_sw_rst_len, 1);
		if (error) goto exit;

		/** Disable EP4 */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep4_tx_en, reg_ep4_tx_en_pos, reg_ep4_tx_en_len, 0);
		if (error) goto exit;

		/** Disable EP5 */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep5_tx_en, reg_ep5_tx_en_pos, reg_ep5_tx_en_len, 0);
		if (error) goto exit;

		/** Disable EP4 NAK */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep4_tx_nak, reg_ep4_tx_nak_pos, reg_ep4_tx_nak_len, 0);
		if (error) goto exit;

		/** Disable EP5 NAK */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep5_tx_nak, reg_ep5_tx_nak_pos, reg_ep5_tx_nak_len, 0);
		if (error) goto exit;

		/** Enable EP4 */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep4_tx_en, reg_ep4_tx_en_pos, reg_ep4_tx_en_len, 1);
		if (error) goto exit;

		/** Set EP4 transfer length */
		buffer[p_reg_ep4_tx_len_7_0 - p_reg_ep4_tx_len_7_0] = (uint8_t) frameSize;
		buffer[p_reg_ep4_tx_len_15_8 - p_reg_ep4_tx_len_7_0] = (uint8_t) (frameSize >> 8);
		error = Standard_writeRegisters (demodulator, 0, Processor_LINK, p_reg_ep4_tx_len_7_0, 2, buffer);

		/** Set EP4 packet size */
		error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_ep4_max_pkt, packetSize);
		if (error) goto exit;

		if (pipValid == True) {
			/** Enable EP5 */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_LINK, p_reg_ep5_tx_en, reg_ep5_tx_en_pos, reg_ep5_tx_en_len, 1);
			if (error) goto exit;

			/** Set EP5 transfer length */
			buffer[p_reg_ep5_tx_len_7_0 - p_reg_ep5_tx_len_7_0] = (uint8_t) frameSize;
			buffer[p_reg_ep5_tx_len_15_8 - p_reg_ep5_tx_len_7_0] = (uint8_t) (frameSize >> 8);
			error = Standard_writeRegisters (demodulator, 0, Processor_LINK, p_reg_ep5_tx_len_7_0, 2, buffer);

			/** Set EP5 packet size */
			error = Standard_writeRegister (demodulator, 0, Processor_LINK, p_reg_ep5_max_pkt, packetSize);
			if (error) goto exit;
		}


		/** Disable 15 SER/PAR mode */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
		if (error) goto exit;
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_mp2if_mpeg_par_mode, mp2if_mpeg_par_mode_pos, mp2if_mpeg_par_mode_len, 0);
		if (error) goto exit;

		if (pipValid == True) {
			/** Enable mp2if2 */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if2_en, reg_mp2if2_en_pos, reg_mp2if2_en_len, 1);
			if (error) goto exit;

			for (i = 1; i < it9130->chipNumber; i++) {
				/** Enable serial mode */
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 1);
				if (error) goto exit;

				/** Enable HostB serial */
				error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 1);
				if (error) goto exit;
			}

			/** Enable tsis */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_tsis_en, reg_tsis_en_pos, reg_tsis_en_len, 1);
			if (error) goto exit;
		} else {
			/** Disable mp2if2 */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if2_en, reg_mp2if2_en_pos, reg_mp2if2_en_len, 0);
			if (error) goto exit;

			for (i = 1; i < it9130->chipNumber; i++) {
				/** Disable serial mode */
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_mp2if_mpeg_ser_mode, mp2if_mpeg_ser_mode_pos, mp2if_mpeg_ser_mode_len, 0);
				if (error) goto exit;

				/** Disable HostB serial */
				error = Standard_writeRegisterBits (demodulator, i, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
				if (error) goto exit;
			}

			/** Disable tsis */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_tsis_en, reg_tsis_en_pos, reg_tsis_en_len, 0);
			if (error) goto exit;
		}

		/** Negate EP4 reset */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2_sw_rst, reg_mp2_sw_rst_pos, reg_mp2_sw_rst_len, 0);
		if (error) goto exit;

		/** Negate EP5 reset */
		error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if2_sw_rst, reg_mp2if2_sw_rst_pos, reg_mp2if2_sw_rst_len, 0);
		if (error) goto exit;

		if (pipValid == True) {
			/** Split 15 PSB to 1K + 1K and enable flow control */
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if2_half_psb, reg_mp2if2_half_psb_pos, reg_mp2if2_half_psb_len, 0);
			if (error) goto exit;
			error = Standard_writeRegisterBits (demodulator, 0, Processor_OFDM, p_reg_mp2if_stop_en, reg_mp2if_stop_en_pos, reg_mp2if_stop_en_len, 1);
			if (error) goto exit;

			for (i = 1; i < it9130->chipNumber; i++) {
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_mpeg_full_speed, reg_mpeg_full_speed_pos, reg_mpeg_full_speed_len, 1);//
				if (error) goto exit;
				error = Standard_writeRegisterBits (demodulator, i, Processor_OFDM, p_reg_mp2if_stop_en, reg_mp2if_stop_en_pos, reg_mp2if_stop_en_len, 0);
				if (error) goto exit;
			}
		}
	}

	it9130->architecture = architecture;

exit:


	return (error);
}

uint32_t Standard_setViterbiRange (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            superFrameCount,
	  uint16_t            packetUnit
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp0;
	uint8_t temp1;

	/** Set super frame count */
	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, qnt_vbc_sframe_num, superFrameCount);
	if (error) goto exit;

	/** Set packet unit. */
	temp0 = (uint8_t) packetUnit;
	temp1 = (uint8_t) (packetUnit >> 8);
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, rsd_packet_unit_7_0, 1, &temp0);
	if (error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, rsd_packet_unit_15_8, 1, &temp1);
	if (error) goto exit;

exit:

	return (error);
}


uint32_t Standard_getViterbiRange (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t*           frameCount,
	  uint16_t*           packetUnit
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp0;
	uint8_t temp1;

	/** Get super frame count */
	error = Standard_readRegister (demodulator, chip, Processor_OFDM, qnt_vbc_sframe_num, frameCount);
	if (error) goto exit;

	/** Get packet unit. */
	error = Standard_readRegisters (demodulator, chip, Processor_OFDM, r_rsd_packet_unit_7_0, 1, &temp0);
	if (error) goto exit;
	error = Standard_readRegisters (demodulator, chip, Processor_OFDM, r_rsd_packet_unit_15_8, 1, &temp1);
	if (error) goto exit;
	*packetUnit = (uint16_t) (temp1 << 8) + (uint16_t) temp0;

exit:

	return (error);
}

uint32_t Standard_getStatistic (
		Demodulator*    demodulator,
	  uint8_t            chip,
	 Statistic*      statistic
) {
	uint32_t error = Error_NO_ERROR;

	IT9130* it9130;
	uint8_t quality;
	uint8_t strength;
	uint8_t buffer[2];

	it9130 = (IT9130*) demodulator;

	/** Get statistic by stream type */
	error = Standard_readRegisters (demodulator, chip, Processor_OFDM, tpsd_lock, mpeg_lock - tpsd_lock + 1, buffer);
	if (error) goto exit;

	if (buffer[tpsd_lock - tpsd_lock])
		it9130->statistic[chip].signalPresented = True;
	else
		it9130->statistic[chip].signalPresented = False;

	if (buffer[mpeg_lock - tpsd_lock])
		it9130->statistic[chip].signalLocked = True;
	else
		it9130->statistic[chip].signalLocked = False;

	error = Standard_getSignalQualityIndication (demodulator, chip, &quality);
	if (error) goto exit;

	it9130->statistic[chip].signalQuality = quality;

	error = Standard_getSignalStrengthIndication (demodulator, chip, &strength);

	if (error) goto exit;

	it9130->statistic[chip].signalStrength = strength;

	*statistic = it9130->statistic[chip];

exit :

	return (error);
}

uint32_t Standard_reboot (
		Demodulator*    demodulator
)  {
	uint32_t error = Error_NO_ERROR;
	uint32_t version;
	uint8_t i;
	IT9130* it9130;
//	uint8_t chip_version = 0;
	//uint32_t chip_Type;
	//uint8_t var[2];

	it9130 = (IT9130*) demodulator;

	error = Standard_getFirmwareVersion (demodulator, Processor_LINK, &version);
	if (error) goto exit;
	if (version == 0xFFFFFFFF) goto exit;       /** I2M and I2U */
	if (version != 0) {
		//for (i = it9130->chipNumber; i > 0; i--) {
			error = Cmd_reboot ( 0);
			User_delay ( 1);
			if (error) goto exit;
		//}

		if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) goto exit;

		User_delay ( 10);

		version = 1;
		for (i = 0; i < 30; i++) {
			error = Standard_getFirmwareVersion (demodulator, Processor_LINK, &version);
			if (error == Error_NO_ERROR) break;
			User_delay ( 10);
		}
		if (error)
			goto exit;

		if (version != 0)
			error = Error_REBOOT_FAIL;
	}

	for (i = it9130->chipNumber; i > 0; i--) {
		error = Standard_writeRegisterBits (demodulator, i - 1, Processor_LINK, p_reg_p_dmb_sw_reset, reg_p_dmb_sw_reset_pos, reg_p_dmb_sw_reset_len, 1);
		if (error) goto exit;
	}


	it9130->booted = False;

exit :
	return (error);
}


uint32_t Standard_controlPowerSaving (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            control
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t temp;
	uint8_t j;
	IT9130* it9130;
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t var[2];

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];

	if (control) {
		/** Power up case */
		if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {
			error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_reg_afe_mem0, 3, 1, 0);
			if (error) goto exit;
			error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_dyn0_clk, 0);
			if (error) goto exit;
		} else {
			/** TS, SPI, and SDIO case */
		}


		/** Fixed current leakage */
		switch (it9130->busId) {
			case Bus_SPI :
			case Bus_SDIO :
			case Bus_USB :
			case Bus_USB11 :
				if ((it9130->chipNumber > 1)) {
					if(chip > 0) {
						/** Pictor & Orion & Omega & Omega_LNA*/
                        if (it9130->tunerDescription->tunerId == 0x35 || it9130->tunerDescription->tunerId == 0x39) {
							/** Disable HostB parallel */
							error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 0);
							if (error) goto exit;
						} else if(chip_Type == 0x9135 || chip_Type==0x9175){
							/** Enable HostB serial */
							if( (it9130->architecture == Architecture_PIP))
								error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 1);
							else
								error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							/** Disable HostB parallel */
							error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 0);
							if (error) goto exit;
						}
					}
				}
				break;
		}
	} else {
		/** Power down case */
		if ((it9130->busId == Bus_USB) || (it9130->busId == Bus_USB11)) {

			error = Standard_writeRegister (demodulator, chip, Processor_OFDM, suspend_flag, 1);
			if (error) goto exit;
			error = Standard_writeRegister (demodulator, chip, Processor_OFDM, trigger_ofsm, 0);
			if (error) goto exit;

			for (j = 0; j < 150; j++) {
				error = Standard_readRegister (demodulator, chip, Processor_OFDM, suspend_flag, &temp);
				if (error) goto exit;
				if (!temp) break;
				User_delay ( 10);
			}
			error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_reg_afe_mem0, 3, 1, 1);//omega not use ;power down demod adc
			if (error) goto exit;

		} else {
			/** TS SPI SDIO */
		}

		/** Fixed current leakage */
		switch (it9130->busId) {
			case Bus_SPI :
			case Bus_SDIO :
			case Bus_USB :
			case Bus_USB11 :
				if ((it9130->chipNumber > 1)) {
					if(chip > 0) {
						/** Pictor & Orion & Omega */
                        if (it9130->tunerDescription->tunerId == 0x35 || it9130->tunerDescription->tunerId == 0x39 || chip_Type == 0x9135 || chip_Type==0x9175) {
							/** Enable HostB parallel */
							error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_ser_mode, reg_top_hostb_mpeg_ser_mode_pos, reg_top_hostb_mpeg_ser_mode_len, 0);
							if (error) goto exit;
							error = Standard_writeRegisterBits (demodulator, chip, Processor_LINK, p_reg_top_hostb_mpeg_par_mode, reg_top_hostb_mpeg_par_mode_pos, reg_top_hostb_mpeg_par_mode_len, 1);
							if (error) goto exit;
						}
					}
				}
				break;
		}
	}

exit :
	return (error);
}

uint32_t Standard_controlTunerLeakage (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            control
) {
	uint32_t error = Error_NO_ERROR;


	IT9130* it9130;
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t var[2];
	uint8_t value[15] = {0};

	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];

	if (control) {
		/** Power up case */
	    error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_p_if_en,  1);
		if (error) goto exit;

	} else {

		/** Fixed tuner current leakage */
		error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_dyn0_clk, 0);
		if (error) goto exit;
		//0xec40

        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_p_if_en,  0);
	    if (error) goto exit;

		value[1] = 0x0c;


        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_pd_a, 15, value);
	    if (error) goto exit;

		value[1] = 0;

        //0xec12~0xec15
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_lna_g, 4, value);
	    if (error) goto exit;

        //oxec17~0xec1f
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_pgc, 9, value);
	    if (error) goto exit;

        //0xec22~0xec2b
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_clk_del_sel, 10, value);
	    if (error) goto exit;

        //0xec20
        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, 0xec20,  0);
	    if (error) goto exit;

        //0xec3f
        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, 0xec3F,  1);
	    if (error) goto exit;
	}

exit :

	return (error);
}

uint32_t Standard_controlTunerPowerSaving (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            control
) {

	uint32_t error = Error_NO_ERROR;

	IT9130* it9130;
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t var[2];
	uint8_t value[15] = {0};
	it9130 = (IT9130*) demodulator;

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];

	if (control) {
		/** Power up case */
	    error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_p_if_en,  1);
		if (error) goto exit;

	} else {

		/** tuner power down */
		error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_dyn0_clk, 0);
		if (error) goto exit;
		//0xec40

        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_reg_p_if_en,  0);
	    if (error) goto exit;

		value[0] = 0x3F; //0xec02
		value[1] = 0x1F; //0xec03
		value[2] = 0x3F; //0xec04
		value[3] = 0x3E; //0xec05

        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_pd_a, 15, value);
	    if (error) goto exit;

		value[0] = 0;
		value[1] = 0;
		value[2] = 0;
		value[3] = 0;

        //0xec12~0xec15
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_lna_g, 4, value);
	    if (error) goto exit;

        //oxec17~0xec1f
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_pgc, 9, value);
	    if (error) goto exit;

        //0xec22~0xec2b
        error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_reg_clk_del_sel, 10, value);
	    if (error) goto exit;

        //0xec20
        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, 0xec20,  0);
	    if (error) goto exit;

        //0xec3f
        error = Standard_writeRegister (demodulator, chip, Processor_OFDM, 0xec3F,  1);
	    if (error) goto exit;

	}
	exit:
	return (error);
}

uint32_t Standard_controlPidFilter (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            control
) {
	uint32_t error = Error_NO_ERROR;

	error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_mp2if_pid_en, mp2if_pid_en_pos, mp2if_pid_en_len, control);

	return (error);
}


uint32_t Standard_resetPidFilter (
		Demodulator*    demodulator,
	  uint8_t            chip
) {
	uint32_t error = Error_NO_ERROR;

	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_mp2if_pid_rst, mp2if_pid_rst_pos, mp2if_pid_rst_len, 1);
	if (error) goto exit;

exit :

	return (error);
}


uint32_t Standard_addPidToFilter (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  uint8_t            index,
	  Pid             pid
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t writeBuffer[2];
	IT9130* it9130;

	it9130 = (IT9130*) demodulator;

	/** Enable pid filter */
	error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_mp2if_pid_en, mp2if_pid_en_pos, mp2if_pid_en_len, 1);
	if (error) goto exit;

	writeBuffer[0] = (uint8_t) pid.value;
	writeBuffer[1] = (uint8_t) (pid.value >> 8);

	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, p_mp2if_pid_dat_l, 2, writeBuffer);
	if (error) goto exit;

	error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, p_mp2if_pid_index_en, mp2if_pid_index_en_pos, mp2if_pid_index_en_len, 1);
	if (error) goto exit;

	error = Standard_writeRegister (demodulator, chip, Processor_OFDM, p_mp2if_pid_index, index);
	if (error) goto exit;

exit :

	return (error);
}


uint32_t Standard_getSNR (
		Demodulator*    demodulator,
	  uint8_t            chip,
     uint8_t*           snr
) {

    uint32_t error = Error_NO_ERROR;
    ChannelModulation    channelModulation;
    uint32_t   snr_value;

    error= Standard_getChannelModulation(demodulator, chip, &channelModulation);
    if (error) goto exit;

    error = Standard_getSNRValue(demodulator, chip, &snr_value);
    if (error) goto exit;

    if (channelModulation.transmissionMode == TransmissionMode_2K)
        snr_value = snr_value*4;
    else if (channelModulation.transmissionMode == TransmissionMode_4K)
        snr_value = snr_value*2;
    else
        snr_value = snr_value*1;

    if(channelModulation.constellation == 0) //Constellation_QPSK
    {

        if(snr_value < 0xB4771)         *snr = 0;
        else if(snr_value < 0xC1AED)    *snr = 1;
        else if(snr_value < 0xD0D27)    *snr = 2;
        else if(snr_value < 0xE4D19)    *snr = 3;
        else if(snr_value < 0xE5DA8)    *snr = 4;
        else if(snr_value < 0x107097)   *snr = 5;
        else if(snr_value < 0x116975)   *snr = 6;
        else if(snr_value < 0x1252D9)   *snr = 7;
        else if(snr_value < 0x131FA4)   *snr = 8;
        else if(snr_value < 0x13D5E1)   *snr = 9;
        else if(snr_value < 0x148E53)   *snr = 10;
        else if(snr_value < 0x15358B)   *snr = 11;
        else if(snr_value < 0x15DD29)   *snr = 12;
        else if(snr_value < 0x168112)   *snr = 13;
        else if(snr_value < 0x170B61)   *snr = 14;
        else if(snr_value < 0x17A532)   *snr = 15;
        else if(snr_value < 0x180F94)   *snr = 16;
        else if(snr_value < 0x186ED2)   *snr = 17;
        else if(snr_value < 0x18B271)   *snr = 18;
        else if(snr_value < 0x18E118)   *snr = 19;
        else if(snr_value < 0x18FF4B)   *snr = 20;
        else if(snr_value < 0x190AF1)   *snr = 21;
        else if(snr_value < 0x191451)   *snr = 22;
        else   *snr = 23;
    }
    else if ( channelModulation.constellation == 1) //Constellation_16QAM
    {
        if(snr_value < 0x4F0D5)    *snr = 0;
        else if(snr_value < 0x5387A)   *snr = 1;
        else if(snr_value < 0x573A4)   *snr = 2;
        else if(snr_value < 0x5A99E)   *snr = 3;
        else if(snr_value < 0x5CC80)   *snr = 4;
        else if(snr_value < 0x5EB62)   *snr = 5;
        else if(snr_value < 0x5FECF)   *snr = 6;
        else if(snr_value < 0x60B80)   *snr = 7;
        else if(snr_value < 0x62501)   *snr = 8;
        else if(snr_value < 0x64865)   *snr = 9;
        else if(snr_value < 0x69604)   *snr = 10;
        else if(snr_value < 0x6F356)   *snr = 11;
        else if(snr_value < 0x7706A)   *snr = 12;
        else if(snr_value < 0x804D3)   *snr = 13;
        else if(snr_value < 0x89D1A)   *snr = 14;
        else if(snr_value < 0x93E3D)   *snr = 15;
        else if(snr_value < 0x9E35D)   *snr = 16;
        else if(snr_value < 0xA7C3C)   *snr = 17;
        else if(snr_value < 0xAFAF8)   *snr = 18;
        else if(snr_value < 0xB719D)   *snr = 19;
        else if(snr_value < 0xBDA6A)   *snr = 20;
        else if(snr_value < 0xC0C75)   *snr = 21;
        else if(snr_value < 0xC3F7D)   *snr = 22;
        else if(snr_value < 0xC5E62)   *snr = 23;
        else if(snr_value < 0xC6C31)   *snr = 24;
        else if(snr_value < 0xC7925)   *snr = 25;
        else    *snr = 26;
    }
    else if ( channelModulation.constellation == 2) //Constellation_64QAM
    {
        if(snr_value < 0x256D0)    *snr = 0;
        else if(snr_value < 0x27A65)   *snr = 1;
        else if(snr_value < 0x29873)   *snr = 2;
        else if(snr_value < 0x2B7FE)   *snr = 3;
        else if(snr_value < 0x2CF1E)   *snr = 4;
        else if(snr_value < 0x2E234)   *snr = 5;
        else if(snr_value < 0x2F409)   *snr = 6;
        else if(snr_value < 0x30046)   *snr = 7;
        else if(snr_value < 0x30844)   *snr = 8;
        else if(snr_value < 0x30A02)   *snr = 9;
        else if(snr_value < 0x30CDE)   *snr = 10;
        else if(snr_value < 0x31031)   *snr = 11;
        else if(snr_value < 0x3144C)   *snr = 12;
        else if(snr_value < 0x315DD)   *snr = 13;
        else if(snr_value < 0x31920)   *snr = 14;
        else if(snr_value < 0x322D0)   *snr = 15;
        else if(snr_value < 0x339FC)   *snr = 16;
        else if(snr_value < 0x364A1)   *snr = 17;
        else if(snr_value < 0x38BCC)   *snr = 18;
        else if(snr_value < 0x3C7D3)   *snr = 19;
        else if(snr_value < 0x408CC)   *snr = 20;
        else if(snr_value < 0x43BED)   *snr = 21;
        else if(snr_value < 0x48061)   *snr = 22;
        else if(snr_value < 0x4BE95)   *snr = 23;
        else if(snr_value < 0x4FA7D)   *snr = 24;
        else if(snr_value < 0x52405)   *snr = 25;
        else if(snr_value < 0x5570D)   *snr = 26;
        else if(snr_value < 0x59FEB)   *snr = 27;
        else if(snr_value < 0x5BF38)   *snr = 28;
		else if(snr_value < 0x5F78F)   *snr = 29;
        else if(snr_value < 0x612C3)   *snr = 30;
        else if(snr_value < 0x626BE)   *snr = 31;
        else    *snr = 32;
    } else
        goto exit;

exit:
    return error;

}

uint32_t Standard_getSNRValue(
		Demodulator*    demodulator,
	    uint8_t            chip,
       uint32_t*    snr_value
) {

    uint32_t error = Error_NO_ERROR;
   	uint8_t superFrame_num=0;
	uint8_t snr_reg[3];

	error = Standard_readRegisters(demodulator, chip, Processor_OFDM, 0x2c,3,snr_reg);
    if(error) goto exit;

    *snr_value = (snr_reg[2]<<16)+(snr_reg[1]<<8)+snr_reg[0];

	error = Standard_readRegister(demodulator, chip, Processor_OFDM, 0xF78b, (unsigned char *)&superFrame_num); //gets superFrame num
    if(error) goto exit;
	if(superFrame_num)
          *snr_value/=superFrame_num;
exit:
    return(error);

}

uint32_t Standard_setMultiplier (
		Demodulator*    demodulator,
      Multiplier      multiplier
) {
    uint32_t error = Error_NO_ERROR;
//    uint8_t oldAdcMultiplier = 0;
    uint8_t newAdcMultiplier = 0;
    uint8_t buffer[3];
    Long controlWord;
    uint8_t i;
    IT9130* it9130;

    it9130 = (IT9130*) demodulator;

    if (multiplier == Multiplier_1X)
        newAdcMultiplier = 0;
    else
        newAdcMultiplier = 1;


    for (i = 0; i < it9130->chipNumber; i++) {
        /** Write ADC multiplier factor to firmware. */
        error = Standard_writeRegister (demodulator, i, Processor_OFDM, adcx2, newAdcMultiplier);
        if (error) goto exit;
    }

    /** Compute FCW and load them to device */

	if (it9130->fcw >= 0x00400000) {
        controlWord = it9130->fcw - 0x00800000;
    } else {
        controlWord = it9130->fcw;
    }
    if (newAdcMultiplier == 1) {
        controlWord /= 2;
    } else {
        controlWord *= 2;
    }
    it9130->fcw = 0x7FFFFF & controlWord;

    buffer[0] = (uint8_t) (it9130->fcw & 0x000000FF);
    buffer[1] = (uint8_t) ((it9130->fcw & 0x0000FF00) >> 8);
    buffer[2] = (uint8_t) ((it9130->fcw & 0x007F0000) >> 16);
    for (i = 0; i < it9130->chipNumber; i++) {
		error = Standard_writeRegisters (demodulator, i, Processor_OFDM, bfs_fcw_7_0, bfs_fcw_22_16 - bfs_fcw_7_0 + 1, buffer);
    }

exit :
    return (error);
}

uint32_t Standard_setStreamPriority (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Priority		priority
){
    uint32_t error = Error_NO_ERROR;

    if(priority)
        error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, g_reg_dec_pri, reg_dec_pri_pos, reg_dec_pri_len, 0);
    else
        error = Standard_writeRegisterBits (demodulator, chip, Processor_OFDM, g_reg_dec_pri, reg_dec_pri_pos, reg_dec_pri_len, 1);
    return (error);
}

uint32_t Standard_initDetectTable (
		Demodulator*    demodulator,
	  uint8_t            chip
) {
	uint32_t error = Error_NO_ERROR;
	uint8_t lo_freq[18] =
	{
		0x51, 0x16,	//121.5M
		0x6F, 0x54,	//163.5M
		0x73, 0x44,	//205.5M
		0x91, 0x67,	//247.5M
		0x95, 0x5B,	//285.5M
		0xB0, 0x98,	//354M
		0xCF, 0xD2,	//450M
		0xD3, 0x32,	//546M
		0xEF, 0x0C	//642M
	};
	uint8_t lna_cap[9] =
	{
		0, 0, 0, 0, 0, 0, 1, 3, 4
	};

	error = Standard_writeRegisters(demodulator, chip, Processor_OFDM, 0x4597, 18, lo_freq);	//scan_channel_lo_freq[9]
	if (error) goto exit;
	error = Standard_writeRegisters(demodulator, chip, Processor_OFDM, 0x45A9, 9, lna_cap);	//scan_channel_lna_cap[9]
 	if (error) goto exit;

exit :
	return (error);
}

uint32_t Standard_initDecryptTable (
	 Demodulator*    demodulator,
	  uint8_t            chip
) {
	uint32_t error = Error_NO_ERROR;
	uint8_t IP_Table[8] = {1, 5, 2, 0, 3, 7, 4, 6};
	uint8_t IPR_Table[8] = {3, 0, 2, 4, 6, 1, 7, 5};
	uint8_t SW_Table[8] = {4, 5, 6, 7, 0, 1, 2, 3};
	uint8_t EP_Table[8] = {7, 4, 5, 6, 5, 6, 7, 4};
	uint8_t P4_Table[8] = {0, 0, 0, 0, 5, 7, 6, 4};
	uint8_t SBOX1_Table[16] = {1, 3, 0, 2, 3, 1, 2, 0, 0, 3, 2, 1, 1, 3, 3, 2};
	uint8_t SBOX2_Table[16] = {0, 2, 1, 0, 2, 1, 3, 3, 3, 2, 0, 1, 1, 0, 0, 3};

	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45B8, 16, SBOX1_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45C8, 16, SBOX2_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45D8, 8, IP_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45E0, 8, IPR_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45E8, 8, SW_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45F0, 8, EP_Table);
	if(error) goto exit;
	error = Standard_writeRegisters (demodulator, chip, Processor_OFDM, 0x45F8, 8, P4_Table);
	if(error) goto exit;


exit :
	return (error);
}

uint32_t Standard_setDcaLatchTime(
		Demodulator*    demodulator,
	  uint8_t            masterValue,
	  uint8_t            slaveValue
){
	uint32_t error = Error_NO_ERROR;
	IT9130* it9130;
	it9130 = (IT9130*)demodulator;


	if ((it9130->chipNumber>1) && ((it9130->architecture == Architecture_DCA) || (it9130->architecture == Architecture_DCAEx))) {
		error = Standard_writeRegister(demodulator, 0, Processor_OFDM, p_reg_dca_fpga_latch, masterValue);
		if (error) goto exit;
		error = Standard_writeRegister(demodulator, 1, Processor_OFDM, p_reg_dca_fpga_latch, slaveValue);
	}
exit:
	return (error);

}

uint32_t Standard_acquireChannelEx(
		Demodulator*    demodulator,
		uint16_t			bandwidth,
	  uint32_t           masterFreq,
	  uint32_t           slaveFreq
){

	uint32_t error = Error_NO_ERROR;

	uint8_t begin;
	uint8_t end;
	uint8_t i;

	IT9130* it9130;

	it9130 = (IT9130*) demodulator;


	begin = 0;
	end = it9130->chipNumber;

	for (i = begin; i < end; i++) {
		error = Standard_selectBandwidth (demodulator, i, bandwidth, it9130->adcFrequency);
		if (error) goto exit;
		it9130->bandwidth[i] = bandwidth;
	}


	if (it9130->architecture != Architecture_DCAEx) {
		error = Standard_maskDcaOutput(demodulator );
		if (error) goto exit;
	}


	/** Set frequency */

	error = Standard_setFrequency(demodulator, 0, masterFreq);
	if (error) goto exit;

	if (it9130->chipNumber > 1){
		error = Standard_setFrequency(demodulator, 1, slaveFreq);
		if (error) goto exit;
	}


exit :

	return (error);
}

