#ifndef _ORION_CAL_H
#define _ORION_CAL_H

#include "it9510.h"
#define ORION_IQIK_M_CAL_MAX	64
#define ORION_IQIK_M_CAL_MID	32

/*----- MCH 2011-02-22: for far_aci_detection -----*/
#define _FAR_ACI_IDX_SKIP 0xF285
#define FAR_ACI_NUM_BAND 8
#define FAR_ACI
#define ORION_FAR_ACI_BUF_ADDR 0x0123



typedef enum
{
    OrionMsg_OK,
    OrionMsg_ERROR = 0x11110001,
    OrionMsg_Param_not_match,
} OrionMsg;

typedef enum
{
	ORION_GAIN          = 0x0001,		/*return ORION_LNA_GAIN, ORION_PGC_GAIN, ORION_PGA1_GAIN, and ORION_PGA2_GAIN*/
	ORION_LNA_GAIN      = 0x0002,
	ORION_PGC_GAIN      = 0x0004,
	ORION_PGA1_GAIN     = 0x0008,
	ORION_PGA2_GAIN     = 0x0010,
	ORION_RF_BACKOFF    = 0x0020,
	ORION_BB_BACKOFF    = 0x0040,
	ORION_DCC           = 0x0080,		/*return ORION_DCC_I and ORION_DCC_Q*/
	ORION_DCC_I         = 0x0100,
	ORION_DCC_Q         = 0x0200,
	ORION_RF_GMAX       = 0x0400,
	ORION_TOTAL_GMAX    = 0x0800,
	ORION_P_INBAND_SHIFT= 0x1000,
	ORION_P_ALL         = 0x2000,	/*return ORION_P_RSSI, ORION_P_TOTAL, and ORION_P_INBAND*/
    ORION_PARAM_ALL     = 0xFFFF,
} OrionParam_enum;

typedef struct
{
    struct {
        int Gain1;
        int Gain2;
        int Gain3;
        int Gain4;
        int RF_Backoff;
        int BB_Backoff;
    } Gain;

    struct {
        int DCC1;
        int DCC2;
    } DCC;
    
    int RF_Gmax;
    int Total_Gmax;
    int P_Inband_Shift;

    struct {
        int P1;
        int P2;
        int P3;
        int P4;
        int RF_Backoff;
        int BB_Backoff;
        int RF_Gain;
        int Total_Gain;
    } P_All;

} OrionParam;

extern Demodulator* Afatech_ORION_demodulator;
extern uint8_t Afatech_ORION_chip;

unsigned long orion_cal_init(IT9510INFO*    modulator, uint32_t crystalFreq);
unsigned long orion_cal_setfreq(IT9510INFO* modulator, unsigned int bw_kHz, unsigned int rf_freq_kHz);
unsigned long orion_cal_rmr(IT9510INFO* modulator,unsigned short mem_addr, unsigned char len, unsigned char *buf);
unsigned long orion_cal_wmr(IT9510INFO* modulator,unsigned short mem_addr, unsigned char len, unsigned char *val);
unsigned int orion_cal_get_cal_freq_iqik(unsigned int rf_freq_kHz, unsigned char iqik_m_cal);
unsigned int orion_cal_get_freq_code(unsigned int rf_freq_kHz, unsigned int *nc, unsigned int *nv, unsigned int *mv);
unsigned int orion_cal_get_nc(unsigned int rf_freq_kHz);
unsigned int orion_cal_get_nv(unsigned int nc);
unsigned char orion_cal_get_lna_cap_sel(unsigned int rf_freq_kHz);
unsigned int orion_cal_get_lo_freq(unsigned int rf_freq_kHz);
unsigned char orion_cal_get_lpf_bw(unsigned int bw_kHz);
unsigned char orion_cal_get_far_aci_idx_skip(unsigned int rf_freq_kHz);
void orion_cal_get_far_aci_freq_lna(unsigned int rf_freq_kHz);

unsigned long orion_cal_get_param(IT9510INFO* modulator, OrionParam_enum items,  OrionParam *data);
unsigned long orion_cal_set_param(IT9510INFO* modulator, OrionParam_enum items,  OrionParam *data);
unsigned long orion_cal_set_registers(IT9510INFO* modulator);
unsigned long orion_cal_run_dccc_w_iqik(IT9510INFO* modulator, Bool log_on);
unsigned long orion_cal_set_agc(IT9510INFO* modulator, unsigned char lna, unsigned char pgc, unsigned char pga1, unsigned char pga2);
unsigned long orion_cal_run_iqik(IT9510INFO* modulator);
unsigned long orion_cal_read_iqik(IT9510INFO* modulator, int *vi_real, int *vi_imag, int *vq_real, int *vq_imag);
unsigned long orion_dc_scan (IT9510INFO* modulator, Bool ofs);
unsigned long orion_dc_scan_core (IT9510INFO* modulator, int dc_i_ctr, int dc_i_range, int dc_q_ctr, int dc_q_range, Bool ofs, unsigned long limit, int* dc_i_min, int* dc_q_min);
unsigned long orion_dc_scan_core2(IT9510INFO* modulator, int dc_i_ctr, int dc_i_range, int dc_q_ctr, int dc_q_range, Bool ofs, unsigned long limit, int* dc_i_min, int* dc_q_min);

unsigned long orion_cal_read_tune_rssi(IT9510INFO* modulator, unsigned int *rssi_avg, unsigned int avg_times);
unsigned long orion_agc_dcc (IT9510INFO* modulator, int tx_pw);

uint32_t orion_calDC (IT9510INFO* modulator, uint32_t frequency,Bool ofs);

#define FW_SUPPORT
// define function prototype
typedef unsigned long (*ORION_SCAN_CORE_FUNC) (IT9510INFO* modulator, int dc_i_ctr, int dc_i_range, int dc_q_ctr, int dc_q_range, Bool ofs, unsigned long limit, int* dc_i_min, int* dc_q_min);

unsigned long orion_cal_run_iqik2(IT9510INFO* modulator);
unsigned long orion_cal_read_iqik2(IT9510INFO* modulator, int *vi_real, int *vi_imag, int *vq_real, int *vq_imag);
unsigned long setDCCalibrationValue2(IT9510INFO* modulator, int dc_i, int dc_q);
unsigned long setOFSCalibrationValue2(IT9510INFO* modulator, uint8_t ofs_i, uint8_t ofs_q);

// #define FW_SUPPORT_DBG
typedef unsigned long(*ORION_CAL_RUN_IQIK_FUNC) (IT9510INFO* modulator);
typedef unsigned long(*ORION_CAL_READ_IQIK_FUNC) (IT9510INFO* modulator, int *vi_real, int *vi_imag, int *vq_real, int *vq_imag);
typedef unsigned long(*ORION_SET_DC_FUNC) (IT9510INFO* modulator, int dc_i, int dc_q);
typedef unsigned long(*ORION_SET_OFS_FUNC) (IT9510INFO* modulator, uint8_t ofs_i, uint8_t ofs_q);

#endif
