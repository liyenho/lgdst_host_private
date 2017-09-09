#include "orion_cal.h"


//#include "time.h"
//#include "math.h"

static unsigned int c_fband_min[13] = {
	/*the reason to use 392000(but not 400000) because cal_rssi will use 400000-8000= 392000;*/
	392000, 440000, 484000, 533000, 587000, 645000, 710000, 782000, 860000, 
	1450000, 1492000, 1660000, 1685000
};

static unsigned int c_faraciband_min[7] = {
	474000, 538000, 602000, 666000, 730000, 794000, 858000
};

//static unsigned int c_fN_min[9] = {
//	49000, 74000, 111000, 148000, 222000, 296000, 445000, 573000, 890000
//};

/*----- update by Hidy 06/30-----*/
static unsigned int c_fN_min[9] = {
	53000, 74000, 111000, 148000, 222000, 296000, 445000, 573000, 950000
};


static unsigned int c_fUHF_min = 392000;

static unsigned int c_lo_bias[4] = {6, 5, 4, 3};
static unsigned int c_lo_m1200[3] = {3885, 4395, 4905};
static unsigned int c_lo_m2048[3] = {4047, 4578, 5109};

static unsigned char g_clock_mode;
static unsigned int g_fxtal_kHz;
static unsigned int g_fdiv;

/*----- MCH 2011-02-22: for far_aci_detection -----*/
static unsigned int g_far_aci_buf_addr;
static unsigned int g_far_aci_freq_list[8];
static unsigned char g_far_aci_freq_code[16];
static unsigned char g_far_aci_lna_cap_sel[8];
static unsigned char g_far_aci_idx_skip;
static unsigned int g_far_aci_freq_min = 442000;
static unsigned int g_far_aci_freq_max = 954000;
static unsigned int g_far_aci_freq_inc = 64000;


static int g_rf_gmax = 424;			/*42.4;*/
static int g_total_gmax = 774;		/*77.4;*/
static int g_p_inband_shift = -479;	/*-47.9;*/

static int g_orion_version = 2;		/*default: 2 for v2 or earlier*/

//extern uint32_t g_CrystalFreq;

unsigned long orion_cal_init(IT9510INFO*    modulator, uint32_t crystalFreq)
{
	unsigned long error = 0;
	unsigned char buf[4], val;
	unsigned char pd_ori[3] = {0xa6, 0x44, 0xfd}; //orion pd (0x48 ~ 0x4a) default value
	unsigned int m_bdry, nc, nv, fbc_vld_cnt;
	unsigned long tmp_numer, tmp_denom;

	// turn on clock form eagle, Michael 20140423
	val = 0x80;
	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xfb26, 1, &val);
	if (error) goto exit;

	g_orion_version = 3;	/*3 for v3 or later*/

	/*----- set_clock_mode -----*/
	if (crystalFreq == 20480)
		g_clock_mode = 1;
	else if (crystalFreq == 12000)
		g_clock_mode = 0;
	else if (crystalFreq == 20000)
		g_clock_mode = 2;

	/*----- set_registers -----*/
    error = orion_cal_set_registers(modulator);

	// reset fbc
	val = 0x01;
	error = orion_cal_wmr(modulator,0x0101, 1, &val);
	if (error) goto exit;
	val = 0x00;
	error = orion_cal_wmr(modulator,0x0101, 1, &val);
	if (error) goto exit;

	// run fbc
	val = 0x01;
	error = orion_cal_wmr(modulator,0x0100, 1, &val);
	if (error) goto exit;
	error = orion_cal_rmr(modulator,0x0100, 1, &val);
	if (error) goto exit;
	fbc_vld_cnt = 0;
	while (val == 1) 
	{
		error = orion_cal_rmr(modulator,0x0100, 1, &val);
		if (error) goto exit;
		fbc_vld_cnt++;
		if (fbc_vld_cnt == 100) {break;}	
	}

	error = orion_cal_rmr(modulator,0x0103, 1, &buf[1]);	/*p_reg_p_fbc_n_code*/
	if (error) goto exit;
	error = orion_cal_rmr(modulator,0x0123, 2, &buf[2]);	/*r_reg_r_fbc_m_bdry_7_0*/
	if (error) goto exit;

	/*----- set_fref_kHz & m_cal -----*/
	switch(g_clock_mode) {
		case 0:	g_fxtal_kHz = 2000; g_fdiv = 3; val = 16;	break;	/*12.00MHz, 12000/18 = 2000/3*/
		case 1:	g_fxtal_kHz = 640; g_fdiv = 1;	val = 6;	break;	/*20.48MHz, 20480/32 = 640/1*/
		case 2: g_fxtal_kHz = 2000; g_fdiv = 3; val = 16;	break;	/*20.00MHz, 20000/30 = 2000/3*/
		default:g_fxtal_kHz = 640; g_fdiv = 1;	val = 6;	break;	/*20.48MHz, 20480/32 = 640/1*/
	}

	/*----- set_fbdry -----*/
	nc = buf[1];
	nv = orion_cal_get_nv(nc);
	m_bdry = (buf[3]<<8) + buf[2];	/*m_bdry = m_bdry_15_8 << 8 + m_bdry_7_0*/

	tmp_numer = (unsigned long)g_fxtal_kHz * (unsigned long)m_bdry;
	tmp_denom = (unsigned long)g_fdiv * (unsigned long)nv;
	c_fN_min[7] = (unsigned int) (tmp_numer / tmp_denom);
	
	c_fN_min[6] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 6);
	c_fN_min[5] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 8);
	c_fN_min[4] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 12);
	c_fN_min[3] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 16);
	c_fN_min[2] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 24);
	c_fN_min[1] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 36);
	/*----- update c_fN_min[6] ~c_fN_min[1], by Hidy 06/30-----*/

	/*----- write -----*/
	val = 0x00;
	error = orion_cal_wmr(modulator,0x0181, 1, &val);		/*p_reg_p_iqik_m_cal*/
	if (error) goto exit;

	
	//error = orion_cal_rmr(modulator ,0x0048, 3, pd_ori);	//p_reg_t_pd_7_0
	//if (error) goto exit;
	
	// turn on adc_i and adc_q
	val = (pd_ori[0] & 0xe7);
	error = orion_cal_wmr(modulator ,0x0048, 1, &val);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x0049, 1, &pd_ori[1]);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x004a, 1, &pd_ori[2]);
	if (error) goto exit;

	val = 0x07;
	error = orion_cal_wmr(modulator,0x0182, 1, &val);		//p_reg_p_iqik_duration
	if (error) goto exit;

	error = orion_cal_run_dccc_w_iqik(modulator, False);
	if (error) goto exit;

	// turn off clock from eagle, Michael 20140423
	val = 0x00;
	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xfb26, 1, &val);
	if (error) goto exit;

exit:
	return (error);
}
unsigned long orion_cal_setfreq(IT9510INFO* modulator, unsigned int bw_kHz, unsigned int rf_freq_kHz)
{
	unsigned long error = 0;
	unsigned char val_,val[7];
	unsigned char buf[4];
	unsigned int tmp, loc_vld_cnt;

	/*----- update c_fN_min[7] again, by MCH -----*/
	unsigned int m_bdry, nc, nv;
	unsigned long tmp_numer, tmp_denom;
	error = orion_cal_rmr(modulator,0x0103, 1, &buf[1]);	/*p_reg_p_fbc_n_code*/
	if (error) goto exit;
	error = orion_cal_rmr(modulator,0x0123, 2, &buf[2]);	/*r_reg_r_fbc_m_bdry_7_0*/
	if (error) goto exit;

	/*----- set_fbdry -----*/
	nc = buf[1];
	nv = orion_cal_get_nv(nc);
	m_bdry = (buf[3]<<8) + buf[2];	/*m_bdry = m_bdry_15_8 << 8 + m_bdry_7_0*/

	tmp_numer = (unsigned long)g_fxtal_kHz * (unsigned long)m_bdry;
	tmp_denom = (unsigned long)g_fdiv * (unsigned long)nv;
	c_fN_min[7] = (unsigned int) (tmp_numer / tmp_denom);
	/*----- update c_fN_min[7] again, by MCH -----*/
	c_fN_min[6] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 6);
	c_fN_min[5] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 8);
	c_fN_min[4] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 12);
	c_fN_min[3] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 16);
	c_fN_min[2] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 24);
	c_fN_min[1] = (unsigned int) (tmp_numer / (unsigned long)g_fdiv/ 36);
	/*----- update c_fN_min[6] ~c_fN_min[1], by Hidy 06/30-----*/
	


	/*----- read -----*/
	error = orion_cal_rmr(modulator,0x004C, 1, buf);		/*p_reg_t_ctrl*/
	if (error) goto exit;
	error = orion_cal_rmr(modulator,0x0181, 1, &buf[1]);	/*p_reg_p_iqik_m_cal*/
	if (error) goto exit;

	val[0] = orion_cal_get_lna_cap_sel(rf_freq_kHz);
	val[1] = orion_cal_get_lpf_bw(bw_kHz);

	/*----- set_rf_mode -----*/
//	unsigned char ctrl;
	if (rf_freq_kHz < c_fUHF_min) {
		val[2] = buf[0]&0xE7;			/*ctrl<4:3>=0b00 for VHF*/
	} else {
		val[2] = (buf[0]&0xE7) | 0x08;	/*ctrl<4:3>=0b01 for UHF*/
	}

	/*----- set_cal_freq -----*/
	tmp = orion_cal_get_cal_freq_iqik(rf_freq_kHz, buf[1]);
	val[3] = (unsigned char) (tmp & 0xFF);
	val[4] = (unsigned char) ((tmp >> 8) & 0xFF);

	/*----- set_lo_freq -----*/
	tmp = orion_cal_get_lo_freq(rf_freq_kHz);
	val[5] = (unsigned char) (tmp & 0xFF);
	val[6] = (unsigned char) ((tmp >> 8) & 0xFF);

	/*----- write -----*/

	if (g_orion_version == 2) {
		error = orion_cal_wmr(modulator,0x0246, 1, val);		/*p_reg_p_lnac_lna_cap_sel*/
	} else {
		error = orion_cal_wmr(modulator,0x0206, 1, val);		/*p_reg_p_lnac_lna_cap_sel*/
	}
	if (error) goto exit;
	error = orion_cal_wmr(modulator,0x0056, 1, &val[1]);	/*p_reg_t_lpf_bw*/
	if (error) goto exit;
	error = orion_cal_wmr(modulator,0x004C, 1, &val[2]);	/*p_reg_t_ctrl*/
	if (error) goto exit;
	error = orion_cal_wmr(modulator,0x004D, 2, &val[3]);	/*p_reg_t_cal_freq_7_0*/
	if (error) goto exit;
	error = orion_cal_wmr(modulator,0x004F, 2, &val[5]);	/*p_reg_t_lo_freq_7_0*/
	if (error) goto exit;

	/*---run loc by Michael 05/05/14---*/
	val_ =0x01;
	error = orion_cal_wmr(modulator,0x0241, 1, &val_);	/*p_reg_p_loc_rst*/
	if (error) goto exit;
	val_ =0x00;
	error = orion_cal_wmr(modulator,0x0241, 1, &val_);	/*p_reg_p_loc_rst*/
	if (error) goto exit;

	val_ =0x01;
	error = orion_cal_wmr(modulator,0x0240, 1, &val_);	/*p_reg_p_loc_vld*/
	if (error) goto exit;
	
	error = orion_cal_rmr(modulator,0x0240, 1, &val_);	/*p_reg_p_loc_vld*/
	if (error) goto exit;
	loc_vld_cnt = 0;
	while (val_ == 1) 
	{
		error = orion_cal_rmr(modulator,0x0240, 1, &val_);	/*p_reg_p_loc_vld*/
		if (error) goto exit;
		loc_vld_cnt++;
		if (loc_vld_cnt == 100) {break;}	
	}

	
	/*---modified by Hidy 07/16/10 apply AAGCI clear---*/
	val_ =0x01;
	error = orion_cal_wmr(modulator,0x00D0, 1, &val_);	/*p_reg_p_aagci_clear*/
	if (error) goto exit;
	
	val_ =0x00;
	error = orion_cal_wmr(modulator,0x00D0, 1, &val_);	/*p_reg_p_aagci_clear*/
	if (error) goto exit;

	

exit:
	return (error);
}
/*orion_cal_rmr (_read _multiple _registers)*/
unsigned long orion_cal_rmr(IT9510INFO* modulator,unsigned short mem_addr, unsigned char len, unsigned char *buf)
{	
	unsigned long error = 0;
	unsigned char temp[2] ;
	temp[1] = (uint8_t)mem_addr;
	temp[0] = (uint8_t)(mem_addr>>8);
	


	error = IT9510_writeGenericRegisters(modulator, 0x9E, 2,temp);
	if(error) goto exit; 
	error = IT9510_readGenericRegisters(modulator, 0x9E, (uint8_t)len,buf);

exit:
	return (error);
}
/*orion_cal_wmr (_write _multiple _registers)*/
unsigned long orion_cal_wmr(IT9510INFO* modulator, unsigned short mem_addr, unsigned char len, unsigned char *val)
{
	unsigned char temp[255] ;
	unsigned char write_len;
	int i;
	temp[1] = (uint8_t)mem_addr;
	temp[0] = (uint8_t)(mem_addr>>8);
	write_len = 2;

	for(i=0; i<len; i++){

		temp[write_len] = val[i];
		write_len++; 
	}

	return(IT9510_writeGenericRegisters(modulator, 0x9E, write_len,temp));
	

}


unsigned int orion_cal_get_cal_freq_iqik(unsigned int rf_freq_kHz, unsigned char iqik_m_cal)
{
	unsigned int nc, nv, mv, cal_freq;

	cal_freq = orion_cal_get_freq_code(rf_freq_kHz, &nc, &nv, &mv);
	if (g_clock_mode==0 && (iqik_m_cal < ORION_IQIK_M_CAL_MID)) {
		mv = mv + ((((unsigned int)iqik_m_cal)*nv*9)>>5);
	}
	else if (g_clock_mode==0 && (iqik_m_cal >= ORION_IQIK_M_CAL_MID)) {
		iqik_m_cal = ORION_IQIK_M_CAL_MAX - iqik_m_cal;
		mv = mv - ((((unsigned int)iqik_m_cal)*nv*9)>>5);
	}
	else if (g_clock_mode==1 && (iqik_m_cal < ORION_IQIK_M_CAL_MID)) {
		mv = mv + ((((unsigned int)iqik_m_cal)*nv)>>1);
	}
	else {	/*(g_clock_mode==1 && (iqik_m_cal >= ORION_IQIK_M_CAL_MID))*/
		iqik_m_cal = ORION_IQIK_M_CAL_MAX - iqik_m_cal;
		mv = mv - ((((unsigned int)iqik_m_cal)*nv)>>1);
	}
	cal_freq = ((nc&0x07) << 13) + mv;

	return cal_freq;
}
unsigned int orion_cal_get_freq_code(unsigned int rf_freq_kHz, unsigned int *nc, unsigned int *nv, unsigned int *mv)
{
	unsigned int freq_code;
	unsigned long tmp_tg, tmp_cal, tmp_m;

	*nc = orion_cal_get_nc(rf_freq_kHz);
	*nv = orion_cal_get_nv(*nc);

	tmp_tg = (unsigned long)rf_freq_kHz * (unsigned long)(*nv) * (unsigned long)g_fdiv;
	tmp_m = (tmp_tg / (unsigned long)g_fxtal_kHz);
	tmp_cal = tmp_m * (unsigned long)g_fxtal_kHz;
	if ((tmp_tg-tmp_cal) >= (g_fxtal_kHz>>1)) {tmp_m = tmp_m+1;}
	*mv = (unsigned int) (tmp_m);

	freq_code = (((*nc)&0x07) << 13) + (*mv);
	return freq_code;
}
unsigned int orion_cal_get_nc(unsigned int rf_freq_kHz)
{
	unsigned int nc;
	if ((rf_freq_kHz <= c_fN_min[1]))										{nc = 0;}	/*74*/
	else if ((rf_freq_kHz > c_fN_min[1]) && (rf_freq_kHz <= c_fN_min[2]))	{nc = 1;}	/*74 111*/	
	else if ((rf_freq_kHz > c_fN_min[2]) && (rf_freq_kHz <= c_fN_min[3]))	{nc = 2;}	/*111 148*/
	else if ((rf_freq_kHz > c_fN_min[3]) && (rf_freq_kHz <= c_fN_min[4]))	{nc = 3;}	/*148 222*/	
	else if ((rf_freq_kHz > c_fN_min[4]) && (rf_freq_kHz <= c_fN_min[5]))	{nc = 4;}	/*222 296*/	
	else if ((rf_freq_kHz > c_fN_min[5]) && (rf_freq_kHz <= c_fN_min[6]))	{nc = 5;}	/*296 445*/
	else if ((rf_freq_kHz > c_fN_min[6]) && (rf_freq_kHz <= c_fN_min[7]))	{nc = 6;}	/*445 573*/
	else if ((rf_freq_kHz > c_fN_min[7]) && (rf_freq_kHz <= c_fN_min[8]))	{nc = 7;}	/*573 890*/
	else 																	{nc = 8;}	/*L-band*/

	return nc;
}
unsigned int orion_cal_get_nv(unsigned int nc)
{
	unsigned int nv;
	switch(nc) {
		case 0:	nv = 48;	break;
		case 1:	nv = 32;	break;
		case 2:	nv = 24;	break;
		case 3:	nv = 16;	break;
		case 4:	nv = 12;	break;
		case 5:	nv = 8;	break;
		case 6:	nv = 6;	break;
		case 7:	nv = 4;	break;
		case 8: nv = 2; break;	/*L-band*/
		default:	nv = 2;	break;
	}
	return nv;
}


unsigned char orion_cal_get_lna_cap_sel(unsigned int rf_freq_kHz)
{
	unsigned char lna_cap_sel;
	if		(rf_freq_kHz <= c_fband_min[1])									{lna_cap_sel=0;}	/*<=440*/
	else if	(rf_freq_kHz > c_fband_min[1] && rf_freq_kHz <= c_fband_min[2])	{lna_cap_sel=1;}	/*440 484*/
	else if	(rf_freq_kHz > c_fband_min[2] && rf_freq_kHz <= c_fband_min[3])	{lna_cap_sel=2;}	/*484 533*/
	else if	(rf_freq_kHz > c_fband_min[3] && rf_freq_kHz <= c_fband_min[4])	{lna_cap_sel=3;}	/*533 587*/
	else if	(rf_freq_kHz > c_fband_min[4] && rf_freq_kHz <= c_fband_min[5])	{lna_cap_sel=4;}	/*587 645*/ 
	else if	(rf_freq_kHz > c_fband_min[5] && rf_freq_kHz <= c_fband_min[6])	{lna_cap_sel=5;}	/*645 710*/
	else if	(rf_freq_kHz > c_fband_min[6] && rf_freq_kHz <= c_fband_min[7])	{lna_cap_sel=6;}	/*710 782*/
	else																	{lna_cap_sel=7;}	/*>782*/
	return lna_cap_sel;
}
unsigned char orion_cal_get_far_aci_idx_skip(unsigned int rf_freq_kHz)
{
	unsigned char far_aci_idx_skip;
	if		(rf_freq_kHz <= c_faraciband_min[0])										{far_aci_idx_skip=0;}	/*<=474*/
	else if	(rf_freq_kHz > c_faraciband_min[0] && rf_freq_kHz <= c_faraciband_min[1])	{far_aci_idx_skip=1;}	/*474 538*/
	else if	(rf_freq_kHz > c_faraciband_min[1] && rf_freq_kHz <= c_faraciband_min[2])	{far_aci_idx_skip=2;}	/*538 602*/
	else if	(rf_freq_kHz > c_faraciband_min[2] && rf_freq_kHz <= c_faraciband_min[3])	{far_aci_idx_skip=3;}	/*602 666*/
	else if	(rf_freq_kHz > c_faraciband_min[3] && rf_freq_kHz <= c_faraciband_min[4])	{far_aci_idx_skip=4;}	/*666 730*/ 
	else if	(rf_freq_kHz > c_faraciband_min[4] && rf_freq_kHz <= c_faraciband_min[5])	{far_aci_idx_skip=5;}	/*730 794*/
	else if	(rf_freq_kHz > c_faraciband_min[5] && rf_freq_kHz <= c_faraciband_min[6])	{far_aci_idx_skip=6;}	/*794 858*/
	else																				{far_aci_idx_skip=7;}	/*>858*/
	return far_aci_idx_skip;
}

void orion_cal_get_far_aci_freq_lna(unsigned int rf_freq_kHz)
{
	int i,k;
	unsigned char j; 
	unsigned int tmp_freq_kHz, tmp_freq_code;
	unsigned int nc, nv, mv;
	
	j=0; k=0;
	for (i=-7;i<=7;i++) {
		if (i==0) {
			g_far_aci_idx_skip = j;	/*record g_far_aci_idx_skip*/
		}
		tmp_freq_kHz = rf_freq_kHz + i*g_far_aci_freq_inc;
		if (tmp_freq_kHz >= g_far_aci_freq_min && tmp_freq_kHz < g_far_aci_freq_max) {
			g_far_aci_freq_list[j] = tmp_freq_kHz;
			tmp_freq_code = orion_cal_get_freq_code(tmp_freq_kHz, &nc, &nv, &mv);
			g_far_aci_freq_code[k] = (unsigned char) (tmp_freq_code & 0xFF);
			g_far_aci_freq_code[k+1] = (unsigned char) ((tmp_freq_code >> 8) & 0xFF);
			g_far_aci_lna_cap_sel[j] = orion_cal_get_lna_cap_sel(tmp_freq_kHz);
			j++; k+=2;
		}
		if (j>=8) break;		/*just in case*/
	}
}
unsigned int orion_cal_get_lo_freq(unsigned int rf_freq_kHz)
{
	unsigned int nc, nv, mv, lo_freq;
	lo_freq = orion_cal_get_freq_code(rf_freq_kHz, &nc, &nv, &mv);
	return lo_freq;
}

unsigned char orion_cal_get_lpf_bw(unsigned int bw_kHz)
{
	unsigned char lpf_bw;
	switch(bw_kHz) {
		case 5000:	lpf_bw = 0;	break;	/*5.0MHz*/
		case 5500:	lpf_bw = 1;	break;	/*5.5MHz*/
		case 6000:	lpf_bw = 2;	break;	/*6.0MHz*/
		case 6500:	lpf_bw = 3;	break;	/*6.5MHz*/
		case 7000:	lpf_bw = 4;	break;	/*7.0MHz*/
		case 7500:	lpf_bw = 5;	break;	/*7.5MHz*/
		case 8000:	lpf_bw = 6;	break;	/*8.0MHz*/
		case 8500:	lpf_bw = 7;	break;	/*8.5MHz*/
		default:	lpf_bw = 6;	break;	/*default: 8MHz*/
	}
	return lpf_bw;
}

unsigned long orion_cal_get_param(IT9510INFO* modulator, OrionParam_enum items,  OrionParam *data)
{
    unsigned long error = OrionMsg_OK;
    unsigned char buf[4] = {0,};

    if (items == 0) {        
        error = OrionMsg_Param_not_match;
        goto exit;
	}

    if (items & ORION_GAIN) {
		error = orion_cal_rmr(modulator,0x00CA, 4, buf);	/*r_reg_r_rf_lna_gain*/
		if (error) goto exit;

        data->Gain.Gain1        = (int) buf[0];
        data->Gain.Gain2        = (int) buf[1];
        data->Gain.Gain3        = (int) buf[2];
        data->Gain.Gain4        = (int) buf[3];
        data->Gain.RF_Backoff   = (int) (7 - buf[0]) * 6 + (7 - buf[1]) * 3;	/*RF_BACKOFF*/
        data->Gain.BB_Backoff   = (int) (3 - buf[2]) * 6 + (15 - buf[3]);		/*BB_BACKOFF*/
    }
    
    if (items & ORION_DCC) {
		error = orion_cal_rmr(modulator,0x0153, 2, buf);	/*r_reg_r_dcc_i_acc*/
		if (error) goto exit;

        data->DCC.DCC1          = (int) buf[0];
        data->DCC.DCC2          = (int) buf[1];
    }
    
    if (items & ORION_RF_GMAX) {
        data->RF_Gmax           = g_rf_gmax;
    }
    
    if (items & ORION_TOTAL_GMAX) {
        data->Total_Gmax        = g_total_gmax;
    }
    
    if (items & ORION_P_INBAND_SHIFT) {
        data->P_Inband_Shift    = g_p_inband_shift;
    }
    
    if (items & ORION_P_ALL) {
		error = orion_cal_rmr(modulator,0x00CA, 4, buf);	/*r_reg_r_rf_lna_gain*/ //tom:wmr??
		if (error) goto exit;

        data->P_All.P1          = (int) buf[0];
        data->P_All.P2          = (int) buf[1];
        data->P_All.P3          = (int) buf[2];
        data->P_All.P4          = (int) buf[3];
        data->P_All.RF_Backoff  = (7 - buf[0]) * 6 + (7 - buf[1]) * 3;	/*RF_BACKOFF*/
        data->P_All.BB_Backoff  = (3 - buf[2]) * 6 + (15 - buf[3]);	/*BB_BACKOFF*/
        data->P_All.RF_Gain     = g_rf_gmax - data->P_All.RF_Backoff * 10;
        data->P_All.Total_Gain  = g_total_gmax - ((data->P_All.RF_Backoff + data->P_All.BB_Backoff) * 10);
    }

exit:
    return (error);
}

unsigned long orion_cal_set_param(IT9510INFO* modulator, OrionParam_enum items,  OrionParam *data)
{
    unsigned long error = OrionMsg_OK;
    unsigned char buf[4] = {0,};

    if (items == 0) {
        error = OrionMsg_Param_not_match;
        goto exit;
    }

    if (items & ORION_GAIN) {
		buf[0] = 1;
		buf[1] = 1;
		error = orion_cal_wmr(modulator,0x00C0, 2, buf);	/*p_reg_p_rf_agc_mode*/
		if (error) goto exit;
		
        buf[0] = (unsigned char) data->Gain.Gain1;
        buf[1] = (unsigned char) data->Gain.Gain2;
        buf[2] = (unsigned char) data->Gain.Gain3;
        buf[3] = (unsigned char) data->Gain.Gain4;
		error = orion_cal_wmr(modulator,0x00C4, 4, buf);	/*p_reg_p_rf_lna_gain*/
		if (error) goto exit;
    }
    
    if (items & ORION_RF_GMAX) {
        g_rf_gmax = data->RF_Gmax;
    }
    
    if (items & ORION_TOTAL_GMAX) {
        g_total_gmax = data->Total_Gmax;
    }
    
    if (items & ORION_P_INBAND_SHIFT) {
        g_p_inband_shift = data->P_Inband_Shift;
    }

exit:
    return (error);
}

unsigned long orion_cal_set_registers(IT9510INFO* modulator)
{
	unsigned long error = 0;
	unsigned char val;

	val = g_clock_mode;

	error = orion_cal_wmr(modulator,0x0085, 1, &val);	//p_reg_p_tsm_init_clock_mode

	if (error) goto exit;

	val = 0x07;
	error = orion_cal_wmr(modulator,0x0103, 1, &val);	/*p_reg_p_fbc_n_code*/
	if (error) goto exit;
    
	val = 0xFF;
	error = orion_cal_wmr(modulator,0x0102, 1, &val);	/*p_reg_p_fbc_pll_del*/
	if (error) goto exit;
    error = orion_cal_wmr(modulator,0x0242, 1, &val);	/*p_reg_p_loc_pll_del*/
	if (error) goto exit;
	error = orion_cal_wmr(modulator,0x0282, 1, &val);	/*p_reg_p_cpllc_pll_del*/
	if (error) goto exit;
    error = orion_cal_wmr(modulator,0x0203, 1, &val);	/*p_reg_p_lnac_lna_cap_del*/
	if (error) goto exit;

	/*--- modified by Hidy---06/30--- */
	//val = 0x72;
	val = 0x76;
	error = orion_cal_wmr(modulator,0x0062, 1, &val);	/*p_reg_t_ctrl_new_7_0*/
    if (error) goto exit;
	
	/*--- modified by Hidy---07/16--- default value is 2a*/
	//val =0x02;
	//error = orion_cal_wmr(0x0063, 1, &val);	/*p_reg_t_ctrl_new_15_8*/
    //if (error) goto exit;

	val =0x00;
	error = orion_cal_wmr(modulator,0x0064, 1, &val);	/*p_reg_t_ctrl_new_23_16*/
    if (error) goto exit;
    
	
	error = orion_cal_wmr(modulator,0x0245, 1, &val);	/*p_reg_p_loc_ceil*/
    if (error) goto exit;

	val =0x03;
	error = orion_cal_wmr(modulator,0x011D, 1, &val);	/*p_reg_p_fbc_lo_bias_0*/
    if (error) goto exit;
    
	val =0x02;
	error = orion_cal_wmr(modulator,0x011E, 1, &val);	/*p_reg_p_fbc_lo_bias_1*/
    if (error) goto exit;
	
	error = orion_cal_wmr(modulator,0x011F, 1, &val);	/*p_reg_p_fbc_lo_bias_2*/
    if (error) goto exit;
	
	error = orion_cal_wmr(modulator,0x0120, 1, &val);	/*p_reg_p_fbc_lo_bias_3*/
    if (error) goto exit;

	val =0x08;
	error = orion_cal_wmr(modulator,0x00D1, 1, &val);	/*p_reg_p_aagci_dccc_pga2*/
    if (error) goto exit;
	
	/*--- modified by Hidy---07/16--- default value is 0*/
	//val =0x01;
	//error = orion_cal_wmr(0x00C0, 1, &val);	/*p_reg_p_rf_agc_mode*/
    //if (error) goto exit;
   
	//error = orion_cal_wmr(0x00C1, 1, &val);	/*p_reg_p_bb_agc_mode*/
    //if (error) goto exit;

	val =0x00;
	error = orion_cal_wmr(modulator,0x00DA, 1, &val);	/*p_reg_p_aagci_apply_loc*/
    if (error) goto exit;

	/*---Modified by Hidy 07/09---*/
	val =0x2E;
	error = orion_cal_wmr(modulator,0x0104, 1, &val);	/*p_reg_p_fbc_m1200_min_7_0*/
    if (error) goto exit;

	val =0x0D;
	error = orion_cal_wmr(modulator,0x0105, 1, &val);	/*p_reg_p_fbc_m1200_min_12_8*/
    if (error) goto exit;

	val =0x2E;
	error = orion_cal_wmr(modulator,0x0106, 1, &val);	/*p_reg_p_fbc_m1200_max_7_0*/
    if (error) goto exit;

	val =0x0F;
	error = orion_cal_wmr(modulator,0x0107, 1, &val);	/*p_reg_p_fbc_m1200_max_12_8*/
    if (error) goto exit;

	val =0x2E;
	error = orion_cal_wmr(modulator,0x0108, 1, &val);	/*p_reg_p_fbc_m1200_mid_7_0*/
    if (error) goto exit;

	val =0x0E;
	error = orion_cal_wmr(modulator,0x0109, 1, &val);	/*p_reg_p_fbc_m1200_mid_12_8*/
    if (error) goto exit;

	val =0xC6;
	error = orion_cal_wmr(modulator,0x010A, 1, &val);	/*p_reg_p_fbc_m2048_min_7_0*/
    if (error) goto exit;

	val =0x0D;
	error = orion_cal_wmr(modulator,0x010B, 1, &val);	/*p_reg_p_fbc_m2048_min_12_8*/
    if (error) goto exit;

	val =0xC6;
	error = orion_cal_wmr(modulator,0x010C, 1, &val);	/*p_reg_p_fbc_m2048_max_7_0*/
    if (error) goto exit;

	val =0x0F;
	error = orion_cal_wmr(modulator,0x010D, 1, &val);	/*p_reg_p_fbc_m2048_max_12_8*/
    if (error) goto exit;

	val =0xC6;
	error = orion_cal_wmr(modulator,0x010E, 1, &val);	/*p_reg_p_fbc_m2048_mid_7_0*/
    if (error) goto exit;

	val =0x0E;
	error = orion_cal_wmr(modulator,0x010F, 1, &val);	/*p_reg_p_fbc_m2048_mid_12_8*/
    if (error) goto exit;
	
	/*---modified by Hidy 07/16/10 apply AAGCI clear---*/
	val =0x01;
	error = orion_cal_wmr(modulator,0x00D0, 1, &val);	/*p_reg_p_aagci_clear*/
    if (error) goto exit;
	
	val =0x00;
	error = orion_cal_wmr(modulator,0x00D0, 1, &val);	/*p_reg_p_aagci_clear*/
    if (error) goto exit;

	// Added by Hidy 09/07/10
	val =0x07;                          /*p_reg_p_dcc_timer_thld_track*/
	error = orion_cal_wmr(modulator,0x014A, 1, &val);
	if (error) goto exit;

	// Added by Hidy 11/12/10
	val =0x00;
	error = orion_cal_wmr(modulator,0x00E4, 1, &val);	/*p_reg_aagci_dcxo_mode*/
    if (error) goto exit;
exit:
	return 0;
}

unsigned long orion_cal_run_dccc_w_iqik(IT9510INFO* modulator, Bool log_on) 
{	
	unsigned long error = 0;
	unsigned char val, pd_ori[3], agc_ori[4], agc_mode_ori[2], dc_i_cal, dc_q_cal, dc_i_min, dc_q_min, i_start, i_end, q_start, q_end, dc_i, dc_q;
	unsigned int vi_abs_min, vq_abs_min, vi_abs, vq_abs;
	int pga1_end, vi_real, vi_imag, vq_real, vq_imag;
	unsigned short dcc_i_addr, dcc_q_addr;

	unsigned char dd = 4, pga1_start = 3;

	int pga1;
	//pga1_start = 3;
	pga1_end = 0;
	

	/*-------- setup parameter --------*/
	error = orion_cal_rmr(modulator ,0x0048, 3, pd_ori);	//p_reg_t_pd_7_0
	if (error) goto exit;
	error = orion_cal_rmr(modulator ,0x00c0, 2, agc_mode_ori);	//p_reg_p_rf_agc_mode
	if (error) goto exit;
	error = orion_cal_rmr(modulator ,0x00c4, 4, agc_ori);	//p_reg_p_rf_lna_gain
	if (error) goto exit;

	// turn off LNA
	val = (pd_ori[1] & 0xdf) + 0x20;
	error = orion_cal_wmr(modulator ,0x0049, 1, &val);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x004a, 1, &pd_ori[2]);
	if (error) goto exit;
	
	val = 0x02;
	error = orion_cal_wmr(modulator ,0x0140, 1, &val);	//p_reg_p_dcc_mode
	if (error) goto exit;
	/*----------------------------------*/

	for (pga1 = pga1_start; pga1 >= pga1_end; pga1--)
	{
		// set agc 7 5 pga1 0
		error = orion_cal_set_agc(modulator, 7, 5, (unsigned char)pga1, 0);
		if (error) goto exit;

		dcc_i_addr = 0x0141 + (unsigned char)pga1*2;
		dcc_q_addr = 0x0142 + (unsigned char)pga1*2;
		error = orion_cal_rmr(modulator , dcc_i_addr, 1, &dc_i_cal);
		if (error) goto exit;
		error = orion_cal_rmr(modulator , dcc_q_addr, 1, &dc_q_cal);
		if (error) goto exit;

		if (log_on) 
		{
			printf("pga1 = %d\n", pga1);
			printf("%d\t%d\n", dc_i_cal, dc_q_cal);
		}

		vi_abs_min = 10000;
		vq_abs_min = 10000;
		dc_i_min = 32;
		dc_q_min = 32;
		i_start = dc_i_cal - dd;
		i_end = dc_i_cal + dd;
		q_start = dc_q_cal - dd;
		q_end = dc_q_cal + dd;

		dc_q = q_start;
		for (dc_i = i_start; dc_i <= i_end; dc_i++)
		{
			error = orion_cal_wmr(modulator ,dcc_i_addr, 1, &dc_i);
			if (error) goto exit;
			error = orion_cal_wmr(modulator ,dcc_q_addr, 1, &dc_q);
			if (error) goto exit;

			error = orion_cal_run_iqik(modulator);
			if (error) goto exit;
			error = orion_cal_read_iqik(modulator, &vi_real, &vi_imag, &vq_real, &vq_imag);
			if (error) goto exit;

			vi_abs = vi_real*vi_real + vi_imag*vi_imag;
			vq_abs = vq_real*vq_real + vq_imag*vq_imag;

			if (vi_abs <= vi_abs_min || dc_i == i_start)
			{
				vi_abs_min = vi_abs;
				dc_i_min = dc_i;
			}
			if (vq_abs <= vq_abs_min || dc_q == q_start) 
			{
				vq_abs_min = vq_abs;
				dc_q_min = dc_q;
			}
			if (log_on) 
			{
				printf("%d\t%d\t%d\t%d\n", dc_i, dc_q, vi_abs, vq_abs);
			}
			dc_q++;
		}

		error = orion_cal_wmr(modulator ,dcc_i_addr, 1, &dc_i_min);
		if (error) goto exit;
		error = orion_cal_wmr(modulator ,dcc_q_addr, 1, &dc_q_min);
		if (error) goto exit;
		
		if (log_on)
		{
			IT9510User_printf("%d\t%d\n", dc_i_min, dc_q_min);
		}
	}
	
	/*-------- resume parameter --------*/
	error = orion_cal_wmr(modulator ,0x0048, 3, pd_ori);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c0, 2, agc_mode_ori);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c4, 4, agc_ori);
	if (error) goto exit;
	/*----------------------------------*/

exit:
	return error;
}

unsigned long orion_cal_set_agc(IT9510INFO* modulator, unsigned char lna, unsigned char pgc, unsigned char pga1, unsigned char pga2)
{
	unsigned long error;
	unsigned char val;
	
	// set agc mode
	val = 0x01;
	error = orion_cal_wmr(modulator ,0x00c0, 1, &val);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c1, 1, &val);
	if (error) goto exit;

	// set agc gain
	error = orion_cal_wmr(modulator ,0x00c4, 1, &lna);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c5, 1, &pgc);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c6, 1, &pga1);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c7, 1, &pga2);
	if (error) goto exit;

exit:
	return error;
}

unsigned long orion_cal_run_iqik(IT9510INFO* modulator)
{
	unsigned long error;
	unsigned char val, iqik_ste;
	unsigned int iqikc_vld_cnt;

	val = 0x00;
	error = orion_cal_wmr(modulator, 0x0180, 1, &val);	//p_reg_p_iqik_mode
	if (error) goto exit;

	val = 0x02;
	error = orion_cal_wmr(modulator, 0x0180, 1, &val);
	if (error) goto exit;
	
	error = orion_cal_rmr(modulator, 0x0194, 1, &iqik_ste); //p_reg_p_iqik_ste
	if (error) goto exit;
	
	iqikc_vld_cnt = 0;
	while (iqik_ste != 2)
	{
		error = orion_cal_rmr(modulator, 0x0194, 1, &iqik_ste); //p_reg_p_iqik_ste
		if (error) goto exit;
	
		iqikc_vld_cnt++;
		if (iqikc_vld_cnt == 100) {break;}
	}
	
exit:
	return error;
}

unsigned long orion_cal_read_iqik(IT9510INFO* modulator, int *vi_real, int *vi_imag, int *vq_real, int *vq_imag)
{
	unsigned long error;
	unsigned char val[3];

	error = orion_cal_rmr(modulator, 0x0188, 3, val);
	if (error) goto exit;
	*vi_real = (val[2] << 16) + (val[1] << 8) + val[0];

	error = orion_cal_rmr(modulator, 0x018b, 3, val);
	if (error) goto exit;
	*vi_imag = (val[2] << 16) + (val[1] << 8) + val[0];

	error = orion_cal_rmr(modulator, 0x018e, 3, val);
	if (error) goto exit;
	*vq_real = (val[2] << 16) + (val[1] << 8) + val[0];

	error = orion_cal_rmr(modulator, 0x0191, 3, val);
	if (error) goto exit;
	*vq_imag = (val[2] << 16) + (val[1] << 8) + val[0];

	// Sign conversion UQ(16,0) -> Q(16,0)
	*vi_real = ((*vi_real + 65536) % 131072) - 65536;
	*vi_imag = ((*vi_imag + 65536) % 131072) - 65536;
	*vq_real = ((*vq_real + 65536) % 131072) - 65536;
	*vq_imag = ((*vq_imag + 65536) % 131072) - 65536;

exit:
	return error;
}

unsigned long orion_dc_scan (IT9510INFO* modulator, Bool ofs) 
{
	uint32_t	error, limit;
	uint8_t	val[6], agc_ori[4], agc_mode_ori[2];
	//Word	addr_start, addr_end, reg_addr;
	uint8_t	val_org[6];
	int		dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, dc_i_min, dc_q_min;
	//FILE	*fid, *fid_reg;
	//char	line[100], a[5], b[50];
	ORION_SCAN_CORE_FUNC orion_dc_scan_core_ptr = NULL;

#ifdef FW_SUPPORT
	orion_dc_scan_core_ptr = orion_dc_scan_core2;
#else
	orion_dc_scan_core_ptr = orion_dc_scan_core;
#endif
     
	error = IT9510_readRegisters (modulator, Processor_OFDM, 0xF752, 6, val_org);
	if (error) goto exit;

	// read original agc setting
	error = orion_cal_rmr(modulator ,0x00c0, 2, agc_mode_ori);	//p_reg_p_rf_agc_mode
	if (error) goto exit;
	error = orion_cal_rmr(modulator ,0x00c4, 4, agc_ori);	//p_reg_p_rf_lna_gain
	if (error) goto exit;
	// set agc 4 7 0 0
	error = orion_agc_dcc (modulator, 0);
	//error = orion_cal_set_agc(modulator, 5, 7, 0, 0);
	if (error) goto exit;


	// enable sine tone
	error = IT9510_setSineTone (modulator, True);
	if (error) goto exit;

	// set scaling facotr of Tx 64
	val[0] = 0x40;
	val[1] = 0x00;
	val[2] = 0x00;
	val[3] = 0x00;
	val[4] = 0x40;
	val[5] = 0x00;
	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xF752, 6, val);
	if (error) goto exit;

	error = IT9510_setDCCalibrationValue (modulator, 0, 0);
	if (error) goto exit;

	if (ofs)
	{
		// large scale
		dc_i_ctr = 32;
		dc_i_range = 32;
		dc_q_ctr = 32;
		dc_q_range = 32;
		//limit = 10000000;
		limit = 100000000;


		error = orion_dc_scan_core_ptr(modulator, dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, True, limit, &dc_i_min, &dc_q_min);
		if (error) goto exit;

		// median scale
		dc_i_ctr = dc_i_min;
		dc_i_range = 8;
		dc_q_ctr = dc_q_min;
		dc_q_range = 8;
		//limit = 1000000;
		limit = 100000000;
		error = orion_dc_scan_core_ptr(modulator, dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, True, limit, &dc_i_min, &dc_q_min);
		if (error) goto exit;

		error = IT9510_setOFSCalibrationValue (modulator, (uint8_t)dc_i_min, (uint8_t)dc_q_min);
		if (error) goto exit;
	}

	//if (!ofs || dc_i_min == 63 || dc_i_min == 0 || dc_q_min == 63 || dc_q_min == 0)
	//{
		// large scale
		dc_i_ctr = 0;
		dc_i_range = 256;
		dc_q_ctr = 0;
		dc_q_range = 256;
		limit = 10000000;
		//limit = 1000000000;
		error = orion_dc_scan_core_ptr(modulator, dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, False, limit, &dc_i_min, &dc_q_min);
		if (error) goto exit;

		// median scale
		dc_i_ctr = dc_i_min;
		dc_i_range = 64;
		dc_q_ctr = dc_q_min;
		dc_q_range = 64;
		limit = 1000000;
		//limit = 1000000000;
		error = orion_dc_scan_core_ptr(modulator, dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, False, limit, &dc_i_min, &dc_q_min);
		if (error) goto exit;

		// small scale
		dc_i_ctr = dc_i_min;
		dc_i_range = 8;
		dc_q_ctr = dc_q_min;
		dc_q_range = 8;
		limit = 100000;
		//limit = 1000000000;
		error = orion_dc_scan_core_ptr(modulator, dc_i_ctr, dc_i_range, dc_q_ctr, dc_q_range, False, limit, &dc_i_min, &dc_q_min);
		if (error) goto exit;
	
		error = IT9510_setDCCalibrationValue (modulator, dc_i_min, dc_q_min);
		if (error) goto exit;
		printf("dc_i_min = %d, dc_q_min =%d\n",dc_i_min,dc_q_min);
	//}

	///////////////////////////////////////////////////////////////////////////////////////////////

	// disable sine tone
	error = IT9510_setSineTone (modulator, False);
	if (error) goto exit;

	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xF752, 6, val_org);
	if (error) goto exit;

	// set agc original setting
	error = orion_cal_wmr(modulator ,0x00c0, 2, agc_mode_ori);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x00c4, 4, agc_ori);
	if (error) goto exit;

exit:
	return(error);
}

unsigned long orion_dc_scan_core (IT9510INFO* modulator, int dc_i_ctr, int dc_i_range, int dc_q_ctr, int dc_q_range, Bool ofs, unsigned long limit, int* dc_i_min, int* dc_q_min)
{
	uint32_t error = 0, ssum_min = 0, ssum_pre, ssum;
	int dc_i_end, dc_q_end, dc_i, dc_q, vi_real, vi_imag, vq_real, vq_imag, dc_i_inc, dc_q_inc;

	ORION_CAL_RUN_IQIK_FUNC orion_cal_run_iqik_ptr = NULL;
	ORION_SET_DC_FUNC orion_set_dc_ptr = NULL;
	ORION_SET_OFS_FUNC orion_set_ofs_ptr = NULL;
	ORION_CAL_READ_IQIK_FUNC orion_cal_read_iqik_ptr = NULL;

#ifdef FW_SUPPORT_DBG
	orion_cal_run_iqik_ptr = orion_cal_run_iqik2;
	orion_set_dc_ptr = setDCCalibrationValue2;
	orion_set_ofs_ptr = setOFSCalibrationValue2;
	orion_cal_read_iqik_ptr = orion_cal_read_iqik2;
#else
	orion_cal_run_iqik_ptr = orion_cal_run_iqik;
	orion_set_dc_ptr = IT9510_setDCCalibrationValue;
	orion_set_ofs_ptr = IT9510_setOFSCalibrationValue;
	orion_cal_read_iqik_ptr = orion_cal_read_iqik;
#endif
	
	*dc_i_min = 0;
	*dc_q_min = 0;
	
    dc_i_inc = (int)(dc_i_range/8);
    dc_q_inc = (int)(dc_q_range/8);
                   
	// dc_i, - direction
	dc_i_end = dc_i_ctr - dc_i_range; 
	dc_q = dc_q_ctr;
	ssum_pre = 0;
	ssum = 0;
  
	if (ofs) { if (dc_i_end < 0) { dc_i_end = 0;} } 
	else { if (dc_i_end < -256) { dc_i_end = -256;} }

	for (dc_i = dc_i_ctr; dc_i >= dc_i_end; dc_i = dc_i - dc_i_inc)
	{
		if (ofs) { error = orion_set_ofs_ptr(modulator, (uint8_t)dc_i, (uint8_t)dc_q); }
		else { error = orion_set_dc_ptr(modulator, dc_i, dc_q); }
		if (error) goto exit;

		error = orion_cal_run_iqik_ptr(modulator);
		if (error) goto exit;

		error = orion_cal_read_iqik_ptr(modulator, &vi_real, &vi_imag, &vq_real, &vq_imag);
		if (error) goto exit;

		ssum_pre = ssum;
		ssum = (vi_real-vq_imag)*(vi_real-vq_imag) + (vi_imag+vq_real)*(vi_imag+vq_real);
			
		if (ssum <= ssum_min || dc_i == dc_i_ctr)
		{
			ssum_min = ssum;
    		*dc_i_min = dc_i;
		}

		//printf ("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",dc_i, dc_q, vi_real, vi_imag, vq_real, vq_imag, ssum);

		if (((long)ssum_pre <= (long)(ssum-limit)) && (ssum_pre != 0)) 
		{ break; }
	}

	// dc_i, + direction
	dc_i_end = dc_i_ctr + dc_i_range; 
	ssum_pre = 0;
	ssum = 0;
  
	if (ofs) { if (dc_i_end > 63) { dc_i_end = 63;} } 
	else { if (dc_i_end > 255) { dc_i_end = 255;} }

	for (dc_i = dc_i_ctr; dc_i <= dc_i_end; dc_i = dc_i + dc_i_inc)
	{
		if (ofs) { error = orion_set_ofs_ptr(modulator, (uint8_t)dc_i, (uint8_t)dc_q); }
		else { error = orion_set_dc_ptr(modulator, dc_i, dc_q); }
		if (error) goto exit;

		error = orion_cal_run_iqik_ptr(modulator);
		if (error) goto exit;

		error = orion_cal_read_iqik_ptr(modulator, &vi_real, &vi_imag, &vq_real, &vq_imag);
		if (error) goto exit;

		ssum_pre = ssum;
		ssum = (vi_real-vq_imag)*(vi_real-vq_imag) + (vi_imag+vq_real)*(vi_imag+vq_real);
			
		if (ssum <= ssum_min)
		{
			ssum_min = ssum;
    		*dc_i_min = dc_i;
		}

		//printf ("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",dc_i, dc_q, vi_real, vi_imag, vq_real, vq_imag, ssum);

		if ((long)(ssum_pre) <= (long)(ssum-limit) && ssum_pre != 0) { break; }
	}
	
	// dc_q, - direction
	dc_q_end = dc_q_ctr - dc_q_range; 
	dc_i = *dc_i_min;
	ssum_pre = 0;
	ssum = 0;
  
	if (ofs) { if (dc_q_end < 0) { dc_q_end = 0;} } 
	else { if (dc_q_end < -256) { dc_q_end = -256;} }

	for (dc_q = dc_q_ctr; dc_q >= dc_q_end; dc_q = dc_q - dc_q_inc)
	{
		if (ofs) { error = orion_set_ofs_ptr(modulator, (uint8_t)dc_i, (uint8_t)dc_q); }
		else { error = orion_set_dc_ptr(modulator, dc_i, dc_q); }
		if (error) goto exit;

		error = orion_cal_run_iqik_ptr(modulator);
		if (error) goto exit;

		error = orion_cal_read_iqik_ptr(modulator, &vi_real, &vi_imag, &vq_real, &vq_imag);
		if (error) goto exit;
		
		ssum_pre = ssum;
		ssum = (vi_real-vq_imag)*(vi_real-vq_imag) + (vi_imag+vq_real)*(vi_imag+vq_real);
			
		if (ssum <= ssum_min || dc_q == dc_q_ctr)
		{
			ssum_min = ssum;
    		*dc_q_min = dc_q;
		}

		//printf ("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",dc_i, dc_q, vi_real, vi_imag, vq_real, vq_imag, ssum);

		if ((long)(ssum_pre) <= (long)(ssum-limit) && ssum_pre != 0) { break; }
	}

	// dc_q, + direction
	dc_q_end = dc_q_ctr + dc_q_range; 
	ssum_pre = 0;
	ssum = 0;
  
	if (ofs) { if (dc_q_end > 63) { dc_q_end = 63;} } 
	else { if (dc_q_end > 255) { dc_q_end = 255;} }

	for (dc_q = dc_q_ctr; dc_q <= dc_q_end; dc_q = dc_q + dc_q_inc)
	{
		if (ofs) { error = orion_set_ofs_ptr(modulator, (uint8_t)dc_i, (uint8_t)dc_q); }
		else { error = orion_set_dc_ptr(modulator, dc_i, dc_q); }
		if (error) goto exit;

		error = orion_cal_run_iqik_ptr(modulator);
		if (error) goto exit;

		error = orion_cal_read_iqik_ptr(modulator, &vi_real, &vi_imag, &vq_real, &vq_imag);
		if (error) goto exit;

		ssum_pre = ssum;
		ssum = (vi_real-vq_imag)*(vi_real-vq_imag) + (vi_imag+vq_real)*(vi_imag+vq_real);
			
		if (ssum <= ssum_min)
		{
			ssum_min = ssum;
    		*dc_q_min = dc_q;
		}

		//printf ("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",dc_i, dc_q, vi_real, vi_imag, vq_real, vq_imag, ssum);

		if ((long)(ssum_pre) <= (long)(ssum-limit) && ssum_pre != 0) { break; }
	}

exit:
	return(error);
}

unsigned long orion_dc_scan_core2(IT9510INFO* modulator, int dc_i_ctr, int dc_i_range, int dc_q_ctr, int dc_q_range, Bool ofs, unsigned long limit, int* dc_i_min, int* dc_q_min)
{
	uint32_t error = 0;
	uint8_t tmp_buf[14] = {0};
	uint8_t i;

	tmp_buf[0] = 0x20; // run orion_dc_scan_core() function

	tmp_buf[1] = (dc_i_ctr >> 8) & 0xFF;
	tmp_buf[2] = dc_i_ctr & 0xFF;

	tmp_buf[3] = (dc_i_range >> 8) & 0xFF;
	tmp_buf[4] = dc_i_range & 0xFF;

	tmp_buf[5] = (dc_q_ctr >> 8) & 0xFF;
	tmp_buf[6] = dc_q_ctr & 0xFF;

	tmp_buf[7] = (dc_q_range >> 8) & 0xFF;
	tmp_buf[8] = dc_q_range & 0xFF;

	tmp_buf[9] = ofs & 0xFF;

	tmp_buf[10] = (limit >> 24) & 0xFF;
	tmp_buf[11] = (limit >> 16) & 0xFF;
	tmp_buf[12] = (limit >> 8) & 0xFF;
	tmp_buf[13] = limit & 0xFF;

	error = IT9510_writeRegisters2(modulator, Processor_LINK, 0x4900, 14, tmp_buf);
	if (error) goto exit;

 // get the orion_dc_scan_core() result
   uint8_t buf[4]={0};	
	error = IT9510_readRegisters2(modulator, Processor_LINK, 0x7E00, 4, buf);
	if (error) goto exit;
	
	*dc_i_min = (signed short)((buf[0] << 8) + buf[1]);
	*dc_q_min = (signed short)((buf[2] << 8) + buf[3]);

exit:
	return(error);
}

unsigned long orion_cal_run_iqik2(IT9510INFO* modulator)
{
	uint32_t error = 0;
	uint8_t tmp_buf[32] = {};
	uint8_t i;

	tmp_buf[0] = 0x21; // run orion_cal_run_iqik() function

	error = IT9510_writeRegisters(modulator, Processor_LINK, 0x4900, 1, tmp_buf);
	if (error) goto exit;

exit:
	return(error);
}

unsigned long setDCCalibrationValue2(IT9510INFO* modulator, int dc_i, int dc_q)
{
	uint32_t error = 0;
	uint8_t tmp_buf[32] = {};
	uint8_t i;

	tmp_buf[0] = 0x23; // run IT9510_setDCCaibrationValue() function

	tmp_buf[1] = (dc_i >> 8) & 0xFF;
	tmp_buf[2] = dc_i & 0xFF;

	tmp_buf[3] = (dc_q >> 8) & 0xFF;
	tmp_buf[4] = dc_q & 0xFF;

	error = IT9510_writeRegisters(modulator, Processor_LINK, 0x4900, 5, tmp_buf);
	if (error) goto exit;

exit:
	return(error);
}

unsigned long setOFSCalibrationValue2(IT9510INFO* modulator, uint8_t ofs_i, uint8_t ofs_q)
{
	uint32_t error = 0;
	uint8_t tmp_buf[32] = {};
	uint8_t i;

	tmp_buf[0] = 0x24; // run IT9510_setOFSCalibrationValue() function

	tmp_buf[1] = ofs_i & 0xFF;
	tmp_buf[2] = ofs_q & 0xFF;

	error = IT9510_writeRegisters(modulator, Processor_LINK, 0x4900, 3, tmp_buf);
	if (error) goto exit;

exit:
	return(error);
}

unsigned long orion_cal_read_iqik2(IT9510INFO* modulator, int *vi_real, int *vi_imag, int *vq_real, int *vq_imag)
{
	uint32_t error = 0;
	uint8_t tmp_buf[32] = {};
	uint8_t i;

	tmp_buf[0] = 0x22; // run orion_cal_read_iqik() function

	error = IT9510_writeRegisters(modulator, Processor_LINK, 0x4900, 1, tmp_buf);
	if (error) goto exit;

	// ZeroMemory(tmp_buf, sizeof(tmp_buf));
	for (i = 0; i < sizeof(tmp_buf); i++)
		tmp_buf[i] = 0;

	// get the orion_dc_scan_core() result

	error = IT9510_readRegisters(modulator, Processor_LINK, 0x7E00, 16, tmp_buf);
	if (error) goto exit;

	*vi_real = (int)((tmp_buf[0] << 24) + (tmp_buf[1] << 16) + (tmp_buf[2] << 8) + tmp_buf[3]);
	*vi_imag = (int)((tmp_buf[4] << 24) + (tmp_buf[5] << 16) + (tmp_buf[6] << 8) + tmp_buf[7]);
	*vq_real = (int)((tmp_buf[8] << 24) + (tmp_buf[9] << 16) + (tmp_buf[10] << 8) + tmp_buf[11]);
	*vq_imag = (int)((tmp_buf[12] << 24) + (tmp_buf[13] << 16) + (tmp_buf[14] << 8) + tmp_buf[15]);

// dbg
/*	int vi_real_, vi_imag_, vq_real_, vq_imag_;
	vi_real_ = (int)((tmp_buf[16] << 24) + (tmp_buf[17] << 16) + (tmp_buf[18] << 8) + tmp_buf[19]);
	vi_imag_ = (int)((tmp_buf[20] << 24) + (tmp_buf[21] << 16) + (tmp_buf[22] << 8) + tmp_buf[23]);
	vq_real_ = (int)((tmp_buf[24] << 24) + (tmp_buf[25] << 16) + (tmp_buf[26] << 8) + tmp_buf[27]);
	vq_imag_ = (int)((tmp_buf[28] << 24) + (tmp_buf[29] << 16) + (tmp_buf[30] << 8) + tmp_buf[31]);
*/
exit:
	return(error);
}

unsigned long orion_cal_read_tune_rssi(IT9510INFO* modulator, unsigned int *rssi_avg, unsigned int avg_times)
{
	unsigned char val[5], rssi_tuner[5];
	unsigned long error = 0, rssi_acc, rssi_tuner_com;
	unsigned int k;
	val[0] = 0x03;
	error = orion_cal_wmr(modulator ,0x005B, 1, val);
	if (error) goto exit;

	rssi_acc = 0;
	*rssi_avg = 0;

	for (k = 0; k < avg_times; k++) 
	{
		val[0] = 0x00;
		error = orion_cal_wmr(modulator ,0x0001, 1, val);
		if (error) goto exit;

		error = orion_cal_rmr(modulator ,0x000A, 5, val);
		if (error) goto exit;

		rssi_tuner[0] = (val[0] >> 6) ^ 0x02;
		rssi_tuner[1] = (val[1] >> 6);
		rssi_tuner[2] = (val[2] >> 6);
		rssi_tuner[3] = (val[3] >> 6);
		rssi_tuner[4] = (val[4] >> 6);
		rssi_tuner_com = (rssi_tuner[0] << 8) + (rssi_tuner[1] << 6) + (rssi_tuner[2] << 4) + (rssi_tuner[3] << 2) + rssi_tuner[4];
		rssi_acc = rssi_acc + rssi_tuner_com;
	}
	*rssi_avg = rssi_acc/avg_times;

exit:
	return (error);
}

unsigned long orion_agc_dcc (IT9510INFO* modulator, int tx_pw)
{
	unsigned long error = 0;
	unsigned char val, pd_ori[3];
	int add_gain, near_gain, gain_diff;
	int rssi_avg, near_idx;

	unsigned char lna_tb[29] = {7,7,7,7,7,7,7,7,6,6,6,5,5,5,5,5,3,3,3,3,3,3,3,3,2,1,1,1,0};
	unsigned char pgc_tb[29] = {7,6,5,4,5,6,7,6,7,6,5,7,6,5,4,3,7,6,5,4,3,2,1,0,1,2,1,0,0};
	unsigned char pga1_tb[29] = {3,3,3,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//double backoff_tb[29] = {0,2.5,5.3,8,10.7,13.6,17,19.5,22,24.5,27.2,29,31.5,34.3,37,39.9,41.8,44.4,47.1,49.8,52.7,56,58.9,62.2,64.8,67,70,73.2,75.3};
	unsigned int backoff_tb[29] = {0,25,53,8,107,136,17,195,22,245,272,29,315,343,37,399,418,444,471,498,527,56,589,622,648,67,70,732,753};
	unsigned int temp;
	unsigned int i;

	error = orion_cal_rmr(modulator ,0x0048, 3, pd_ori);	//p_reg_t_pd_7_0
	if (error) goto exit;

	// turn on adc_i and adc_q
	val = 0x3e;
	error = orion_cal_wmr(modulator ,0x0048, 1, &val);
	if (error) goto exit;
	error = orion_cal_wmr(modulator ,0x004a, 1, &pd_ori[2]);
	if (error) goto exit;

	if (tx_pw > -12 && tx_pw <= 0) 
	{
		error = orion_cal_set_agc(modulator, 3, 0, 0, 0);
		if (error) goto exit;

		error = orion_cal_read_tune_rssi(modulator, &temp, 100);
		if (error) goto exit;
		rssi_avg = (int)temp;
		//add_gain = (double(rssi_avg)-808)/13.5+34.276-3;

		add_gain = (rssi_avg-808)*100/135 + 34276 - 3000 ;


   
	} else if (tx_pw > -27 && tx_pw <= -12) 
	{
		error = orion_cal_set_agc(modulator, 3, 5, 0, 0);
		if (error) goto exit;

		error = orion_cal_read_tune_rssi(modulator, &temp, 100);
		if (error) goto exit;
	    rssi_avg = (int)temp;
		
		//add_gain = (double(rssi_avg)-808)/13.5+19.216-3;
		add_gain = (rssi_avg-808)*100/135 + 19216 - 3000;
	} else 
	{
		error = orion_cal_set_agc(modulator, 6, 5, 0, 0);
		if (error) goto exit;

		error = orion_cal_read_tune_rssi(modulator, &temp, 100);
		if (error) goto exit;
		rssi_avg = (int)temp;
		
		//add_gain = (double(rssi_avg)-808)/13.5-0.668-3;
		add_gain = (rssi_avg-808)*100/135 - 668 -3000;
    }

	add_gain = add_gain/1000;


	near_idx = 0;
	near_gain = 1000;
	for (i = 0; i < sizeof(backoff_tb)/sizeof(backoff_tb[0]); i++) 
	{
		//gain_diff = add_gain - backoff_tb[i];
		gain_diff = (add_gain*10 - backoff_tb[i])/10;
		if (abs(gain_diff) < abs(near_gain))
		{
			near_idx = i;
			near_gain = gain_diff;
		}
	}

	error = orion_cal_set_agc(modulator, lna_tb[near_idx], pgc_tb[near_idx], pga1_tb[near_idx], 0);
	if (error) goto exit;


	error = orion_cal_wmr(modulator ,0x0048, 3, pd_ori);
	if (error) goto exit;

exit:
	return (error);
}

uint32_t orion_calDC (
	  IT9510INFO*	modulator, 
	  uint32_t		frequency,
      Bool		ofs
	
) {
	uint8_t	val;
	uint8_t	temp;
	uint32_t	error = 0;

	if(frequency>950000 || frequency<70000){

		goto orion_pd; 
	}
	// turn on clock from eagle, Michael 20140423
	val = 0x80;
	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xFB26, 1, &val);
	if (error) goto exit;



	error = orion_cal_setfreq(modulator, 8000, frequency);
	if (error) goto exit;

	error = orion_dc_scan(modulator, ofs);
	if (error) goto exit;


	//----------------
orion_pd:
	temp = 0xFE;
	error = orion_cal_wmr(modulator ,0x0048, 1, &temp);
	if (error) goto exit;
	temp = 0xDF;
	error = orion_cal_wmr(modulator ,0x0049, 1, &temp);
	if (error) goto exit;
	temp = 0xFD;
	error = orion_cal_wmr(modulator ,0x004a, 1, &temp);
	if (error) goto exit;


	//----------------

	// turn off clock from eagle, Michael 20140423
	val = 0x00;
	error = IT9510_writeRegisters (modulator, Processor_OFDM, 0xFB26, 1, &val);
	if (error) goto exit;

exit:
	return(error);
}
