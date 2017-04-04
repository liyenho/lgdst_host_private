#ifndef __RF2072_H__
#define __RF2072_H__


typedef struct {
	 uint8_t  addr;
	 uint16_t data;
 } dev_cfg;

 /* target 482MHz if , 2392MHr RF, 1910MHz lo */
static dev_cfg regs_2072_ch0[] = {
	 	{0x0,  0xFFFA},		//reg_LF
	 	{0x1,  0x4064},		//reg_XO
	 	{0x2,  0x9055},		//reg_CAL_TIME	
	 	{0x3,  0x2D02},		//reg_VCO_CTRL	 
	 	{0x4,  0xB0BF},		//reg_CT_CAL1	 
	 	{0x5,  0xB0BF},		//reg_CT_CAL2	
	 	{0x6,  0x0028},		//reg_PLL_CAL1	
	 	{0x7,  0x0028},		//reg_PLL_CAL2	 
	 	{0x8,  0xFC06},		//reg_VCO_AUTO	 
	 	{0x9,  0x8224},		//reg_PLL_CTRL	 
	 	{0xA,  0x0207},		//reg_PLL_BIAS	 
	 	{0xB,  0x4F00},		//reg_MIX_CONT	 
	 	{0xC,  0x2324},		//reg_P1_FREQ1	 
	 	{0xD,  0x6276},		//reg_P1_FREQ2	 
	 	{0xE,  0x2700},		//reg_P1_FREQ3	 
	 	{0xF,  0x1B96},		//reg_P2_FREQ1	 
	 	{0x10, 0xC4E4},		//reg_P2_FREQ2	
	 	{0x11, 0x4E00},		//reg_P2_FREQ3	
	 	{0x12, 0x2A80},		//reg_FN_CTRL	
	 	{0x13, 0x0000},		//reg_EXT_MOD	
	 	{0x14, 0x0000},		//reg_FMOD	 
	 	{0x15, 0x0000},		//reg_SDI_CTRL
	 	//{0x15, 0x8000},	//reg_SDI_CTRL
	 	{0x16, 0x70C1},  	//reg_GPO ???	 
	 	{0x17, 0x0000},		//reg_T_VCO	 
	 	{0x18, 0x0283},		//reg_IQMOD1	
	 	{0x19, 0xF00F},		//reg_IQMOD2	 
	 	{0x1A, 0x0000},		//reg_IQMOD3	 
	 	{0x1B, 0x000F},		//reg_IQMOD4	
	 	{0x1C, 0xC002},		//reg_T_CTRL	
	 	{0x1D, 0x0001},		//reg_DEV_CTRL	
	 	{0x1E, 0x0001},		//reg_TEST
	 		
 	};

 	
 #define ARRAY_SIZE(ary_i) ((ary_i == 0)? sizeof(regs_2072_ch0)/sizeof(dev_cfg) : 0)
 	
 #define GET_ARRAY(ary_i) ((ary_i == 0)? &regs_2072_ch0: NULL)
#endif
