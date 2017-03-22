
 	typedef struct {
	 	uint8_t  addr;
	 	uint16_t data;
 	} dev_cfg;

 	/* ??? check all listed registers here ??? */
 	static dev_cfg regs_2072_ch0[] = {
	 	{0x0,  0xFFFA},	//reg_LF
	 	{0x1,  0x4064},		//reg_XO
	 	{0x2,  0x9055},		//reg_CAL_TIME
	 	{0x3,  0x2D02},		//reg_VCO_CTRL
	 	{0x4,  0xB0BF},	//reg_CT_CAL1
	 	{0x5,  0xB0BF},	//reg_CT_CAL2
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
	 	{0x10, 0xC4E4},	//reg_P2_FREQ2
	 	{0x11, 0x4E00},	//reg_P2_FREQ3
	 	{0x12, 0x2A80},	//reg_FN_CTRL
	 	{0x13, 0x0000},		//reg_EXT_MOD
	 	{0x14, 0x0000},		//reg_FMOD
	 	{0x15, 0x0000},		//reg_SDI_CTRL
	 	{0x16, 0x70C1},  	//reg_GPO ???
	 	{0x17, 0x0000},		//reg_T_VCO
	 	{0x18, 0x0283},		//reg_IQMOD1
	 	{0x19, 0xF00F},	//reg_IQMOD2
	 	{0x1A, 0x0000},	//reg_IQMOD3
	 	{0x1B, 0x000F},	//reg_IQMOD4
	 	{0x1C, 0xC002},	//reg_T_CTRL
	 	{0x1D, 0x0001},	//reg_DEV_CTRL
	 	{0x1E, 0x0001},	//reg_TEST
 	};

 	#define ARRAY_SIZE(ary_i) ((ary_i == 0)? sizeof(regs_2072_ch0)/sizeof(dev_cfg) : \
 	/*(ary_i == 1)? sizeof(regs_2072_ch1)/sizeof(dev_cfg) : \
 	(ary_i == 2)? sizeof(regs_2072_ch2)/sizeof(dev_cfg) : \
 	(ary_i == 3)? sizeof(regs_2072_ch3)/sizeof(dev_cfg) : \
 	(ary_i == 4)? sizeof(regs_2072_ch4)/sizeof(dev_cfg) : \
 	(ary_i == 5)? sizeof(regs_2072_ch5)/sizeof(dev_cfg) : \
 	(ary_i == 6)? sizeof(regs_2072_ch6)/sizeof(dev_cfg) : \
 	(ary_i == 7)? sizeof(regs_2072_ch7)/sizeof(dev_cfg) : \
 	(ary_i == 8)? sizeof(regs_2072_ch8)/sizeof(dev_cfg) : \
 	(ary_i == 9)? sizeof(regs_2072_ch9)/sizeof(dev_cfg) : \
 	(ary_i ==10)? sizeof(regs_2072_ch10)/sizeof(dev_cfg) : \
 	(ary_i ==11)? sizeof(regs_2072_ch11)/sizeof(dev_cfg) : \
 	(ary_i ==12)? sizeof(regs_2072_ch12)/sizeof(dev_cfg) : \
 	(ary_i ==13)? sizeof(regs_2072_ch13)/sizeof(dev_cfg) : \
 	(ary_i ==14)? sizeof(regs_2072_ch14)/sizeof(dev_cfg) : \
 	(ary_i ==15)? sizeof(regs_2072_ch15)/sizeof(dev_cfg) : \
 	(ary_i ==16)? sizeof(regs_2072_ch16)/sizeof(dev_cfg) : \
 	(ary_i ==17)? sizeof(regs_2072_ch17)/sizeof(dev_cfg) : \
 	(ary_i ==18)? sizeof(regs_2072_ch18)/sizeof(dev_cfg) : \
 	(ary_i ==19)? sizeof(regs_2072_ch19)/sizeof(dev_cfg) : \
 	(ary_i ==20)? sizeof(regs_2072_ch20)/sizeof(dev_cfg) :*/ \
 	0)

 	#define GET_ARRAY(ary_i) ((ary_i == 0)? &regs_2072_ch0: \
 	/*(ary_i == 1)? &regs_2072_ch1: \
 	(ary_i == 2)? &regs_2072_ch2: \
 	(ary_i == 3)? &regs_2072_ch3: \
 	(ary_i == 4)? &regs_2072_ch4: \
 	(ary_i == 5)? &regs_2072_ch5: \
 	(ary_i == 6)? &regs_2072_ch6: \
 	(ary_i == 7)? &regs_2072_ch7: \
 	(ary_i == 8)? &regs_2072_ch8: \
 	(ary_i == 9)? &regs_2072_ch9: \
 	(ary_i ==10)? &regs_2072_ch10: \
 	(ary_i ==11)? &regs_2072_ch11: \
 	(ary_i ==12)? &regs_2072_ch12: \
 	(ary_i ==13)? &regs_2072_ch13: \
 	(ary_i ==14)? &regs_2072_ch14: \
 	(ary_i ==15)? &regs_2072_ch15: \
 	(ary_i ==16)? &regs_2072_ch16: \
 	(ary_i ==17)? &regs_2072_ch17: \
 	(ary_i ==18)? &regs_2072_ch18: \
 	(ary_i ==19)? &regs_2072_ch19: \
 	(ary_i ==20)? &regs_2072_ch20:*/ \
 	NULL)

