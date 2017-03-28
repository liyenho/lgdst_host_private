
 	#define USB_6612_SIZE_VAL						0xc
 	#define USB_6612_SIZE_LEN						sizeof(uint32_t)

 	typedef struct {
	 	uint8_t  addr;
	 	uint16_t data;
 	} dev_cfg;

 	/* target 482MHz if , 2392MHr RF, 1910MHz lo */
 	static dev_cfg regs_6612_ch0[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0470}, //read auto from diag reg val page
	 	{0x4,  0x0600}, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x0280}, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0x0aca}, //read auto from diag reg val page
	 	{0x23, 0x0000}, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0000}, //read auto from diag reg val page
	 	{0x40, 0x0070}, //read auto from diag reg val page
	 	{0x42, 0x000c}, //read auto from diag reg val page
	 	{0x43, 0x001b}, //read auto from diag reg val page
	 	{0x44, 0x001c}, //read auto from diag reg val page
	 	{0x45, 0x0000}, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x31  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2403.25Mhz RF, 1921.25MHz lo */
 	static dev_cfg regs_6612_ch1[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0032}, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2407.25Mhz RF, 1925.25MHz lo */
 	static dev_cfg regs_6612_ch2[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0xd2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2411.25Mhz RF, 1929.25MHz lo */
 	static dev_cfg regs_6612_ch3[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x172 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};


 	/* target 482MHz if , 2415.25Mhz RF, 1933.25MHz lo */
 	static dev_cfg regs_6612_ch4[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x212 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};
 	/* target 482MHz if , 2419.25Mhz RF, 1937.25MHz lo */
 	static dev_cfg regs_6612_ch5[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x2b2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2423.25Mhz RF, 1941.25MHz lo */
 	static dev_cfg regs_6612_ch6[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x352 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2427.25Mhz RF, 1945.25MHz lo */
 	static dev_cfg regs_6612_ch7[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x03f2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2431.25Mhz RF, 1949.25MHz lo */
 	static dev_cfg regs_6612_ch8[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0492 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2435.25Mhz RF, 1953.25MHz lo */
 	static dev_cfg regs_6612_ch9[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0532 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2439.25Mhz RF, 1957.25MHz lo */
 	static dev_cfg regs_6612_ch10[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x05d2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x32  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2443.25Mhz RF, 1961.25MHz lo */
 	static dev_cfg regs_6612_ch11[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0072 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2447.25Mhz RF, 1965.25MHz lo */
 	static dev_cfg regs_6612_ch12[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0112 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2451.25Mhz RF, 1969.25MHz lo */
 	static dev_cfg regs_6612_ch13[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x01b2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2455.25Mhz RF, 1973.25MHz lo */
 	static dev_cfg regs_6612_ch14[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0252 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2459.25Mhz RF, 1977.25MHz lo */
 	static dev_cfg regs_6612_ch15[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x02f2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2463.25Mhz RF, 1981.25MHz lo */
 	static dev_cfg regs_6612_ch16[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0392 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2467.25Mhz RF, 1985.25MHz lo */
 	static dev_cfg regs_6612_ch17[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0432 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2471.25Mhz RF, 1989.25MHz lo */
 	static dev_cfg regs_6612_ch18[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x04d2 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2475.25Mhz RF, 1993.25MHz lo */
 	static dev_cfg regs_6612_ch19[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0572 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x33  }, //read auto from diag reg val page
 	};

 	/* target 482MHz if , 2479.25Mhz RF, 1997.25MHz lo */
 	static dev_cfg regs_6612_ch20[] = {
	 	{0x0,  0x0},
	 	{0x1,  0xffff}, //read auto from diag reg val page
	 	{0x3,  0x0012 }, //read auto from diag reg val page
	 	{0x4,  0x600 }, //read auto from diag reg val page
	 	{0x10, 0x1e73}, //read auto from diag reg val page
	 	{0x20, 0x280 }, //read auto from diag reg val page
	 	{0x21, 0x1001}, //read auto from diag reg val page
	 	{0x22, 0xaca }, //read auto from diag reg val page
	 	{0x23, 0x0   }, //read auto from diag reg val page
	 	{0x30, 0x1c66}, //read auto from diag reg val page
	 	{0x31, 0x0   }, //read auto from diag reg val page
	 	{0x40, 0x70  }, //read auto from diag reg val page
	 	{0x42, 0xc   }, //read auto from diag reg val page
	 	{0x43, 0x1b  }, //read auto from diag reg val page
	 	{0x44, 0x1c  }, //read auto from diag reg val page
	 	{0x45, 0x0   }, //read auto from diag reg val page
	 	{0x46, 0x23  }, //read auto from diag reg val page
	 	{0x49, 0x3c  }, //read auto from diag reg val page
	 	{0x50, 0x488 }, //read manually from engineering page
	 	{0x51, 0x0   }, //read manually from engineering page
	 	{0x60, 0x0   }, //read auto from diag reg val page
	 	{0x7c, 0x1e23}, //read auto from diag reg val page
	 	{0x7d, 0x4001}, //read auto from diag reg val page
	 	{0x2,  0x34  }, //read auto from diag reg val page
 	};

 	#define ARRAY_SIZE(ary_i) ((ary_i == 0)? sizeof(regs_6612_ch0)/sizeof(dev_cfg) : \
 	(ary_i == 1)? sizeof(regs_6612_ch1)/sizeof(dev_cfg) : \
 	(ary_i == 2)? sizeof(regs_6612_ch2)/sizeof(dev_cfg) : \
 	(ary_i == 3)? sizeof(regs_6612_ch3)/sizeof(dev_cfg) : \
 	(ary_i == 4)? sizeof(regs_6612_ch4)/sizeof(dev_cfg) : \
 	(ary_i == 5)? sizeof(regs_6612_ch5)/sizeof(dev_cfg) : \
 	(ary_i == 6)? sizeof(regs_6612_ch6)/sizeof(dev_cfg) : \
 	(ary_i == 7)? sizeof(regs_6612_ch7)/sizeof(dev_cfg) : \
 	(ary_i == 8)? sizeof(regs_6612_ch8)/sizeof(dev_cfg) : \
 	(ary_i == 9)? sizeof(regs_6612_ch9)/sizeof(dev_cfg) : \
 	(ary_i ==10)? sizeof(regs_6612_ch10)/sizeof(dev_cfg) : \
 	(ary_i ==11)? sizeof(regs_6612_ch11)/sizeof(dev_cfg) : \
 	(ary_i ==12)? sizeof(regs_6612_ch12)/sizeof(dev_cfg) : \
 	(ary_i ==13)? sizeof(regs_6612_ch13)/sizeof(dev_cfg) : \
 	(ary_i ==14)? sizeof(regs_6612_ch14)/sizeof(dev_cfg) : \
 	(ary_i ==15)? sizeof(regs_6612_ch15)/sizeof(dev_cfg) : \
 	(ary_i ==16)? sizeof(regs_6612_ch16)/sizeof(dev_cfg) : \
 	(ary_i ==17)? sizeof(regs_6612_ch17)/sizeof(dev_cfg) : \
 	(ary_i ==18)? sizeof(regs_6612_ch18)/sizeof(dev_cfg) : \
 	(ary_i ==19)? sizeof(regs_6612_ch19)/sizeof(dev_cfg) : \
 	(ary_i ==20)? sizeof(regs_6612_ch20)/sizeof(dev_cfg) : \
 	0)

 	#define GET_ARRAY(ary_i) ((ary_i == 0)? &regs_6612_ch0: \
 	(ary_i == 1)? &regs_6612_ch1: \
 	(ary_i == 2)? &regs_6612_ch2: \
 	(ary_i == 3)? &regs_6612_ch3: \
 	(ary_i == 4)? &regs_6612_ch4: \
 	(ary_i == 5)? &regs_6612_ch5: \
 	(ary_i == 6)? &regs_6612_ch6: \
 	(ary_i == 7)? &regs_6612_ch7: \
 	(ary_i == 8)? &regs_6612_ch8: \
 	(ary_i == 9)? &regs_6612_ch9: \
 	(ary_i ==10)? &regs_6612_ch10: \
 	(ary_i ==11)? &regs_6612_ch11: \
 	(ary_i ==12)? &regs_6612_ch12: \
 	(ary_i ==13)? &regs_6612_ch13: \
 	(ary_i ==14)? &regs_6612_ch14: \
 	(ary_i ==15)? &regs_6612_ch15: \
 	(ary_i ==16)? &regs_6612_ch16: \
 	(ary_i ==17)? &regs_6612_ch17: \
 	(ary_i ==18)? &regs_6612_ch18: \
 	(ary_i ==19)? &regs_6612_ch19: \
 	(ary_i ==20)? &regs_6612_ch20: \
 	NULL)

