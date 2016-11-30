//////////////////////////////////////////////////////////////////////////////////
// quick scripts
//////////////////////////////////////////////////////////////////////////////////
"sh sm <ret>" will make all the libraries and demo code
"sh sln <ret>" will build links into the /usr/lib directory. (YOU MUST UDATE DIRECTORY PATH MANUALLY IN SCRIPT)
"sh sr <ret>" will run the demo code
//////////////////////////////////////////////////////////////////////////////////

Build procedures for liblgdst and libaccess
 	1. No more IPC requirement on either Tx or Rx host library implementation
	2. make clean; make lgdst_t(r)x access_ctl_t(r)x access_vid_t(r)x; at Tx/Rx transport folder respectively
	3. since we'll have to run usb lower level codes in preliviged mode, create file links in /usr/lib
		 cd /usr/lib;
		 sudo ln -s ${TX/RX-transport folder}/liblgdst_t(r)x.so.1.0 ./liblgdst_t(r)x.so
		 sudo ln -s ${TX/RX-transport folder}/libaccess_ctl_t(r)x.so.1.0 ./libaccess_ctl_t(r)x.so
		 sudo ln -s ${TX/RX-transport folder}/libaccess_vid_t(r)x.so.1.0 ./libaccess_vid_t(r)x.so

Build procedures for sample Tx/Rx codes using -llgdst_t(r)x and -laccess_ctl_t(r)x, -laccess_vid_t(r)x
	1. there are two kinds of sample codes which demostrate how to bring usb (tx/rx) core up for routine operation
		 and how to bring usb (tx/rx) core up for traditional 'lgdst' command line utility, now are unified inside
		 one executable
	2. make Makefile demo_t(rx) to build sample program for either Tx/Rx
	3. to enable routine operation, such like
		 	sudo ./demo_tx  0 127.0.0.1 5559								# run routine control and video radio links, all ips and ports are not used
		 	sudo ./demo_rx 0 127.0.0.1 127.0.0.1 5561			# any more but keep as being compatible with original interface
		 at the same command prompt, try to issue lgdst command such like
		 	lgdstc 0 tx ctune 0x40 or lgdstv 0 tx atten 0x3000
		 	lgdstv 0 rx pair-id 00 00 00 00 00 00 00 00 00 01 or lgdstv 0 rx vscan
		 cmd lgdstc and lgdstv are used to indicate which type of lgdst library, control or video, to pass into

TX access library usage (all numbers are in hex radix)
	lgdst 0 tx Vn														gives current NIOS HW/FW version
	lgdst 0 tx Vf															gives current FPGA HW version
	lgdst 0 tx Va															gives current Atmel FW version
	lgdst 0 tx Uf file												upgrade FPGA HW with given bin file
	lgdst 0 tx Uf0 														switch FPGA to app image
	lgdst 0 tx Ua file												upgrade Atmel FW with given bin file
	lgdst 0 tx ns															restart TS streaming activity
	lgdst 0 tx s																stop TS streaming activity
	lgdst 0 tx wbs adr blksize v0 v1 v2...		write block of values to FPGA registers address = adr with length = blksize, stored onto flash as well
	lgdst 0 tx ws adr v0											write value v0 to FPGA register address = adr, stored onto flash as well
	lgdst 0 tx wb adr v0 v1 v2...							write block of values to FPGA registers address = adr with length = blksize
	lgdst 0 tx w adr v0											write value v0 to FPGA register address = adr
	lgdst 0 tx rb adr blksize									read block of values from FPGA register address = adr with length = blksize
	lgdst 0 tx r adr 													read a value from FPGA register address = adr
	lgdst 0 tx pair-id 10-hex-digit					send PAIR ID to Atmel for Si4463, pair-id equals to 1 shall put atmel in pairing mode w/o setting actual ID
	lgdst 0 tx pair-locked 									query Si4463 locked status after piar-id command is issued
	lgdst 0 tx Cst														query Si4463 current operating mode whether if it is long range or short range, not active for now
	lgdst 0 tx MDst													query Si4463 current modem states
	lgdst 0 tx rfch chn											send desired video transimmsion channel #, chn
	lgdst 0 tx atten pwr_lvl									send desired video transmission power level, pwr_lvl
	lgdst 0 tx temp													query Si4463 current operating temperature, in degree C
	lgdst 0 tx ctune cap											send Si4463 desired cap value for frequency compensation
	lgdst 0 tx calib													put Atmel and Si4463 in calibration mode for operating frequency range
	lgdst 0 tx calib-qry 											query Atmel and Si4463 status of cap value calibration, it reports desired cap value for F compesnation and saved onto flash

RX access library usage (all numbers are in hex radix)
	lgdst 0 rx Vc															gives current CPLD HW version
	lgdst 0 rx Va														gives current Atmel FW version
	lgdst 0 rx Uc file												upgrade CPLD HW with given bin file
	lgdst 0 rx ns															restart TS streaming activity
	lgdst 0 rx s															stop TS streaming activity
	lgdst 0 rx wbs adr blksize v0 v1 v2...		write block of values to FPGA registers address = adr with length = blksize, stored onto flash as well
	lgdst 0 rx ws adr v0											write value v0 to FPGA register address = adr, stored onto flash as well
	lgdst 0 rx wb adr v0 v1 v2...							write block of values to FPGA registers address = adr with length = blksize
	lgdst 0 rx w adr v0											write value v0 to FPGA register address = adr
	lgdst 0 rx rb adr blksize									read block of values from FPGA register address = adr with length = blksize
	lgdst 0 rx r adr 													read a value from FPGA register address = adr
	lgdst 0 rx pair-id 10-hex-digit					send PAIR ID to Atmel for Si4463, pair-id equals to 1 shall put atmel in pairing mode w/o setting actual ID
	lgdst 0 rx pair-locked 									query Si4463 locked status after piar-id command is issued
	lgdst 0 rx locked												query Siano dual receiver on current locked status
	lgdst 0 rx retune IF-freq								retune Siano dual receiver with desired IF operating ferquency, IF-freq
	lgdst 0 rx recv														query Siano dual receiver on current reception status
	lgdst 0 rx sig														query Siano dual receiver on current reception status in short description mode
	lgdst 0 rx Cst														query Si4463 current operating mode whether if it is long range or short range, not active for now
	lgdst 0 rx MDst													query Si4463 current modem states
	lgdst 0 rx rfch chn											send desired video reception channel #, chn
	lgdst 0 rx temp													query Si4463 current operating temperature, in degree C
	lgdst 0 rx ctune cap											send Si4463 desired cap value for frequency compensation
	lgdst 0 rx calib													put Atmel and Si4463 in calibration mode for operating frequency range
	lgdst 0 rx calib-qry 											query Atmel and Si4463 status of cap value calibration, it reports desired cap value for F compesnation and saved onto flash



