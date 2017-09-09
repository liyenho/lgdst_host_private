#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define UDPOUT_FILE_TS   	"cap160910_pid0x100.ts"
#define UDPOUT_FILE_TS_D  "cap160910_pid0x100_dual.ts"

#define ONE_SEC_WORTHY	251920
 unsigned char ts_buff[ONE_SEC_WORTHY];
 const int ts_size = sizeof(ts_buff);

int main(void) {
	FILE *out, *in;
	out  = fopen(UDPOUT_FILE_TS_D, "w");
	in = fopen(UDPOUT_FILE_TS, "r");
	if (!out || !in) {
		puts("failed to access TS files");
		if (out) fclose(out);
		if (in) fclose(in);
		exit (-1);
	}
	unsigned char tp_buff[188], *pb = ts_buff;
	int rd, size= 0, sec_w= 0;
	while(1) {
		rd = fread(tp_buff,1,188,in);
		if (188 != rd) {
			fclose(out);
			fclose(in);
			return;
		}
		fwrite(tp_buff, 188, 1, out);
		if (sec_w)
			fwrite(pb+size, 188, 1, out);
		memcpy(pb+size, tp_buff, 188);
		size += rd;
		if (size>=ts_size) {
			sec_w = 1;
			size = 0;
		}
	}
	return;
}
