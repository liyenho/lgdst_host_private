#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void main(int argc, char **argv) {
	if (2 != argc) {
		printf("usage: %s dbg_ts_file\n", argv[0]);
		return;
	}
	FILE *fts = fopen(argv[1], "rb");
	fseek(fts, 0, SEEK_END);
	long fl = ftell(fts);
	if (fl % 188) {
		puts("invalid video TS file length");
		fclose(fts);
		return ;
	}
	fseek(fts, 0, SEEK_SET); // rewind
	uint8_t *vts = (uint8_t*)malloc(fl);
	if (0==vts) {
		puts("failed to allocate video TS buffer");
		fclose(fts);
		return ;
	}
	if (fl != fread(vts, 1, fl, fts)) {
		puts("failed to read video TS file");
		fclose(fts);
		free(vts);
		return;
	}
	fclose(fts);
	uint8_t *pb1 = vts;
	int pidvid = 0x40, err_cnt = 1;
	do {
		if ((0x80 & *(pb1+1)) && // check upon only video packet
			((pb1[1]&0x1f)==((pidvid>>8)&0x1f))  &&
	      ((pb1[2]&0xff)==((pidvid)   &0xff))) {
			printf("still found error, cnt = %d\n", err_cnt++);
		}
		pb1 = pb1 + 188;
		fl = fl - 188;
	} while(fl) ;
	free(vts);
	return;
}
