#include <stdio.h>
#include <string.h>

#define true 	1
#define false	0

static int htoi(char s[])
{
    int hexdigit,i,inhex,n;
    i = 0;
    if( s[i] == '0')
    {
        ++i;
        if(s[i] == 'x' || s[i] == 'X')
            ++i;
    }

    n = 0;
    inhex = true;

    for(;inhex==true;++i)
    {
        if(s[i] >='0' && s[i] <='9')
            hexdigit= s[i] - '0';
        else if(s[i] >='a' && s[i] <='f')
            hexdigit= s[i] -'a' + 10;
        else if(s[i] >='A' && s[i] <='F')
            hexdigit= s[i] -'A' + 10;
        else
            inhex = false;

        if(inhex == true)
            n = 16 * n + hexdigit;
    }
    return n;
}

void main(int argc, char ** argv) {
	if (3 != argc) {
		printf("usage: %s hex-file bin-file", argv[0]);
		return ;
	}
	FILE *fi = fopen(argv[1], "rt"),
				 *fo = fopen(argv[2], "wb");
	if (!fi) {
		printf("failed to open %s\n", argv[1]);
		return ;
	}
	if (!fo) {
		printf("failed to open %s\n", argv[2]);
		fclose(fi);
		return ;
	}
	char sbuf[64]={0}, *pb;
	int n, len, ret = fscanf(fi, "%s", sbuf);
	while (1 == ret) {
		pb = &sbuf[1] ; // bypass ':'
		len = strlen(pb);
 //printf("len = %d\n", len);
 //printf("sbuf = %s\n", sbuf);
		if (len & 1) {
			puts("invalid string length, can't be odd #");
			break;
		}
		char hex[2+1] = {0};
		for (n=0; n<len/2; n++) {
			*(hex+0) = *pb++;
			*(hex+1) = *pb++;
			int val = htoi(hex);
			if (1 != fwrite(&val, 1, 1, fo)) {
				printf("failed to write %s!!!!!!!\n", argv[2]);
				break;
			}
		}
		memset(sbuf, 0, sizeof(sbuf));
		ret = fscanf(fi, "%s", sbuf);
 //printf("ret = %d\n", ret);
	}
	fclose(fi);
	fclose(fo);
}