#Compiler definitions
CC=gcc

#C flag definitions
CFLAGS 	  = -O3  -std=gnu99  -I/usr/include/ -I/usr/local/include/libusb-1.0
LDFLAGS		= -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm 
# makefile changes necessary to generate .so file


#Library and executable names
TARGET = usb_tx
OBJS = orion_cal.o timer.o tuner.o it9510.o user.o usb_tx.o platform_it9517.o
ACCESS = lgdst
	SRC_ACCESS	= fpga_access.c timer.c

all:$(TARGET)

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@
$(TARGET):$(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

${ACCESS}: ${SRC_ACCESS}
	${CC} -o $@ $(CFLAGS) ${SRC_ACCESS} $(LDFLAGS)

clean:
	/bin/rm -f ${ACCESS}
	/bin/rm -f usb_tx
	/bin/rm -f *.o
-include $(wildcard *.d) dummy


