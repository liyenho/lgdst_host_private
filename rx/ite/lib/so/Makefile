#Compiler definitions
CC=gcc


#C flag definitions
CFLAGS 	  = -O3  -DLIB -std=gnu99  -I/usr/include/ -I/usr/local/include/libusb-1.0
LDFLAGS	= -L./ -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm
CUR_DIR = $(shell pwd)

ifeq "$(VFC)"	"1"
	CFLAGS		+= -I/usr/local/include/libavcodec -I/usr/local/include/libavformat -I/usr/local/include/libavutil -DVIDEO_STATS
	LDFLAGS	+= -lavformat -lavcodec -lavutil -lswresample -lm
endif

# makefile changes necessary to generate .so file
SRC_LGDST = cmd.c it9130.c omega.c standard.c tuner.c user.c usb_rx.c platform_it9137.c timer.c
o.usb_rx = $(SRC_LGDST:.c=.o)
usb_rx: $(o.usb_rx)
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -fPIC ${SRC_LGDST} $(LDFLAGS)
	/bin/touch $@	# create dummy target
	cd /usr/lib; sudo /bin/ln -fs "$(CUR_DIR)/lib$@.so.1.0" lib$@.so
	cd "$(CUR_DIR)"

SRC_ACCESS= fpga_access.c timer.c
o.acs_rx = $(SRC_ACCESS:.c=.o)
access_rx: $(o.acs_rx)
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -fPIC ${SRC_ACCESS} $(LDFLAGS)
	/bin/touch $@	# create dummy target
	cd /usr/lib; sudo /bin/ln -fs "$(CUR_DIR)/lib$@.so.1.0" lib$@.so
	cd "$(CUR_DIR)"

SRC_DEMO= demo_rx.c timer.c
o.demo = $(SRC_DEMO:.c=.o)
demo_rx: $(o.demo)
	${CC} -o $@ $(CFLAGS) ${SRC_DEMO}  $(LDFLAGS) -lusb_rx -laccess_rx

all: usb_rx access_rx demo_rx

#.c.o:
#	$(CC) $(CFLAGS) -c $< -o $@

clean:
	/bin/rm -f *usb_rx.so*
	/bin/rm -f *access_*rx.so*
	/bin/rm -f demo_rx
	/bin/rm -f *.o
	# remove dummy targets
	/bin/rm -f usb_rx
	/bin/rm -f access_rx


