#Compiler definitions
CC=gcc

#C flag definitions
CFLAGS 	  = -O3 -Wfatal-errors -std=gnu99 -I./ -I/usr/include/ -I/usr/local/include/libusb-1.0
LDFLAGS		= -L./ -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm #-lX11 -lglut -lGL -lGLU
# makefile changes necessary to generate .so file

ifeq "$(SND)"	"1"
	CFLAGS			+= -DSND
endif
ifeq "$(REC)"	"1"
	CFLAGS			+= -DREC
endif

#Library and executable names
SRC_LGDST = lgdst_rx.c
dep.lgdst: ${SRC_LGDST}
	@echo "Creating USB RX dependencies..."
	for i in ${SRC_LGDST}; do \
	($(CC) -MM $(CFLAGS) $$i >> tmp_depend); done
	mv tmp_depend dep.lgdst

o.lgdst = $(SRC_LGDST:.c=.o)
lgdst_rx: dep.lgdst $(o.lgdst)
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DREC -fPIC $@.c $(LDFLAGS)
	/bin/touch $@	# create dummy target
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

SRC_ACCESS_C= fpga_access_c.c timer.c
dep.acs_ctl: ${SRC_ACCESS_C}
	@echo "Creating CTL dependencies..."
	for i in ${SRC_ACCESS_C}; do \
	($(CC) -MM $(CFLAGS) $$i >> tmp_depend); done
	mv tmp_depend dep.acs_ctl

o.acs_ctl = $(SRC_ACCESS_C:.c=.o)
access_ctl_rx: dep.acs_ctl $(o.acs_ctl)
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DLIB -fPIC ${SRC_ACCESS_C} $(LDFLAGS)
	/bin/touch $@	# create dummy target
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

SRC_ACCESS_V= fpga_access_v.c timer.c
dep.acs_vid: ${SRC_ACCESS_V}
	@echo "Creating VID dependencies..."
	for i in ${SRC_ACCESS_C}; do \
	($(CC) -MM $(CFLAGS) $$i >> tmp_depend); done
	mv tmp_depend dep.acs_vid

o.acs_vid = $(SRC_ACCESS_V:.c=.o)
access_vid_rx: dep.acs_vid $(o.acs_vid)
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DLIB -fPIC ${SRC_ACCESS_V} $(LDFLAGS)
	/bin/touch $@	# create dummy target
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

SRC_DEMO= demo_rx.c timer.c
dep.demo: ${SRC_DEMO}
	@echo "Creating DEMO dependencies..."
	for i in ${SRC_DEMO}; do \
	($(CC) -MM $(CFLAGS) $$i >> tmp_depend); done
	mv tmp_depend dep.demo

o.demo = $(SRC_DEMO:.c=.o)
demo_rx: dep.demo $(o.demo)
	${CC} -o $@ $(CFLAGS) $@.c timer.c  $(LDFLAGS) -llgdst_rx -laccess_ctl_rx -laccess_vid_rx

clean:
	/bin/rm -f *lgdst_rx.so*
	/bin/rm -f *access_*rx.so*
	/bin/rm -f demo_rx
	/bin/rm -f dep.*
	/bin/rm -f *.o
	# remove dummy targets
	/bin/rm -f lgdst_rx
	/bin/rm -f access_ctl_rx
	/bin/rm -f access_vid_rx

-include dep.lgdst
-include dep.acs_ctl
-include dep.acs_vid
-include dep.demo
