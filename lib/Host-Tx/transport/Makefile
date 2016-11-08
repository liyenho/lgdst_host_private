#Compiler definitions
CC=gcc

#C flag definitions
CFLAGS 	  = -O3 -Wfatal-errors -std=gnu99 -I./ -I/usr/include/ -I/usr/local/include/libusb-1.0
LDFLAGS		= -L./ -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm #-lX11 -lglut -lGL -lGLU

ifeq "$(SND)"	"1"
	CFLAGS			+= -DSND
endif
ifeq "$(REC)"	"1"
	CFLAGS			+= -DREC
endif

#Library and executable names
lgdst_tx: lgdst_tx.c
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DSND -fPIC $@.c  $(LDFLAGS)
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

SRC_ACCESS_C= fpga_access_c.c timer.c
access_ctl_tx:	${SRC_ACCESS_C}
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DLIB -fPIC ${SRC_ACCESS_C} $(LDFLAGS)
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

SRC_ACCESS_V= fpga_access_v.c timer.c
access_vid_tx:	${SRC_ACCESS_V}
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DLIB -fPIC ${SRC_ACCESS_V} $(LDFLAGS)
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

demo_tx: demo_tx.c timer.c
	${CC} -o $@ $(CFLAGS) $@.c timer.c  $(LDFLAGS) -llgdst_tx -laccess_ctl_tx -laccess_vid_tx

clean:
	/bin/rm -f *lgdst_tx.so*
	/bin/rm -f *access_*tx.so*
	/bin/rm -f demo_tx
	/bin/rm -f *.o

