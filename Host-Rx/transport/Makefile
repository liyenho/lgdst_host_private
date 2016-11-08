#Compiler definitions
CC=gcc

#C flag definitions
CFLAGS 	  = -O3 -Wfatal-errors -std=gnu99 -I/usr/include/ -I/usr/local/include/libusb-1.0
#LDFLAGS		= -L/usr/lib -L/usr/local/lib -lusb-1.0 -lpthread -lrt -lm -lX11 -lXi -lXmu -lglut -lGL -lGLU
#LDFLAGS		= -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm -lX11 -lXi -lglut -lGL -lGLU
LDFLAGS		= -L/usr/local/lib -L/usr/lib -lusb-1.0 -lpthread -lrt -lm #-lX11 -lglut -lGL -lGLU
# makefile changes necessary to generate .so file

ifneq "$(W)" ""
	CFLAGS 	  += -Dres_width=\"$(W)\"
endif
ifneq "$(H)" ""
	CFLAGS 	  += -Dres_height=\"$(H)\"
endif
ifeq "$(MODE)" "V"
	CFLAGS 	  += -D_enable_video_
endif
ifeq "$(MODE)" "A"
	CFLAGS			+= -D_enable_audio_
endif
ifeq "$(MODE)" "AV"
	CFLAGS			+= -D_enable_audio_ -D_enable_video_
endif
ifeq "$(ISO)"	"1"
	CFLAGS			+= -D_ISO_XFER_=1
endif

ifeq "$(SND)"	"1"
	CFLAGS			+= -DSND
endif
ifeq "$(REC)"	"1"
	CFLAGS			+= -DREC
endif

#Library and executable names
RECORD = usb_record
	SRC_REC		= ${RECORD}.c
PLAY = usb_play
	SRC_PLY		= ${PLAY}.c
EXP = example
	SRC_EXP		= ${EXP}.c
TEST = usb_test
	SRC_TEST		= ${TEST}.c timer.c # timing measure, liyenho
ACCESS = lgdst
	SRC_ACCESS	= fpga_access.c timer.c

${RECORD}: ${SRC_REC}
	${CC} -o $@ $(CFLAGS) ${SRC_REC} $(LDFLAGS)

${PLAY}: ${SRC_PLY}
	${CC} -o $@ $(CFLAGS) ${SRC_PLY} $(LDFLAGS)

${EXP}: ${SRC_EXP}
	${CC} -o $@ $(CFLAGS) ${SRC_EXP} $(LDFLAGS)

${TEST}: ${SRC_TEST}
	${CC} -o $@ $(CFLAGS) ${SRC_TEST} $(LDFLAGS)
ifeq "$(SND)"	"1"
		mv ${TEST} usb_tx
else
ifeq "$(REC)"	"1"
			mv ${TEST} usb_rx
endif
endif

usb_rx: usb_rx.c timer.c
	${CC} -o $@ $(CFLAGS) $@.c timer.c  $(LDFLAGS)

${ACCESS}: ${SRC_ACCESS}
	${CC} -o $@ $(CFLAGS) ${SRC_ACCESS} $(LDFLAGS)

lgdst_rx: lgdst_rx.c
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DREC -fPIC $@.c $(LDFLAGS)
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

access_rx:	${SRC_ACCESS}
	${CC} -shared -o lib$@.so.1.0 $(CFLAGS) -DLIB -fPIC ${SRC_ACCESS} $(LDFLAGS)
	/bin/rm -f lib$@.so
	/bin/ln -s lib$@.so.1.0 lib$@.so

clean:
	/bin/rm -f ${RECORD}
	/bin/rm -f ${PLAY}
	/bin/rm -f ${EXP}
	/bin/rm -f ${TEST}
	/bin/rm -f ${ACCESS}
	/bin/rm -f usb_rx
	/bin/rm -f *lgdst_rx.so*
	/bin/rm -f *access_rx.so*
	/bin/rm -f *.o

