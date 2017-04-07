#!/bin/bash

./configure --disable-static --enable-shared --disable-debug --disable-yasm

if [ $? != 0 ] ; then
	exit $?
fi

make clean
make

if [ $? != 0 ] ; then
	exit $?
fi

sudo make install

if [ $? != 0 ] ; then
	exit $?
fi

sudo mv /usr/local/include/libavutil/time.h /usr/local/include/libavutil/av_time.h

cur_dir=`pwd`

cd /usr/lib

sudo rm -f libavcodec.so.56; sudo ln -s /usr/local/lib/libavcodec.so.56.44.100 libavcodec.so.56
sudo rm -f libavcodec.so; sudo ln -s /usr/local/lib/libavcodec.so.56.44.100 libavcodec.so

sudo rm -f libavdevice.so.56; sudo ln -s /usr/local/lib/libavdevice.so.56.4.100 libavdevice.so.56
sudo rm -f libavdevice.so; sudo ln -s /usr/local/lib/libavdevice.so.56.4.100 libavdevice.so

sudo rm -f libavfilter.so.5; sudo ln -s /usr/local/lib/libavfilter.so.5.17.100 libavfilter.so.5
sudo rm -f libavfilter.so; sudo ln -s /usr/local/lib/libavfilter.so.5.17.100 libavfilter.so

sudo rm -f libavformat.so.56; sudo ln -s /usr/local/lib/libavformat.so.56.38.100 libavformat.so.56
sudo rm -f libavformat.so; sudo ln -s /usr/local/lib/libavformat.so.56.38.100 libavformat.so

sudo rm -f libavutil.so.54; sudo ln -s /usr/local/lib/libavutil.so.54.27.100 libavutil.so.54
sudo rm -f libavutil.so; sudo ln -s /usr/local/lib/libavutil.so.54.27.100 libavutil.so

sudo rm -f libswresample.so.1; sudo ln -s /usr/local/lib/libswresample.so.1.2.100 libswresample.so.1
sudo rm -f libswresample.so; sudo ln -s /usr/local/lib/libswresample.so.1.2.100 libswresample.so

sudo rm -f libswscale.so.3; sudo ln -s /usr/local/lib/libswscale.so.3.1.101 libswscale.so.3
sudo rm -f libswscale.so; sudo ln -s /usr/local/lib/libswscale.so.3.1.101 libswscale.so

cd $cur_dir


