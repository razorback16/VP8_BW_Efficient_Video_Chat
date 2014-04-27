CC=g++
WLEVEL= -Wall -Wdisabled-optimization -Wpointer-arith -Wtype-limits -Wcast-qual -Wvla -Wuninitialized -Wunused-variable -Wunused-but-set-variable -Wno-unused-function
INC=-I/home/subhagato/Codes/libvpx/vp8 -I/home/subhagato/Codes/libvpx/vp8 -I/home/subhagato/Codes/libvpx/vp9 -I/home/subhagato/Codes/libvpx/vp9 -I. -I"/home/subhagato/Codes/libvpx"
CFLAGS=-c $(WLEVEL) -march=native -O3
LDFLAGS= `pkg-config opencv --cflags --libs` -lvpx -lm -lpthread
SOURCES= video_transmit.cpp
LIBLDIR =   ../libvpx

all:  video_transmit video_receive

rgb_to_yuv420.o: rgb_to_yuv420.cpp rgb_to_yuv420.h
	$(CC) $(CFLAGS) rgb_to_yuv420.cpp

video_transmit.o: video_transmit.cpp
	$(CC) $(CFLAGS) video_transmit.cpp

video_receive.o: video_receive.cpp
	$(CC) $(CFLAGS) video_receive.cpp

video_transmit: video_transmit.o rgb_to_yuv420.o
	$(CC) -o $@ video_transmit.o rgb_to_yuv420.o $(LIBLDIR)/ivfenc.c.o $(LIBLDIR)/tools_common.c.o $(LIBLDIR)/video_writer.c.o $(LDFLAGS)

video_receive: video_receive.o rgb_to_yuv420.o
	$(CC) -o $@ video_receive.o rgb_to_yuv420.o $(LIBLDIR)/ivfdec.c.o $(LIBLDIR)/tools_common.c.o $(LIBLDIR)/video_reader.c.o $(LDFLAGS)

clean:
	rm -rf *.o video_transmit video_receive *~ *.bak