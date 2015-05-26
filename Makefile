
SRC = main.c camera.c

INC = -I$(INC_DIR)
SHARE_LIB = -lopencv_core    \
            -lopencv_highgui \

CFLAGS = -g -Wfatal-errors -Wl,--allow-multiple-definition -Wall  `pkg-config --cflags --libs opencv-2.4.9` $(INC) $(SHARE_LIB)

CPP = g++

LOCAL_TARGET = main

.PHONY: all clean

all: $(LOCAL_TARGET)

$(LOCAL_TARGET): main.o camera.o
	g++ -o $@ $^ $(CFLAGS)

main.o: main.c 
	g++ -c $<


camera.o:camera.c
	g++ -c $<

clean:
	rm -f $(LOCAL_TARGET) *.o
