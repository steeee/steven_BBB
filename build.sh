#!/bin/bash
clear
echo "Buildinging capture"
g++ -O2 -Wall `pkg-config --cflags --libs libv4l2` capture.c `pkg-config --libs opencv`-o capture
echo "OK!"
