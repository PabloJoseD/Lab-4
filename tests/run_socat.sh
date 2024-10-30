#!/bin/sh
sudo socat PTY,link=/dev/ttyACM0,raw,echo=0 PTY,link=/dev/ttyACM1,raw,echo=0

# sudo chmod 777 /dev/pts/*