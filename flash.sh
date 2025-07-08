#!/bin/bash

COMPONENT="$1"
PORT="$2"

source ./esp-idf/export.sh

# Remove lock on serial port by killing process using it
PID=$(lsof "$PORT" 2>/dev/null | awk 'NR>1 {print $2; exit}')
if [ -n "$PID" ]; then
    kill -9 "$PID"
fi

cd $COMPONENT

idf.py -p $PORT -b 115200 flash monitor