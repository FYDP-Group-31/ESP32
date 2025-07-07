#!/bin/bash

source ./esp-idf/export.sh

COMPONENT="$1"

cd $COMPONENT

idf.py build