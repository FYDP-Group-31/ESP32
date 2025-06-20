#!/bin/bash

git submodule init
git submodule update

python3 -m pip install --upgrade certifi
python3 -m pip install setuptools

cd esp-idf
./install.sh esp32
source ./export.sh