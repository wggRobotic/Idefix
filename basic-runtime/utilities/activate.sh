#!/bin/bash

cd ~/Idefix || exit

python3 -m venv .venv --clear
source .venv/bin/activate

curl https://bootstrap.pypa.io/get-pip.py | python

python3 -m pip install --upgrade pip setuptools jmespath pyserial throttle inputs smbus RPi.GPIO pick
