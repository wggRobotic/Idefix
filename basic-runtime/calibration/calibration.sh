#!/bin/bash

cd ~/Idefix || exit
export PYTHONPATH=./basic-runtime


~/Idefix/.venv/bin/python3 basic-runtime/calibration/calibration/calibration.py
