#!/bin/bash

echo "╔══╣ Install: coqui_tts_ros (STARTING) ╠══╗"

sudo apt-get update -y 
sudo apt install alsa-base

python3 -m pip install -U pip
python3 -m pip install \
    requests \
    wave

echo "╚══╣ Install: coqui_tts_ros (FINISHED) ╠══╝"