#!/bin/bash

echo "╔══╣ Install: coqui_tts_ros (STARTING) ╠══╗"

sudo apt-get update -y 
# sudo apt install alsa-base
sudo apt install ffmpeg

python3 -m pip install -U pip
python3 -m pip install \
    requests \
    wave \
    pygame

echo "╚══╣ Install: coqui_tts_ros (FINISHED) ╠══╝"