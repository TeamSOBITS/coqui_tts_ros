#!/bin/bash

echo "╔══╣ Install: coqui_tts_ros (STARTING) ╠══╗"

DIR="$(pwd)"
cd ..
git clone https://github.com/TeamSOBITS/coqui_tts
cd coqui_tts
bash install.sh
cd $DIR

python3 -m pip install -U pip
python3 -m pip install requests

echo "╚══╣ Install: coqui_tts_ros (FINISHED) ╠══╝"