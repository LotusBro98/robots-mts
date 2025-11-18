#!/bin/bash

sudo apt-get update
sudo apt-get install ffmpeg libsm6 libxext6 python3-spidev -y

git config --global user.email "lotusbro98@gmail.com"
git config --global user.name "Lotus Bro"

python -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt