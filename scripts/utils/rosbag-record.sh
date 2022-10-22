#!/bin/bash
# Script for easy recording of rosbags

# Set max duration of bag record -> float in 
MAX_DURATION=30

# Name for saving bag
NAME=testbag-$(date +%y%m%d-%H%M%S)

# Goal directory
GOAL_DIR=../data/local-bags

# Recording all topics listed after *record* in one line
rosbag record odom \
  --duration=$MAX_DURATION \
  --output-name=$NAME \

if [ -d "$PWD/$GOAL_DIR" ]; then
  echo "Move file to goal directory"
  mv $PWD/$NAME.bag $PWD/$GOAL_DIR
else 
  echo "Can't move file to goal directory"
fi