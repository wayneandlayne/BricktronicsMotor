#!/bin/bash
#
# A script to verify all example sketches using the Arduino
# command line --verify option.

# Update this variable to point to your Arduino binary
ARDUINO=~/installers/arduino*/arduino

for ino in `find examples/ | grep ino`; do
    echo "Verifying $ino"
    $ARDUINO --verify --board arduino:avr:uno $ino
done

