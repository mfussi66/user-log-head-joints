#!/bin/bash

# Constants
ROLLFILE="roll_output.log"
PITCHFILE="pitch_output.log"
DATETIME_FOLDER=$(date +"%Y%m%d%H%M")
CURRENT_TASK=$1
NEWDIR=${DATETIME_FOLDER}_${CURRENT_TASK}

# Check if log files exist
if [ ! -f "$ROLLFILE" ]; then
    echo "$ROLLFILE not found."
    exit 1
fi

if [ ! -f "$PITCHFILE" ]; then
    echo "$PITCHFILE not found."
    exit 2
fi

# Create directory that is containing .log files
mkdir "${NEWDIR}"
mv $ROLLFILE $PITCHFILE $NEWDIR
echo "Done."

