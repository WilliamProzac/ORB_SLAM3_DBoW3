#!/bin/bash

# Default sequence
SEQUENCE="V1_01_easy"
TIMESTAMP_FILE="V101.txt"

# If a sequence is provided as an argument, use it
if [ "$1" != "" ]; then
    SEQUENCE=$1
    # Basic mapping from sequence folder name to timestamp file name
    # e.g., V1_01_easy -> V101.txt, MH_01_easy -> MH01.txt
    if [[ "$SEQUENCE" == *"V1_01"* ]]; then TIMESTAMP_FILE="V101.txt"; fi
    if [[ "$SEQUENCE" == *"V1_02"* ]]; then TIMESTAMP_FILE="V102.txt"; fi
    if [[ "$SEQUENCE" == *"V1_03"* ]]; then TIMESTAMP_FILE="V103.txt"; fi
    if [[ "$SEQUENCE" == *"V2_01"* ]]; then TIMESTAMP_FILE="V201.txt"; fi
    if [[ "$SEQUENCE" == *"V2_02"* ]]; then TIMESTAMP_FILE="V202.txt"; fi
    if [[ "$SEQUENCE" == *"V2_03"* ]]; then TIMESTAMP_FILE="V203.txt"; fi
    if [[ "$SEQUENCE" == *"MH_01"* ]]; then TIMESTAMP_FILE="MH01.txt"; fi
    if [[ "$SEQUENCE" == *"MH_02"* ]]; then TIMESTAMP_FILE="MH02.txt"; fi
    if [[ "$SEQUENCE" == *"MH_03"* ]]; then TIMESTAMP_FILE="MH03.txt"; fi
    if [[ "$SEQUENCE" == *"MH_04"* ]]; then TIMESTAMP_FILE="MH04.txt"; fi
    if [[ "$SEQUENCE" == *"MH_05"* ]]; then TIMESTAMP_FILE="MH05.txt"; fi
fi

DATASET_PATH="/home/ywl/dataset/vicon_room1/${SEQUENCE}"
TIMESTAMP_PATH="./Examples/Stereo-Inertial/EuRoC_TimeStamps/${TIMESTAMP_FILE}"

echo "================================================================"
echo "Testing ORB_SLAM3_DBoW3 (Map Sparsification) on EuRoC Dataset"
echo "Sequence: ${SEQUENCE}"
echo "Dataset Path: ${DATASET_PATH}"
echo "Timestamp File: ${TIMESTAMP_PATH}"
echo "================================================================"

# Execute the Stereo-Inertial example
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    ./Vocabulary/orbvoc.dbow3 \
    ./Examples/Stereo-Inertial/EuRoC.yaml \
    ${DATASET_PATH} \
    ${TIMESTAMP_PATH}

echo "Done."
