#!/bin/bash

SCRIPT=$(realpath "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# Start timer
SECONDS=0

docker build -t nidhi/px4_1.16_ros2 -f ${SCRIPTPATH}/Dockerfile  ${SCRIPTPATH}/..

# Stop timer and show elapsed time
echo "Docker build completed in $SECONDS seconds."

