#!/usr/bin/env bash

PI_USER="pi"
PI_HOST="132.206.126.229"
PI_PORT="2004"
REMOTE_DIR="~/accelerometer/ADXL345/data"
LOCAL_DIR="./data"

mkdir -p "$LOCAL_DIR"

LATEST_FILE=$(ssh -p ${PI_PORT} ${PI_USER}@${PI_HOST} "ls -t ${REMOTE_DIR}/accelerometer_data_*.csv | head -n 1")

scp -P ${PI_PORT} ${PI_USER}@${PI_HOST}:"$LATEST_FILE" "$LOCAL_DIR"
