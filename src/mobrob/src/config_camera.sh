#!/bin/bash
while :
do
    v4l2-ctl -c exposure_absolute=80
    sleep 5
done
