#!/usr/bin/env bash


while [ -z "$RAW" ]; do
    RAW=$(rostopic list | grep battery_state | sed 's|^[^/]*/\([^/]*\)/.*$|\1|')
done

echo $RAW

