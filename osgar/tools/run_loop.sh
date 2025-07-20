#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Error: <wait> <dtc>"
  exit 1
fi

# If script reaches here, exactly one parameter was provided
echo "Parameter is: $1, $2"

while true; do
  echo
  echo "Waiting for release of STOP button"
  sleep 1
  python -m osgar.record $1

  echo
  echo "Waiting for RORO25-back termination"
  sleep 1
  python -m osgar.record $2

done
