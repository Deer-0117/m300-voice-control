#!/bin/sh
set -eu

device="${1:-/dev/ttyACM0}"
settle_seconds="${2:-2}"

while [ ! -c "$device" ]; do
    sleep 1
done

# Allow the remaining USB interfaces to finish enumeration after ttyACM appears.
sleep "$settle_seconds"
