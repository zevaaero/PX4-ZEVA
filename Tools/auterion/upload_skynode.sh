#!/usr/bin/env bash

DIR="$(dirname $(readlink -f $0))"
PX4_BINARY_FILE="$1"

# allow these to be overridden
[ -z "$AUTOPILOT_HOST" ] && AUTOPILOT_HOST=10.41.0.1
[ -z "$AUTOPILOT_PORT" ] && AUTOPILOT_PORT=44444

echo "Uploading to $AUTOPILOT_HOST..."

"$DIR"/remote_update_fmu.sh -f "$PX4_BINARY_FILE" -d "$AUTOPILOT_HOST" -p $AUTOPILOT_PORT

exit 0

