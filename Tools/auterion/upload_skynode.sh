#!/usr/bin/env bash

DIR="$(dirname $(readlink -f $0))"
PX4_BINARY_FILE="$1"
DEFAULT_AUTOPILOT_HOST=10.41.0.1
DEFAULT_AUTOPILOT_PORT=22

for i in "$@"
do
    case $i in
        --default-ip=*)
        DEFAULT_AUTOPILOT_HOST="${i#*=}"
        ;;
        --default-port=*)
        DEFAULT_AUTOPILOT_PORT="${i#*=}"
        ;;
        *)
            # unknown option
        ;;
    esac
done

# allow these to be overridden
[ -z "$AUTOPILOT_HOST" ] && AUTOPILOT_HOST=$DEFAULT_AUTOPILOT_HOST
[ -z "$AUTOPILOT_PORT" ] && AUTOPILOT_PORT=$DEFAULT_AUTOPILOT_PORT

echo "Uploading to $AUTOPILOT_HOST..."

"$DIR"/remote_update_fmu.sh -f "$PX4_BINARY_FILE" -d "$AUTOPILOT_HOST" -p $AUTOPILOT_PORT

exit 0

