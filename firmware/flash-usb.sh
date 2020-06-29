#!/bin/sh
#
# Flash hex files via remote control USB interface.
#

basedir="$(dirname "$0")"
[ "$(echo "$basedir" | cut -c1)" = '/' ] || basedir="$PWD/$basedir"

flashtool="$basedir/../remote/simplepwm"

exec "$flashtool" \
	--write-flash "$basedir/simplepwm.hex" \
	--write-eeprom "$basedir/simplepwm.eep.hex" \
	"$@"
