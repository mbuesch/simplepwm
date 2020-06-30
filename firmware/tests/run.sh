#!/bin/sh

basedir="$(dirname "$0")"
[ "$(echo "$basedir" | cut -c1)" = '/' ] || basedir="$PWD/$basedir"

cd "$basedir" || exit 1

for t in *.py; do
	echo "Running $t ..."
	python3 -m unittest "$t" || exit 1
done
rm -rf "$basedir"/test_*.bin "$basedir"/__pycache__
