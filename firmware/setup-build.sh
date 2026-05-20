#!/bin/sh
# Bootstrap MPLAB X .generated_files placeholders so `make CONF=default`
# works on a fresh clone.
#
# Normally MPLAB X recreates these flag-files automatically on Project
# Open. Without that step the CLI build fails with:
#   "*** No rule to make target '.generated_files/flags/default/<hash>'"
# because Makefile-default.mk lists each flag-file as an explicit
# dependency.
#
# Safe to re-run; only touches missing files, never overwrites content.

set -eu
cd "$(dirname "$0")"

if [ ! -f nbproject/Makefile-default.mk ]; then
    echo "error: nbproject/Makefile-default.mk not found — run from firmware/" >&2
    exit 1
fi

mkdir -p .generated_files/flags/default

count=0
grep -oE '\.generated_files/flags/default/[a-zA-Z0-9]+' \
        nbproject/Makefile-default.mk \
    | sort -u \
    | while read -r p; do
        if [ ! -f "$p" ]; then
            touch "$p"
            count=$((count + 1))
        fi
        echo "$p"
    done | wc -l | xargs -I {} echo "Ensured {} flag-file placeholders exist."
