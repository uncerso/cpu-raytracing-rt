#!/usr/bin/env bash

if (($# == 2)); then
    ./target/release/cpu-raytracing-rt custom "$2" < "$1"
else
    ./target/release/cpu-raytracing-rt glTF "$@"
fi
