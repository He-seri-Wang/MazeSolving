#!/usr/bin/env bash
set -euo pipefail

# Compile native tests with g++
CPP=g++
CXXFLAGS="-std=c++17 -Iinclude -Itest -I./src -O2"
SRC=test/test_ir_sensor.cpp
OUT=build/test_runner
mkdir -p build

echo "Compiling $SRC..."
$CPP $CXXFLAGS $SRC -o $OUT

echo "Running tests..."
$OUT
