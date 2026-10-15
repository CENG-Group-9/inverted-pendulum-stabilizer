#!/bin/bash
# Generate build files in a 'build' folder, then compile
cmake -S . -B build
cmake --build build
