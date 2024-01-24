#!/bin/bash

# Save the file name as a variable called SOURCE_FILE
SOURCE_FILE=$1

rm build -rf
mkdir -p build
cd build

#Make the project using the passed in source file
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..