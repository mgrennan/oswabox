#!/bin/bash

# Create an auto-incrementing build number.

echo "#define Version \"0.0.6\"" > version.h
echo "#define BuildDate \"`date`\"" >> version.h
