#!/bin/bash

# Replace the tilde with the home directory
temp=$1
temp=${temp/\~/$HOME}

# Create directory if it doesn't exist
[ ! -d $temp ] && mkdir $temp  && echo "Created path for new Rtabmap database: $temp"