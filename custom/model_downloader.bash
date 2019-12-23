#!/bin/bash
#
# Auterion Model Downloader
#
# (c) 2019 Auterion
#

if [ "$#" != 2 ]; then
    echo -e "usage: ./model_downloader.bash src_dir build_dir\n"
    return 1
fi

SRC_DIR=$1
BUILD_DIR=$2

# Check if model files are present
if test -d "$SRC_DIR/Tools/sitl_gazebo/models/almend1"; then
    echo "Auterion Simulation Models found, continuing.."
else
    # Not found, download and unzip
    echo "Auto-Downloading Auterion Simulation Models.."
    cd $SRC_DIR/Tools/sitl_gazebo/models
    wget https://au-models.s3.amazonaws.com/auterion-models.zip
    unzip auterion-models.zip
    rm -f auterion-models.zip
fi
