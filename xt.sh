#!/bin/bash

# Check if the script is already running in this shell
if [ ! -z "$IN_XEETAH_SHELL" ]; then
    echo "A Xeetah shell environment is already running."
    exit 0
fi

# Set a flag to indicate the script has been run
export IN_XEETAH_SHELL=1

# Setting some environment variables
export QUESTA_HOME="/home/jiefan/tools/questasim_2023.3"
export VIVADO_HOME="/home/jiefan/tools/Xilinx/2022.2/Vivado/2022.2"

export XEETAH_HOME="/home/jiefan/github/smartnic"
export XEETAH_GEN_PYBIND="${XEETAH_HOME}/py/xeetah/generated/pybind11"
export XEETAH_CPP_GEN="${XEETAH_HOME}/cpp/generated"
export PYTHONPATH="${PYTHONPATH}:/home/jiefan/github/smartnic/py:/home/jiefan/github/cocotb-bus/src"
export MTI_VCO_MODE=64 # questa setting
export SIM=questa # cocotb simulator
export COCOTB_REDUCED_LOG_FMT=1

PATH=${XEETAH_HOME}/py/xeetah/bin:$PATH
PATH=/home/jiefan/tools/mgls/bin:$PATH
PATH=${QUESTA_HOME}/questasim/bin:$PATH
PATH=${VIVADO_HOME}/bin:$PATH
export PATH

# Modify the PS1 variable to add <xt> at the end of the prompt
export ORIGINAL_PS1="$PS1"
export PS1="<xt>[\u@\h \W]\$"

trap 'history -a' EXIT

# Launch a new shell
exec bash
