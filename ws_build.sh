#!/usr/bin/env bash

#########################################################################################
# GLOBALS
#########################################################################################
# Path to this script
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Configuration file for Doxygen
DOXYGEN_CONFIG_FILE="mosaic.dconfig"

# All these directories will be deleted permanently when clean in invoked
DIRS_TO_TRASH=(
    docs
)

#########################################################################################
# FUNCTIONS
#########################################################################################

function ws_build () {
    echo ${SCRIPTPATH}
    cd ${SCRIPTPATH}/../..
    catkin_make
    echo -e "\n\033[1;33m Workspace build complete \033[0m\n"
}

function testing () {
    rostopic list > /dev/null 2>&1
    
    if [[ $? != 0 ]]
    then

        echo "Oops! Master is offline"
        exit 1
        
    else
        
        cd ${SCRIPTPATH}/../..
        catkin_make run_tests
        
    fi
}

function gendoc () {
    
    if [[ -f $DOXYGEN_CONFIG_FILE ]]
    then
        cd ${SCRIPTPATH}
        doxygen ${DOXYGEN_CONFIG_FILE} && \
        cd docs/latex
        make && \
        echo -e "\n\033[1;33m Generated docs \033[0m\n"
        
    else
        
        echo -e "\n\033[1;33m config file not found! Skipping docs auto-generation \033[0m\n"
        
    fi
}

function cleanup () {
    cd ${SCRIPTPATH}
    
    for dir in "${DIRS_TO_TRASH[@]}"; do
        if [[ -d $dir ]]
        then
            
            rm -rf $dir
            
        fi
    done
    
    echo -e "\n\033[1;33m Workspace cleaned \033[0m\n"
}

#########################################################################################
# ENTRY POINT
#########################################################################################

if [[ $1 = "gendoc" ]]
then
    gendoc
elif [[ $1 = "clean" ]]
then
    cleanup
elif [[ $1 = "testing" ]]
then
    testing
else
    ws_build
fi
