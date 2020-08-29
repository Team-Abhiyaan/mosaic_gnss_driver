#!/usr/bin/env bash

#########################################################################################
# GLOBALS
#########################################################################################

# Configuration file for Doxygen
DOXYGEN_CONFIG_FILE="mosaic.dconfig"

# All these directories will be deleted permanently when clean in invoked
DIRS_TO_TRASH=(
    bin
    build
    docs
)

#########################################################################################
# FUNCTIONS
#########################################################################################

function ws_build () {
    
    cmake -H. -Bbuild && \
    cmake --build build -- -j8 && \
    echo -e "\n\033[1;33m Workspace build complete \033[0m\n"
    
}

function gendoc () {
    
    if [[ -f $DOXYGEN_CONFIG_FILE ]]
    then
        
        doxygen $DOXYGEN_CONFIG_FILE && \
        cd docs/latex
        make && \
        echo -e "\n\033[1;33m Generated docs \033[0m\n"
        
    else
        
        echo -e "\n\033[1;33m config file not found! Skipping docs auto-generation \033[0m\n"
        
    fi
}

function cleanup () {
    
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
else
    ws_build
fi
