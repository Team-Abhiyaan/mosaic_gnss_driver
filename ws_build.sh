#!/usr/bin/env bash
# build project

function ws_build () {
    cmake -H. -Bbuild && \
    cmake --build build -- -j8 && \
    echo -e "\n\033[1;33m Workspace build complete \033[0m\n"
}

function gendoc () {
    if [[ -f mosaic.dconfig ]]
    then
        doxygen mosaic.dconfig && \
        cd docs/latex
        make && \
        echo -e "\n\033[1;33m Generated docs \033[0m\n"
    else
        echo -e "\n\033[1;33m config file not found! Skipping docs auto-generation \033[0m\n"
    fi
}

function cleanup () {
    if [[ -d bin ]]; then rm -rf bin; fi
    if [[ -d build ]]; then rm -rf build; fi
    if [[ -d docs ]]; then rm -rf docs; fi
    
    echo -e "\n\033[1;33m Workspace cleaned \033[0m\n"
}

if [[ $1 = "gendoc" ]]
then
    gendoc
elif [[ $1 = "clean" ]]
then
    cleanup
else
    ws_build
fi
