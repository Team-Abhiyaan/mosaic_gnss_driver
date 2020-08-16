#!/usr/bin/env bash
# build project

cmake -H. -Bbuild && \
cmake --build build -- -j8 && \
echo -e "\033[1;33mWorkspace build complete \033[0m"
