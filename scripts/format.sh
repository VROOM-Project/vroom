#!/usr/bin/env bash

# Initially borrowed and adapted from
# https://github.com/Project-OSRM/osrm-backend/blob/master/scripts/format.sh

set -o errexit
set -o pipefail
set -o nounset

# Runs the Clang Formatter in parallel on the code base.
# Return codes:
#  - 1 there are files to be formatted or clang-format 18 is missing
#  - 0 everything looks fine

# Get CPU count
OS=$(uname)
NPROC=1
if [[ $OS = "Linux" ]] ; then
    NPROC=$(nproc)
elif [[ ${OS} = "Darwin" ]] ; then
    NPROC=$(sysctl -n hw.physicalcpu)
fi

# Discover clang-format
if type clang-format-18 2> /dev/null ; then
    CLANG_FORMAT=clang-format-18
elif type clang-format 2> /dev/null ; then
    # Clang format found, but need to check version
    CLANG_FORMAT=clang-format
    V=$(clang-format --version)
    if [[ $V != *18* ]] ; then
        echo "clang-format is not 18 (returned ${V})"
        exit 1
    fi
else
    echo "No appropriate clang-format found (expected clang-format-18, or clang-format)"
    exit 1
fi

find src libvroom_examples -type f -name '*.h' -o -name '*.cpp' \
  | xargs -I{} -P ${NPROC} ${CLANG_FORMAT} -i -style=file {}
