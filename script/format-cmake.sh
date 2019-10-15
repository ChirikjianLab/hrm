#!/bin/bash

current_dir=$(git rev-parse --show-toplevel)
find $current_dir -iname CMakeLists.txt -o -iname *.cmake \
    | xargs cmake-format -i
