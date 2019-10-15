#!/bin/bash

current_dir=$(git rev-parse --show-toplevel)
find $current_dir/src -iname *.h -o -iname *.cpp \
    | xargs clang-format-6.0 -style=file -i -fallback-style=none
find $current_dir/test -iname *.h -o -iname *.cpp \
    | xargs clang-format-6.0 -style=file -i -fallback-style=none
