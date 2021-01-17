#!/usr/bin/env sh
find -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs clang-format-6.0 -style=file -i

