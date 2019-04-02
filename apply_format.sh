#!/usr/bin/env sh
find -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs clang-format-3.9 -style=file -i

