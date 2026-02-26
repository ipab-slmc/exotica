<!--
# Checklist

- code formatting: run `find -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs clang-format-14 -style=file -i` in the root directory

- add unit test(s)

- ensure tests build and check results: run `colcon test --packages-select <package> && colcon test-result --all`

--!>
