# Checklist

- [ ] code formatting: run `find -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs clang-format-3.9 -style=file -i` in the root directory

- [ ] add unit test(s)

- [ ] ensure tests build and check results: run `catkin run_tests && catkin_test_results`
