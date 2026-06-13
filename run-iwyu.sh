cmake --preset lx -DCMAKE_CXX_INCLUDE_WHAT_YOU_USE=include-what-you-use
cmake --build --preset lx > iwyu.log 2>&1
