# This is only used for test compiles for MacOS.
# The actual build process is done on github.

set(CMAKE_SYSTEM_NAME Darwin)
set(TOOLCHAIN_PREFIX /osxcross/target/bin/oa64)
set(CMAKE_OSX_DEPLOYMENT_TARGET "14.0.0" CACHE STRING "Minimum OS X deployment version")

set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

# need a local installation that supports symbolic links
# Shared folders from Windows won't work.
set(SDK "/osxcross/SDK" CACHE PATH "X-Plane SDK root")

# cross compilers to use for C and C++
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-clang)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-clang++)
