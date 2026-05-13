# CMake toolchain file for building macOS targets from Linux using osxcross.
# Usage:
#   cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-osxcross.cmake \
#         -DCMAKE_BUILD_TYPE=Release ..

set(CMAKE_SYSTEM_NAME Darwin)
set(CMAKE_SYSTEM_PROCESSOR "x86_64;arm64")

set(OSXCROSS_ROOT "/osxcross/target" CACHE PATH "osxcross target root")

# Use older SDK that osxcross understands (frameworks layout)
set(SDK "${CMAKE_CURRENT_LIST_DIR}/../SDK-4.0.1" CACHE PATH "X-Plane SDK root")

set(CMAKE_C_COMPILER   "${OSXCROSS_ROOT}/bin/o64-clang")
set(CMAKE_CXX_COMPILER "${OSXCROSS_ROOT}/bin/o64-clang++")

# Universal binary via lipo is handled by the Makefile.
# Under CMake/osxcross we build x86_64 only by default; set
# CMAKE_OSX_ARCHITECTURES on the command line to override.
set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "Target architecture")
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "macOS deployment target")

# Static expat shipped with macPorts inside osxcross
set(OSXCROSS_EXPAT_LIB
    "${OSXCROSS_ROOT}/macports/pkgs/opt/local/lib/libexpat.a"
    CACHE FILEPATH "Path to static libexpat.a from osxcross macPorts")
