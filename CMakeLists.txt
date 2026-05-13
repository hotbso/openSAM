cmake_minimum_required(VERSION 3.20)
project(openSAM LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# Configurable paths
# ==============================================================================
set(SDK "${CMAKE_CURRENT_SOURCE_DIR}/../SDK" CACHE PATH "X-Plane SDK root")
set(XPLIB "${CMAKE_CURRENT_SOURCE_DIR}/../xplib" CACHE PATH "xplib root")

# ==============================================================================
# Version header generation (mirrors Makefile.common logic)
# ==============================================================================
find_package(Git)
set(OPENSAM_COMMIT "unknown")
set(XPLIB_COMMIT "unknown")
if(GIT_FOUND)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE OPENSAM_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%h
        WORKING_DIRECTORY ${XPLIB}
        OUTPUT_VARIABLE XPLIB_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
set(VERSION_SHORT "unknown")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.mak")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.mak" _ver_mak)
    if(_ver_mak MATCHES "VERSION=([^\r\n]+)")
        set(VERSION_SHORT "${CMAKE_MATCH_1}")
    endif()
endif()
set(VERSION_L "${VERSION_SHORT} ${OPENSAM_COMMIT},xplib:${XPLIB_COMMIT}")
set(_vh "${CMAKE_CURRENT_SOURCE_DIR}/version.h")
set(_vh_content "// generated file!\n#define VERSION \"${VERSION_L}\"\n#define VERSION_SHORT \"${VERSION_SHORT}\"\n")
if(EXISTS "${_vh}")
    file(READ "${_vh}" _vh_existing)
    if(NOT _vh_existing STREQUAL _vh_content)
        file(WRITE "${_vh}" "${_vh_content}")
    endif()
else()
    file(WRITE "${_vh}" "${_vh_content}")
endif()

# ==============================================================================
# Sources (Makefile.common: SOURCES + xplib files resolved from VPATH)
# ==============================================================================
set(SOURCES
    opensam.cpp adgs_ui.cpp autodgs_airport.cpp adgs_api.cpp seasons.cpp
    opensam_airport.cpp samjw.cpp jwctrl.cpp jwctrl_sound.cpp os_ui.cpp os_anim.cpp sam_xml.cpp read_wav.cpp
    plane.cpp myplane.cpp LTAPI.cpp mpadapter.cpp mpadapter_xpilot.cpp mpadapter_tgxp.cpp mpadapter_lt.cpp
    ${XPLIB}/log_msg.cpp ${XPLIB}/widget_ctx.cpp ${XPLIB}/simbrief.cpp
    dgs/apt_airport.cpp dgs/dgs.cpp dgs/dgs_plane.cpp dgs/airport.cpp dgs/marshaller.cpp
    dgs/safedock_t2_24.cpp dgs/safedock_x.cpp sam1_dgs.cpp
    XPListBox.cpp
)

add_library(openSAM SHARED ${SOURCES})
set_target_properties(openSAM PROPERTIES PREFIX "" SUFFIX ".xpl")

# ==============================================================================
# Common compile settings
# ==============================================================================
target_include_directories(openSAM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/dgs
    ${XPLIB}
    ${SDK}/CHeaders/XPLM
    ${SDK}/CHeaders/Widgets
)

target_compile_definitions(openSAM PRIVATE
    XPLM200 XPLM210 XPLM300 XPLM301 XPLM400
    DREF_PREFIX="opensam"
)

target_compile_options(openSAM PRIVATE -O3)

# ==============================================================================
# Platform specifics
# ==============================================================================
if(WIN32)
    # Makefile.mgw64
    target_compile_definitions(openSAM PRIVATE WINDOWS WIN32 IBM=1)
    target_compile_options(openSAM PRIVATE
        -Wpedantic -Wextra -Wall -fdiagnostics-color -Wno-format-overflow -Wno-format-truncation
    )
    # -static causes multiple definition of std::type_info on MinGW when building
    # a shared module; --allow-multiple-definition resolves it (matches Makefile behavior)
    target_link_options(openSAM PRIVATE
        -static-libgcc -static -Wl,--allow-multiple-definition
    )
    target_link_libraries(openSAM PRIVATE
        ${SDK}/Libraries/Win/XPLM_64.lib
        ${SDK}/Libraries/Win/XPWidgets_64.lib
        -l:libexpat.a
        pthread
    )
    install(TARGETS openSAM RUNTIME DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/openSAM-pkg/openSAM/win_x64")

elseif(APPLE)
    # Makefile.mac64 (native macOS or osxcross)
    set(CMAKE_OSX_ARCHITECTURES "x86_64;arm64" CACHE STRING "Universal binary")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "13.0" CACHE STRING "macOS deployment target")

    target_compile_definitions(openSAM PRIVATE APL=1)
    target_compile_options(openSAM PRIVATE
        -Wall -fPIC -fno-stack-protector -fvisibility=hidden -ffast-math -Wno-deprecated-declarations
    )
    target_link_options(openSAM PRIVATE
        -rdynamic -fvisibility=hidden
        -Wl,-exported_symbols_list,${CMAKE_CURRENT_SOURCE_DIR}/linkscript.mac
    )
    find_library(XPLM_FW     XPLM     PATHS "${SDK}/Libraries/Mac" REQUIRED)
    find_library(XPWIDGETS_FW XPWidgets PATHS "${SDK}/Libraries/Mac" REQUIRED)
    # When cross-compiling via osxcross the toolchain file sets OSXCROSS_EXPAT_LIB
    if(DEFINED OSXCROSS_EXPAT_LIB AND EXISTS "${OSXCROSS_EXPAT_LIB}")
        set(_expat "${OSXCROSS_EXPAT_LIB}")
    else()
        set(_expat expat)   # ships with macOS / Homebrew / MacPorts natively
    endif()
    target_link_libraries(openSAM PRIVATE ${XPLM_FW} ${XPWIDGETS_FW} ${_expat})
    install(TARGETS openSAM LIBRARY DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/openSAM-pkg/openSAM/mac_x64")

elseif(UNIX)
    # Makefile.lin64
    target_compile_definitions(openSAM PRIVATE LIN=1)
    target_compile_options(openSAM PRIVATE
        -Wall -Wextra -Wno-format-overflow -Wno-format-truncation
        -fPIC -fno-stack-protector
    )
    target_link_options(openSAM PRIVATE -rdynamic -nodefaultlibs)
    target_link_libraries(openSAM PRIVATE expat)
    install(TARGETS openSAM LIBRARY DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/openSAM-pkg/openSAM/lin_x64")
endif()

# ==============================================================================
# sam_xml_test (all platforms, matching Makefile.mgw64 test target)
# ==============================================================================
if (WIN32)
    option(BUILD_TESTS "Build sam_xml_test" ON)
endif()

if(BUILD_TESTS)
    add_executable(sam_xml_test
        sam_xml_test.cpp
        sam_xml.cpp
        dgs/apt_airport.cpp
        ${XPLIB}/log_msg.cpp
    )
    target_include_directories(sam_xml_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${XPLIB}
        ${SDK}/CHeaders/XPLM
    )
    target_compile_definitions(sam_xml_test PRIVATE
        XPLM200 XPLM210 XPLM300 XPLM301
        LOCAL_DEBUGSTRING
        $<IF:$<BOOL:${WIN32}>,WINDOWS WIN32 IBM=1,>
        $<IF:$<BOOL:${APPLE}>,APL=1,>
        $<IF:$<AND:$<BOOL:${UNIX}>,$<NOT:$<BOOL:${APPLE}>>>,LIN=1,>
    )
    target_compile_options(sam_xml_test PRIVATE -Wall -Wno-format-overflow)
    if(WIN32)
        target_link_libraries(sam_xml_test PRIVATE -l:libexpat.a)
    else()
        target_link_libraries(sam_xml_test PRIVATE expat)
    endif()
endif()
