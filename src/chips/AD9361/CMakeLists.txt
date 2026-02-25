# AD9361 no-OS driver + LimeSuiteNG C++ wrapper
#
# The C driver files are compiled as C (not C++).
# platform.cpp and AD9361.cpp are C++ and use ISerialPort for FPGA CSR access.

set(AD9361_NOOS_SOURCES
    ad9361.c
    ad9361_api.c
    ad9361_conv.c
    util.c
)

set(AD9361_CPP_SOURCES
    platform.cpp
    AD9361.cpp
)

# Compile the no-OS C files with C compiler, suppressing upstream warnings
set_source_files_properties(${AD9361_NOOS_SOURCES} PROPERTIES
    LANGUAGE C
    COMPILE_OPTIONS "-w"          # suppress all warnings from third-party code
)

target_sources(limesuiteng PRIVATE ${AD9361_NOOS_SOURCES} ${AD9361_CPP_SOURCES})

# The no-OS driver includes its own headers via relative paths;
# add this directory so #include "ad9361.h" etc. resolve correctly.
target_include_directories(limesuiteng PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
