cmake_minimum_required(VERSION 3.5)

# Get the version number from header file
file(READ version.h VERSION_HEADER_CONTENT)
string(REGEX MATCH "LIMEPCIE_VERSION \"([0-9]+\\.[0-9]+\\.[0-9]+)\"" DRIVER_VERSION "${VERSION_HEADER_CONTENT}")

if(NOT DRIVER_VERSION)
    message(FATAL_ERROR "Failed to extract version number from version.h")
endif(NOT DRIVER_VERSION)
set(DRIVER_VERSION "${CMAKE_MATCH_1}")

project(
    limepcie-kernel
    VERSION ${DRIVER_VERSION}
    LANGUAGES C)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(MODULE_NAME limepcie)

find_package(LinuxKernelHeaders REQUIRED)

# Get kernel version
execute_process(
    COMMAND uname -r
    OUTPUT_VARIABLE KERNEL_RELEASE
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(KERNEL_SOURCE_DIR /lib/modules/${KERNEL_RELEASE}/build)

# Get architecture
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
# ARCH might be 'aarch64', but the linux lib directories might be named 'arm64'
if(${ARCH} STREQUAL "aarch64" AND NOT EXISTS ${KERNEL_SOURCE_DIR}/arch/${ARCH})
    set(ARCH "arm64")
endif()

# where Kbuild file will be placed
set(KBUILD_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Generate the Kbuild file through cmake.
file(
    WRITE ${KBUILD_FILE_DIR}/Kbuild
    "ccflags-y += -Wno-declaration-after-statement
    ccflags-y += -std=gnu11
    ccflags-y += -I${KBUILD_FILE_DIR}/bsp
    obj-m = limepcie.o")

set(MODULE_OBJECT ${KBUILD_FILE_DIR}/${MODULE_NAME}.ko)
set(KBUILD_CMD
    $(MAKE)
    -C
    ${KERNEL_SOURCE_DIR}
    ARCH=${ARCH}
    # Informs kbuild that an external module is being built.
    # The value given to "M" is the absolute path of the directory where the external module (kbuild file) is located.
    M=${KBUILD_FILE_DIR}
    modules)

set(KBUILD_CLEAN_CMD $(MAKE) -C ${KERNEL_SOURCE_DIR} ARCH=${ARCH} M=${KBUILD_FILE_DIR} clean)

set(KERNEL_SOURCE_RELATIVE_PATHS version.h limepcie.c limepcie.h boards.h bsp/config.h bsp/csr.h bsp/flags.h bsp/mem.h bsp/soc.h)

# Copy all source files into build directory and compile there, as the Kbuild produces artifacts in tree
foreach(SRC_FILENAME ${KERNEL_SOURCE_RELATIVE_PATHS})
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${SRC_FILENAME} ${KBUILD_FILE_DIR}/${SRC_FILENAME} COPYONLY)
endforeach()

add_custom_command(
    OUTPUT ${MODULE_OBJECT}
    COMMAND ${KBUILD_CLEAN_CMD}
    COMMAND ${KBUILD_CMD}
    WORKING_DIRECTORY ${KBUILD_FILE_DIR}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR} # rebuild if anything changes in the source dir
    VERBATIM
    COMMENT "Building Linux kernel module in dir: ${KBUILD_FILE_DIR}")

add_custom_target(limepcie-kernel ALL DEPENDS ${MODULE_OBJECT})

cmake_dependent_option(INSTALL_KERNEL_MODULE "Install Linux LimePCIe kernel module" ON "LIMEPCIE_KERNEL_MODULE" ON)
if(NOT INSTALL_KERNEL_MODULE)
    return()
endif()

# 0 means success, so it's inverted
execute_process(
    COMMAND dkms --version
    RESULT_VARIABLE IS_DKMS_NOT_PRESENT
    OUTPUT_QUIET)

cmake_dependent_option(USE_DKMS "Use DKMS when installing the kernel module" ON "NOT IS_DKMS_NOT_PRESENT" OFF)
add_feature_info(DKMS USE_DKMS "DKMS support for lime PCIe kernel module")

set(MODE_FLAG "--dkms")
if(NOT USE_DKMS)
    set(KERNEL_MODULE_DESTINATION /lib/modules/${KERNEL_RELEASE}/extra)
    install(FILES ${MODULE_OBJECT} DESTINATION ${KERNEL_MODULE_DESTINATION})

    set(MODE_FLAG "--legacy")
endif()

# If module is already loaded, unload it
install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/uninstall-legacy-module.sh)")
install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/unload.sh ${MODULE_NAME})")

install(
    CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh ${MODE_FLAG} ${MODULE_NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})"
)

add_custom_target(
    uninstall-kernel-module
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/uninstall.sh ${MODE_FLAG} ${MODULE_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Uninstalling Linux kernel module ${MODULE_NAME}")
