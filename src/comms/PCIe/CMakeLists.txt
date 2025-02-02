set(THIS_SOURCE_DIR comms/PCIe)

set(COMMS_LIMEPCIE_SOURCES
    ${THIS_SOURCE_DIR}/LMS64C_FPGA_Over_PCIe.cpp
    ${THIS_SOURCE_DIR}/LMS64C_LMS7002M_Over_PCIe.cpp
    ${THIS_SOURCE_DIR}/LimePCIe.cpp
    ${THIS_SOURCE_DIR}/LimePCIeDMA.cpp
    ${THIS_SOURCE_DIR}/PCIE_CSR_Pipe.cpp)

########################################################################
## Feature registration
########################################################################
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_LIMEPCIE "Enable Lite PCIe" ON "UNIX;ENABLE_LIBRARY; NOT APPLE" OFF)
add_feature_info(LIMEPCIE ENABLE_LIMEPCIE "PCIe support")
if(NOT ENABLE_LIMEPCIE)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(limesuiteng PRIVATE ${COMMS_LIMEPCIE_SOURCES})
target_include_directories(limesuiteng PRIVATE ${THIS_SOURCE_DIR})

# build kernel module only on Linux
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_include_directories(limesuiteng PRIVATE ${THIS_SOURCE_DIR}/linux-kernel-module)

    cmake_dependent_option(LIMEPCIE_KERNEL_MODULE "Build Linux LimePCIe kernel module" ON "UNIX; NOT APPLE" OFF)
    add_feature_info(LIMEPCIE_KERNEL_MODULE LIMEPCIE_KERNEL_MODULE "Build Linux LimePCIe kernel module")
    if(LIMEPCIE_KERNEL_MODULE)
        add_subdirectory(${THIS_SOURCE_DIR}/linux-kernel-module)
    endif()
endif()
