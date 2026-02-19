configure_file(BoardSupportLoader.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)
target_sources(limesuiteng PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)

target_sources(limesuiteng PRIVATE DeviceRegistry.cpp DeviceHandle.cpp LMS7002M_SDRDevice.cpp GPIO.cpp)

if(LIMESUITENG_PCIE)
    target_sources(limesuiteng PRIVATE DeviceFactoryPCIe.cpp)
    add_subdirectory(LimeSDR_X3)
    add_subdirectory(LimeSDR_XTRX)
    add_subdirectory(MMX8)
    add_subdirectory(sSDR)
endif()

if(LIMESUITENG_USB_FX3)
    add_subdirectory(LimeSDR)
    target_sources(limesuiteng PRIVATE DeviceFactoryFX3.cpp)
endif()

if(LIMESUITENG_USB_FTDI)
    add_subdirectory(LimeSDR_Mini)
    target_sources(limesuiteng PRIVATE DeviceFactoryFTDI.cpp)
endif()

cmake_dependent_option(LIMESUITENG_LITEX_M2SDR "Enable LiteX-M2SDR (AD9361) PCIe board support" ON "UNIX; NOT APPLE" OFF)
add_feature_info("LIMESUITENG_LITEX_M2SDR" LIMESUITENG_LITEX_M2SDR "LiteX-M2SDR AD9361 PCIe board")

if(LIMESUITENG_LITEX_M2SDR)
    target_sources(limesuiteng PRIVATE DeviceFactoryLiteXM2SDR.cpp)
    add_subdirectory(LiteXM2SDR)
endif()
