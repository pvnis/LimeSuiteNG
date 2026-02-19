# LiteXM2SDR board support — AD9361 RFIC over LitePCIe
#
# Depends on:
#   * Linux kernel headers for litepcie (ioctl definitions, csr.h)
#   * litex_m2sdr user-space libraries: libm2sdr, liblitepcie
#   * Analog Devices AD9361 driver (compiled from sources inside the repo)

set(LITEX_SW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../litex_m2sdr/litex_m2sdr/software")
set(LITEX_KERNEL_DIR "${LITEX_SW_ROOT}/kernel")
set(LITEX_USER_DIR   "${LITEX_SW_ROOT}/user")

# ---- AD9361 driver sources -----------------------------------------------
set(AD9361_SOURCES
    "${LITEX_USER_DIR}/ad9361/ad9361_api.c"
    "${LITEX_USER_DIR}/ad9361/ad9361.c"
    "${LITEX_USER_DIR}/ad9361/ad9361_conv.c"
    "${LITEX_USER_DIR}/ad9361/util.c"
)

# ---- libm2sdr sources (SI5351, AD9361 SPI bridge) ------------------------
set(LIBM2SDR_SOURCES
    "${LITEX_USER_DIR}/libm2sdr/m2sdr_ad9361_spi.c"
    "${LITEX_USER_DIR}/libm2sdr/m2sdr_si5351_i2c.c"
    "${LITEX_USER_DIR}/libm2sdr/m2sdr_flash.c"
)

# ---- liblitepcie sources (DMA management) --------------------------------
set(LIBLITEPCIE_SOURCES
    "${LITEX_USER_DIR}/liblitepcie/litepcie_dma.c"
    "${LITEX_USER_DIR}/liblitepcie/litepcie_helpers.c"
)

# ---- Add all sources to limesuiteng target --------------------------------
target_sources(limesuiteng PRIVATE
    LiteXM2SDR.cpp
    LiteXM2SDRStreaming.cpp
    ${AD9361_SOURCES}
    ${LIBM2SDR_SOURCES}
    ${LIBLITEPCIE_SOURCES}
)

# ---- Include directories --------------------------------------------------
target_include_directories(limesuiteng PRIVATE
    # LitePCIe kernel header (csr.h, litepcie.h, config.h, soc.h, mem.h):
    "${LITEX_KERNEL_DIR}"
    # liblitepcie user header (litepcie_dma.h, litepcie_helpers.h, liblitepcie.h):
    "${LITEX_USER_DIR}/liblitepcie"
    # libm2sdr (libm2sdr.h, m2sdr_ad9361_spi.h, m2sdr_config.h, m2sdr_si5351_i2c.h):
    "${LITEX_USER_DIR}/libm2sdr"
    # AD9361 driver (ad9361_api.h, ad9361.h, platform.h, util.h):
    "${LITEX_USER_DIR}"
    # libliteeth — etherbone.h is included by libm2sdr sources even in USE_LITEPCIE mode:
    "${LITEX_USER_DIR}/libliteeth"
)

# ---- Compile definitions --------------------------------------------------
# USE_LITEPCIE: selects PCIe backend in libm2sdr macros
target_compile_definitions(limesuiteng PRIVATE USE_LITEPCIE=1)

# Suppress known warnings from the third-party AD9361 C code
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(
        ${AD9361_SOURCES}
        ${LIBM2SDR_SOURCES}
        ${LIBLITEPCIE_SOURCES}
        PROPERTIES COMPILE_FLAGS
            "-Wno-sign-compare -Wno-implicit-fallthrough -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-old-style-declaration"
    )
endif()
