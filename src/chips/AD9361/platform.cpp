/*
 * AD9361 no-OS platform implementation for LimeSuiteNG / M2SDR
 *
 * Maps the Analog Devices no-OS platform API to LimeSuiteNG ISerialPort
 * for FPGA CSR access via CMD_BRDCSR_WR/RD (LMS64C protocol).
 *
 * Copyright (c) 2024-2026 Lime Microsystems
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "platform.h"

#include <cstdint>
#include <cstring>
#include <unistd.h>

#include "comms/ISerialPort.h"
#include "protocols/LMS64CProtocol.h"

/*
 * FPGA CSR addresses for the AD9361 SPI master on M2SDR
 * (from LimeSDR_GW csr.csv)
 */
static constexpr uint32_t AD9361_CONFIG_ADDR   = 0xf0004060; /* rst_n[0], enable[1] */
static constexpr uint32_t AD9361_SPI_CTRL_ADDR = 0xf0004070; /* start[0], length[15:8] */
static constexpr uint32_t AD9361_SPI_STAT_ADDR = 0xf0004074; /* done[0] */
static constexpr uint32_t AD9361_SPI_MOSI_ADDR = 0xf0004078; /* 24-bit TX data */
static constexpr uint32_t AD9361_SPI_MISO_ADDR = 0xf000407c; /* 8-bit RX data */

static constexpr uint32_t SPI_CTRL_START  = (1 << 0);
static constexpr uint32_t SPI_CTRL_LENGTH = (1 << 8); /* multiply by bit count */
static constexpr uint32_t SPI_STAT_DONE   = (1 << 0);

/* Global serial port pointer - must be set before calling ad9361_init */
static lime::ISerialPort* g_spi_port = nullptr;

/* Called by M2SDR board class before ad9361_init() */
void m2sdr_platform_set_port(lime::ISerialPort* port)
{
    g_spi_port = port;
}

static inline void csr_write(uint32_t addr, uint32_t value)
{
    if (g_spi_port)
        lime::LMS64CProtocol::CSRegIoWrite(*g_spi_port, addr, value);
}

static inline uint32_t csr_read(uint32_t addr)
{
    if (!g_spi_port)
        return 0;
    lime::OpStatus st;
    return static_cast<uint32_t>(lime::LMS64CProtocol::CSRegIoRead(*g_spi_port, addr, &st));
}

extern "C" {

int32_t spi_init(uint32_t device_id, uint8_t clk_pha, uint8_t clk_pol)
{
    /* Hardware-reset the AD9361: deassert rst_n, then assert */
    csr_write(AD9361_CONFIG_ADDR, 0x00); /* rst_n=0, enable=0 */
    usleep(1000);
    csr_write(AD9361_CONFIG_ADDR, 0x03); /* rst_n=1, enable=1 */
    usleep(1000);
    return 0;
}

int32_t spi_read(uint8_t *data, uint8_t bytes_number)
{
    return 0; /* not used by ad9361 driver */
}

/*
 * AD9361 SPI frame is always 24 bits:
 *   bit 23    : R/W (1=write, 0=read)
 *   bits 22:8 : 15-bit register address
 *   bits 7:0  : 8-bit data
 */
int spi_write_then_read(struct spi_device *spi,
        const unsigned char *txbuf, unsigned n_tx,
        unsigned char *rxbuf, unsigned n_rx)
{
    if (n_tx < 3 || !txbuf)
        return -1;

    bool is_write = (txbuf[0] & 0x80) != 0;

    /* Pack 3 TX bytes into 24-bit MOSI word */
    uint32_t mosi = ((uint32_t)txbuf[0] << 16)
                  | ((uint32_t)txbuf[1] << 8)
                  |  (uint32_t)txbuf[2];

    csr_write(AD9361_SPI_MOSI_ADDR, mosi);

    /* Start SPI: 24 bits, start pulse */
    csr_write(AD9361_SPI_CTRL_ADDR, (24 * SPI_CTRL_LENGTH) | SPI_CTRL_START);

    /* Poll until done (hardware typically completes in < 10 µs) */
    for (int i = 0; i < 100000; i++)
    {
        uint32_t status = csr_read(AD9361_SPI_STAT_ADDR);
        if (status & SPI_STAT_DONE)
            break;
        usleep(1);
    }

    /* Capture MISO for read transactions */
    if (!is_write && n_rx > 0 && rxbuf)
        rxbuf[0] = static_cast<unsigned char>(csr_read(AD9361_SPI_MISO_ADDR) & 0xFF);

    return 0;
}

void gpio_init(uint32_t device_id)
{
    /* no-op: GPIOs managed through CSR config register */
}

void gpio_direction(uint8_t pin, uint8_t direction)
{
    /* no-op */
}

bool gpio_is_valid(int number)
{
    /* All AD9361 GPIOs in default_init_param are -1, so always false */
    return false;
}

void gpio_set_value(unsigned gpio, int value)
{
    /* no-op */
}

void udelay(unsigned long usecs)
{
    usleep(usecs);
}

void mdelay(unsigned long msecs)
{
    usleep(msecs * 1000UL);
}

unsigned long msleep_interruptible(unsigned int msecs)
{
    usleep(msecs * 1000UL);
    return 0;
}

/* AXI ADC functions - not used on M2SDR (no AXI HDL core on host side) */

void axiadc_init(struct ad9361_rf_phy *phy)
{
    /* no-op */
}

int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
    return 0;
}

unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
    return 0;
}

void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
    /* no-op */
}

int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
    return 0;
}

void axiadc_idelay_set(struct axiadc_state *st, unsigned lane, unsigned val)
{
    /* no-op */
}

} /* extern "C" */
