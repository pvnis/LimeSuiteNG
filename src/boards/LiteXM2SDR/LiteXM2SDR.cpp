/*
 * LimeSuiteNG support for LiteX-M2SDR (AD9361 RFIC, PCIe).
 *
 * Copyright (c) 2024-2026 Enjoy-Digital / LimeMicro contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "LiteXM2SDR.h"
#include "LiteXM2SDRStreaming.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/StreamConfig.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <cstdio>

// ---- litepcie / m2sdr / AD9361 includes ---------------------------------
// Kernel header (CSR register map & ioctl defs):
#include "litepcie.h"
// User-space helpers:
#include "litepcie_helpers.h"
// m2sdr library (wraps litepcie_readl/writel as m2sdr_readl/writel):
#include "libm2sdr.h"
// SI5351 clock generator I2C config:
#include "m2sdr_si5351_i2c.h"
// AD9361 SPI wrapper (uses m2sdr_readl/writel internally):
#include "m2sdr_ad9361_spi.h"
// Default initialization parameters (38.4 MHz refclk, 30.72 MS/s, 2T2R, FDD):
#include "m2sdr_config.h"
// AD9361 driver API:
#include "ad9361/ad9361_api.h"
#include "ad9361/ad9361.h"
// AD9361 platform declarations (spi_write_then_read, udelay, etc.):
#include "ad9361/platform.h"

using namespace std::literals::string_literals;

// =========================================================================
//  AD9361 platform glue — C-linkage functions called by the driver
// =========================================================================

namespace {

// Map: spi-id → device file descriptor
std::mutex g_spi_map_mutex;
std::unordered_map<uint8_t, int> g_spi_fd_map;
uint8_t g_spi_next_id = 0;
int g_spi_last_fd = -1;

uint8_t limexm2sdr_spi_register(int fd)
{
    std::lock_guard<std::mutex> lock(g_spi_map_mutex);
    for (uint16_t i = 0; i < 256; ++i)
    {
        uint8_t id = static_cast<uint8_t>(g_spi_next_id + i);
        if (g_spi_fd_map.find(id) == g_spi_fd_map.end())
        {
            g_spi_next_id = static_cast<uint8_t>(id + 1);
            g_spi_fd_map[id] = fd;
            g_spi_last_fd = fd;
            return id;
        }
    }
    throw std::runtime_error("limexm2sdr_spi_register: no free SPI ids");
}

void limexm2sdr_spi_unregister(uint8_t id)
{
    std::lock_guard<std::mutex> lock(g_spi_map_mutex);
    g_spi_fd_map.erase(id);
    g_spi_last_fd = g_spi_fd_map.empty() ? -1 : g_spi_fd_map.begin()->second;
}

static int limexm2sdr_spi_get_fd(const struct spi_device* spi)
{
    std::lock_guard<std::mutex> lock(g_spi_map_mutex);
    auto it = g_spi_fd_map.find(spi->id_no);
    if (it != g_spi_fd_map.end())
        return it->second;
    return g_spi_last_fd; // fallback
}

} // anonymous namespace

// These are defined as extern "C" in platform.h and called by the AD9361 C driver:
extern "C" {

int spi_write_then_read(struct spi_device* spi,
    const unsigned char* txbuf,
    unsigned n_tx,
    unsigned char* rxbuf,
    unsigned n_rx)
{
    int fd = limexm2sdr_spi_get_fd(spi);
    if (fd < 0)
        return -1;
    void* conn = reinterpret_cast<void*>(static_cast<intptr_t>(fd));

    if (n_tx == 2 && n_rx == 1)
    {
        // Single byte read
        rxbuf[0] = m2sdr_ad9361_spi_read(conn, static_cast<uint16_t>(txbuf[0] << 8 | txbuf[1]));
    }
    else if (n_tx == 3 && n_rx == 0)
    {
        // Single byte write
        m2sdr_ad9361_spi_write(conn, static_cast<uint16_t>(txbuf[0] << 8 | txbuf[1]), txbuf[2]);
    }
    else
    {
        lime::warning("LiteXM2SDR spi_write_then_read: unsupported n_tx=%u n_rx=%u", n_tx, n_rx);
        return -1;
    }
    return 0;
}

void udelay(unsigned long usecs) { usleep(static_cast<useconds_t>(usecs)); }

void mdelay(unsigned long msecs) { usleep(static_cast<useconds_t>(msecs * 1000)); }

unsigned long msleep_interruptible(unsigned int msecs)
{
    usleep(static_cast<useconds_t>(msecs * 1000));
    return 0;
}

bool gpio_is_valid(int number) { return number == 0; } // GPIO_RESET_PIN = 0

void gpio_set_value(unsigned /*gpio*/, int /*value*/) {}
void gpio_init(uint32_t /*device_id*/) {}
void gpio_direction(uint8_t /*pin*/, uint8_t /*direction*/) {}

int32_t spi_init(uint32_t /*device_id*/, uint8_t /*clk_pha*/, uint8_t /*clk_pol*/) { return 0; }
int32_t spi_read(uint8_t* /*data*/, uint8_t /*bytes_number*/) { return 0; }

// axiadc stubs (not used but referenced by some driver builds)
void axiadc_init(struct ad9361_rf_phy* /*phy*/) {}
int axiadc_post_setup(struct ad9361_rf_phy* /*phy*/) { return 0; }
unsigned int axiadc_read(struct axiadc_state* /*st*/, unsigned long /*reg*/) { return 0; }
void axiadc_write(struct axiadc_state* /*st*/, unsigned /*reg*/, unsigned /*val*/) {}
int axiadc_set_pnsel(struct axiadc_state* /*st*/, int /*channel*/, enum adc_pn_sel /*sel*/) { return 0; }
void axiadc_idelay_set(struct axiadc_state* /*st*/, unsigned /*lane*/, unsigned /*val*/) {}

} // extern "C"

// =========================================================================
//  LiteXM2SDR implementation
// =========================================================================

namespace lime {

// -------------------------------------------------------------------------
// Static helper: read DNA serial from device fd
// -------------------------------------------------------------------------
std::string LiteXM2SDR::ReadSerial(int fd)
{
    // CSR_DNA_ID_ADDR is defined in the auto-generated csr.h
    // The DNA ID is 57 bits, read as two 32-bit words (upper + lower)
    // For simplicity, read the lower 32-bit word and format as hex.
    uint32_t dna_lo = litepcie_readl(fd, CSR_DNA_ID_ADDR + 4);
    uint32_t dna_hi = litepcie_readl(fd, CSR_DNA_ID_ADDR);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%08X%08X", dna_hi, dna_lo);
    return std::string(buf);
}

// -------------------------------------------------------------------------
// Constructor
// -------------------------------------------------------------------------
LiteXM2SDR::LiteXM2SDR(const std::string& devPath)
    : mFd(-1)
    , mSpiId(0)
    , mAd9361(nullptr)
    , mRxFreq(2.4e9)
    , mTxFreq(2.4e9)
    , mRxSampleRate(30.72e6)
    , mTxSampleRate(30.72e6)
    , mRxBandwidth(30.72e6)
    , mTxBandwidth(30.72e6)
{
    // Default gain values
    for (int ch = 0; ch < MAX_CHANNELS; ++ch)
    {
        mRxGain[ch] = 0.0;
        mTxAtten[ch] = 20000; // 20 dB
        mRxAntenna[ch] = 0;
        mTxAntenna[ch] = 0;
        mChannelEnabled[ch][0] = false;
        mChannelEnabled[ch][1] = false;
    }

    mFd = open(devPath.c_str(), O_RDWR | O_CLOEXEC);
    if (mFd < 0)
        throw std::runtime_error("LiteXM2SDR: cannot open "s + devPath + ": "s + strerror(errno));

    mSpiId = limexm2sdr_spi_register(mFd);

    BuildDescriptor();
    InitHardware(true);
}

// -------------------------------------------------------------------------
// Destructor
// -------------------------------------------------------------------------
LiteXM2SDR::~LiteXM2SDR()
{
    if (mAd9361)
    {
        // Power down AD9361
        litepcie_writel(mFd, CSR_AD9361_CONFIG_ADDR, 0b00);
    }
    limexm2sdr_spi_unregister(mSpiId);
    if (mFd >= 0)
        close(mFd);
}

// -------------------------------------------------------------------------
// Build device descriptor
// -------------------------------------------------------------------------
void LiteXM2SDR::BuildDescriptor()
{
    mDeviceDescriptor.name = "LiteXM2SDR"s;
    mDeviceDescriptor.expansionName = "AD9361"s;
    mDeviceDescriptor.firmwareVersion = "N/A"s;
    mDeviceDescriptor.gatewareVersion = "N/A"s;
    mDeviceDescriptor.serialNumber = 0;

    // Try to read gateware identifier string
    {
        // identifier is stored as ASCII bytes in IDENTIFIER_MEM_BASE
        char ident[64] = {};
        for (int i = 0; i < 16; ++i)
        {
            uint32_t word = litepcie_readl(mFd, CSR_IDENTIFIER_MEM_BASE + i * 4);
            ident[i * 4 + 0] = static_cast<char>((word >> 24) & 0xFF);
            ident[i * 4 + 1] = static_cast<char>((word >> 16) & 0xFF);
            ident[i * 4 + 2] = static_cast<char>((word >> 8) & 0xFF);
            ident[i * 4 + 3] = static_cast<char>((word >> 0) & 0xFF);
        }
        ident[sizeof(ident) - 1] = '\0';
        mDeviceDescriptor.gatewareTargetBoard = std::string(ident);
    }

    // AD9361 RFIC descriptor
    RFSOCDescriptor soc;
    soc.name = "AD9361"s;
    soc.channelCount = 2;

    // Frequency ranges
    soc.frequencyRange = { 70e6, 6e9 };
    soc.samplingRateRange = { 520834.0, 61.44e6 };

    // Antenna paths
    // RX: A_BALANCED (differential input), B_BALANCED, C_BALANCED
    soc.pathNames[TRXDir::Rx] = { "A_BALANCED"s, "B_BALANCED"s, "C_BALANCED"s };
    soc.pathNames[TRXDir::Tx] = { "A"s, "B"s };

    soc.antennaRange[TRXDir::Rx]["A_BALANCED"s] = { 70e6, 6e9 };
    soc.antennaRange[TRXDir::Rx]["B_BALANCED"s] = { 70e6, 6e9 };
    soc.antennaRange[TRXDir::Rx]["C_BALANCED"s] = { 70e6, 6e9 };
    soc.antennaRange[TRXDir::Tx]["A"s] = { 47e6, 6e9 };
    soc.antennaRange[TRXDir::Tx]["B"s] = { 47e6, 6e9 };

    // Low-pass filter ranges
    soc.lowPassFilterRange[TRXDir::Rx] = { 200e3, 56e6 };
    soc.lowPassFilterRange[TRXDir::Tx] = { 1.25e6, 40e6 };

    // Gains
    // RX: combined gain 0..73 dB (via LNA+PGA+TIA chain, exposed as single value)
    soc.gains[TRXDir::Rx].insert(eGainTypes::LNA);
    soc.gainRange[TRXDir::Rx][eGainTypes::LNA] = { 0.0, 73.0 };

    // TX: PAD is the transmit attenuator. Range expressed as attenuation in dB [0..89].
    // Convention: 0 dB = maximum output power (no attenuation), 89 dB = minimum.
    soc.gains[TRXDir::Tx].insert(eGainTypes::PAD);
    soc.gainRange[TRXDir::Tx][eGainTypes::PAD] = { -89.0, 0.0 };

    mDeviceDescriptor.rfSOC.push_back(soc);
}

// -------------------------------------------------------------------------
// Hardware initialization
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::InitHardware(bool doInit)
{
    void* conn = reinterpret_cast<void*>(static_cast<intptr_t>(mFd));

    if (doInit)
    {
        // Configure bit-mode (16-bit / 12-bit, use 16-bit by default)
        litepcie_writel(mFd, CSR_AD9361_BITMODE_ADDR, 0); // 0 = 16-bit mode

        // Enable DMA synchronizer
        litepcie_writel(mFd, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 0);

        // Disable headers (basic data-only mode)
        litepcie_writel(mFd,
            CSR_HEADER_RX_CONTROL_ADDR,
            (1 << CSR_HEADER_RX_CONTROL_ENABLE_OFFSET) | (0 << CSR_HEADER_RX_CONTROL_HEADER_ENABLE_OFFSET));
        litepcie_writel(mFd,
            CSR_HEADER_TX_CONTROL_ADDR,
            (1 << CSR_HEADER_TX_CONTROL_ENABLE_OFFSET) | (0 << CSR_HEADER_TX_CONTROL_HEADER_ENABLE_OFFSET));

        // Disable DMA loopback
        litepcie_writel(mFd, CSR_PCIE_DMA0_LOOPBACK_ENABLE_ADDR, 0);

        // Initialize SI5351 with default 38.4 MHz config
#ifdef CSR_SI5351_BASE
        litepcie_writel(mFd, CSR_SI5351_CONTROL_ADDR, SI5351B_VERSION * (1 << CSR_SI5351_CONTROL_VERSION_OFFSET));
        m2sdr_si5351_i2c_config(conn,
            SI5351_I2C_ADDR,
            si5351_xo_38p4m_config,
            sizeof(si5351_xo_38p4m_config) / sizeof(si5351_xo_38p4m_config[0]));
#endif

        // Set default PHY timing mode (0 = 2T2R, matching default_init_param)
        litepcie_writel(mFd, CSR_AD9361_PHY_CONTROL_ADDR, 0);

        // Power up AD9361 (enable power + deassert reset via CONFIG register)
        litepcie_writel(mFd, CSR_AD9361_CONFIG_ADDR, 0b11);

        // Reset AD9361 SPI master and assert/deassert RESETB
        m2sdr_ad9361_spi_init(conn, 1);
    }

    // Initialize AD9361 RFIC driver
    AD9361_InitParam init_param = default_init_param;
    init_param.id_no = mSpiId;
    init_param.gpio_resetb = 0; // AD9361_GPIO_RESET_PIN
    init_param.gpio_sync = -1;
    init_param.gpio_cal_sw1 = -1;
    init_param.gpio_cal_sw2 = -1;

    int ret = ad9361_init(&mAd9361, &init_param, doInit ? 1 : 0);
    if (ret != 0)
    {
        lime::ReportError(OpStatus::Error, "LiteXM2SDR: ad9361_init failed (%d)", ret);
        return OpStatus::Error;
    }

    if (doInit)
    {
        // Load FIR filters
        ad9361_set_tx_fir_config(mAd9361, tx_fir_config);
        ad9361_set_rx_fir_config(mAd9361, rx_fir_config);

        // Apply defaults
        ad9361_set_rx_lo_freq(mAd9361, static_cast<uint64_t>(mRxFreq));
        ad9361_set_tx_lo_freq(mAd9361, static_cast<uint64_t>(mTxFreq));
        ad9361_set_rx_sampling_freq(mAd9361, static_cast<uint32_t>(mRxSampleRate));
        ad9361_set_tx_sampling_freq(mAd9361, static_cast<uint32_t>(mTxSampleRate));
        ad9361_set_rx_rf_bandwidth(mAd9361, static_cast<uint32_t>(mRxBandwidth));
        ad9361_set_tx_rf_bandwidth(mAd9361, static_cast<uint32_t>(mTxBandwidth));

        for (int ch = 0; ch < MAX_CHANNELS; ++ch)
        {
            ad9361_set_rx_rf_gain(mAd9361, static_cast<uint8_t>(ch), static_cast<int32_t>(mRxGain[ch]));
            ad9361_set_tx_attenuation(mAd9361, static_cast<uint8_t>(ch), static_cast<uint32_t>(mTxAtten[ch]));
        }
    }

    return OpStatus::Success;
}

// -------------------------------------------------------------------------
// GetDescriptor
// -------------------------------------------------------------------------
const SDRDescriptor& LiteXM2SDR::GetDescriptor() const { return mDeviceDescriptor; }

// -------------------------------------------------------------------------
// Configure
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::Configure(const SDRConfig& config, uint8_t moduleIndex)
{
    if (moduleIndex != 0)
        return OpStatus::InvalidValue;

    std::lock_guard<std::mutex> lock(mMutex);

    // Process up to 2 channels
    for (uint8_t ch = 0; ch < MAX_CHANNELS; ++ch)
    {
        const ChannelConfig& chanCfg = config.channel[ch];

        // RX channel
        const auto& rx = chanCfg.rx;
        if (rx.enabled)
        {
            if (rx.centerFrequency > 0)
                SetFrequency(0, TRXDir::Rx, ch, rx.centerFrequency);
            if (rx.sampleRate > 0)
                SetSampleRate(0, TRXDir::Rx, ch, rx.sampleRate, rx.oversample);
            if (rx.lpf > 0)
                SetLowPassFilter(0, TRXDir::Rx, ch, rx.lpf);
            for (const auto& [gainType, gainVal] : rx.gain)
                SetGain(0, TRXDir::Rx, ch, gainType, gainVal);
            SetAntenna(0, TRXDir::Rx, ch, rx.path);
        }
        EnableChannel(0, TRXDir::Rx, ch, rx.enabled);

        // TX channel
        const auto& tx = chanCfg.tx;
        if (tx.enabled)
        {
            if (tx.centerFrequency > 0)
                SetFrequency(0, TRXDir::Tx, ch, tx.centerFrequency);
            if (tx.sampleRate > 0)
                SetSampleRate(0, TRXDir::Tx, ch, tx.sampleRate, tx.oversample);
            if (tx.lpf > 0)
                SetLowPassFilter(0, TRXDir::Tx, ch, tx.lpf);
            for (const auto& [gainType, gainVal] : tx.gain)
                SetGain(0, TRXDir::Tx, ch, gainType, gainVal);
            SetAntenna(0, TRXDir::Tx, ch, tx.path);
        }
        EnableChannel(0, TRXDir::Tx, ch, tx.enabled);
    }

    return OpStatus::Success;
}

// -------------------------------------------------------------------------
// Init / Reset
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::Init() { return InitHardware(true); }

OpStatus LiteXM2SDR::Reset()
{
    // Hard reset: pull AD9361 reset low then re-init
    void* conn = reinterpret_cast<void*>(static_cast<intptr_t>(mFd));
    m2sdr_ad9361_spi_init(conn, 1); // reset=1
    return InitHardware(false);
}

// -------------------------------------------------------------------------
// GPS / lock status (not available on litex_m2sdr)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::GetGPSLock(GPS_Lock* status)
{
    if (status)
        *status = GPS_Lock{};
    return OpStatus::NotImplemented;
}

bool LiteXM2SDR::GetCGENLocked(uint8_t /*moduleIndex*/) { return true; }
bool LiteXM2SDR::GetSXLocked(uint8_t /*moduleIndex*/, TRXDir /*trx*/) { return true; }

// -------------------------------------------------------------------------
// EnableChannel
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    if (moduleIndex != 0 || channel >= MAX_CHANNELS)
        return OpStatus::InvalidValue;

    mChannelEnabled[channel][trx == TRXDir::Tx ? 1 : 0] = enable;

    // Determine if we're in 1T1R or 2T2R mode
    bool bothRxEnabled = mChannelEnabled[0][0] && mChannelEnabled[1][0];
    bool bothTxEnabled = mChannelEnabled[0][1] && mChannelEnabled[1][1];

    uint8_t no_ch_mode = (bothRxEnabled && bothTxEnabled) ? MODE_2x2 : MODE_1x1;
    ad9361_set_no_ch_mode(mAd9361, no_ch_mode);

    // Update PHY control register for 1T1R vs 2T2R timing
    litepcie_writel(mFd, CSR_AD9361_PHY_CONTROL_ADDR, (no_ch_mode == MODE_1x1) ? 1 : 0);

    return OpStatus::Success;
}

// -------------------------------------------------------------------------
// Frequency control
// -------------------------------------------------------------------------
double LiteXM2SDR::GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/)
{
    if (moduleIndex != 0)
        return 0.0;

    uint64_t freq = 0;
    if (trx == TRXDir::Rx)
        ad9361_get_rx_lo_freq(mAd9361, &freq);
    else
        ad9361_get_tx_lo_freq(mAd9361, &freq);
    return static_cast<double>(freq);
}

OpStatus LiteXM2SDR::SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/, double frequency)
{
    if (moduleIndex != 0)
        return OpStatus::InvalidValue;

    int ret;
    if (trx == TRXDir::Rx)
    {
        ret = ad9361_set_rx_lo_freq(mAd9361, static_cast<uint64_t>(frequency));
        if (ret == 0)
            mRxFreq = frequency;
    }
    else
    {
        ret = ad9361_set_tx_lo_freq(mAd9361, static_cast<uint64_t>(frequency));
        if (ret == 0)
            mTxFreq = frequency;
    }
    return (ret == 0) ? OpStatus::Success : OpStatus::Error;
}

// -------------------------------------------------------------------------
// Clock frequency (reference clock — not runtime-configurable here)
// -------------------------------------------------------------------------
double LiteXM2SDR::GetClockFreq(uint8_t /*clk_id*/, uint8_t /*channel*/)
{
    return static_cast<double>(DEFAULT_REFCLK_FREQ);
}

OpStatus LiteXM2SDR::SetClockFreq(uint8_t /*clk_id*/, double /*freq*/, uint8_t /*channel*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// NCO (not available on AD9361 in this integration)
// -------------------------------------------------------------------------
double LiteXM2SDR::GetNCOFrequency(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*index*/, double& phaseOffset)
{
    phaseOffset = 0.0;
    return 0.0;
}

OpStatus LiteXM2SDR::SetNCOFrequency(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*index*/, double /*frequency*/, double /*phaseOffset*/)
{
    return OpStatus::NotImplemented;
}

double LiteXM2SDR::GetNCOOffset(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/) { return 0.0; }
int LiteXM2SDR::GetNCOIndex(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/) { return 0; }
OpStatus LiteXM2SDR::SetNCOIndex(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*index*/, bool /*downconv*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Sample rate
// -------------------------------------------------------------------------
double LiteXM2SDR::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/, uint32_t* rf_samplerate)
{
    if (moduleIndex != 0)
        return 0.0;

    uint32_t rate = 0;
    if (trx == TRXDir::Rx)
        ad9361_get_rx_sampling_freq(mAd9361, &rate);
    else
        ad9361_get_tx_sampling_freq(mAd9361, &rate);

    if (rf_samplerate)
        *rf_samplerate = rate;
    return static_cast<double>(rate);
}

OpStatus LiteXM2SDR::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/, double sampleRate, uint8_t /*oversample*/)
{
    if (moduleIndex != 0)
        return OpStatus::InvalidValue;

    uint32_t rate = static_cast<uint32_t>(sampleRate);
    int ret;

    // Select FIR decimation/interpolation based on rate
    if (trx == TRXDir::Rx)
    {
        if (rate < 1250000)
        {
            mAd9361->rx_fir_dec = 4;
            ad9361_set_rx_fir_config(mAd9361, rx_fir_config_dec4);
        }
        else if (rate < 2500000)
        {
            mAd9361->rx_fir_dec = 2;
            ad9361_set_rx_fir_config(mAd9361, rx_fir_config_dec2);
        }
        else
        {
            mAd9361->rx_fir_dec = 1;
            ad9361_set_rx_fir_config(mAd9361, rx_fir_config);
        }
        ret = ad9361_set_rx_sampling_freq(mAd9361, rate);
        if (ret == 0)
            mRxSampleRate = sampleRate;
    }
    else
    {
        if (rate < 1250000)
        {
            mAd9361->tx_fir_int = 4;
            ad9361_set_tx_fir_config(mAd9361, tx_fir_config_int4);
        }
        else if (rate < 2500000)
        {
            mAd9361->tx_fir_int = 2;
            ad9361_set_tx_fir_config(mAd9361, tx_fir_config_int2);
        }
        else
        {
            mAd9361->tx_fir_int = 1;
            ad9361_set_tx_fir_config(mAd9361, tx_fir_config);
        }
        ret = ad9361_set_tx_sampling_freq(mAd9361, rate);
        if (ret == 0)
            mTxSampleRate = sampleRate;
    }

    return (ret == 0) ? OpStatus::Success : OpStatus::Error;
}

// -------------------------------------------------------------------------
// Gain control
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value)
{
    if (moduleIndex != 0 || channel >= MAX_CHANNELS)
        return OpStatus::InvalidValue;

    if (direction == TRXDir::Rx && gain == eGainTypes::LNA)
    {
        int32_t gain_db = 0;
        ad9361_get_rx_rf_gain(mAd9361, static_cast<uint8_t>(channel), &gain_db);
        value = static_cast<double>(gain_db);
        return OpStatus::Success;
    }

    if (direction == TRXDir::Tx && gain == eGainTypes::PAD)
    {
        uint32_t atten_mdb = 0;
        ad9361_get_tx_attenuation(mAd9361, static_cast<uint8_t>(channel), &atten_mdb);
        // Convert milli-dB attenuation to gain (negative = attenuated)
        value = -static_cast<double>(atten_mdb) / 1000.0;
        return OpStatus::Success;
    }

    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value)
{
    if (moduleIndex != 0 || channel >= MAX_CHANNELS)
        return OpStatus::InvalidValue;

    if (direction == TRXDir::Rx && gain == eGainTypes::LNA)
    {
        double clamped = std::max(0.0, std::min(73.0, value));
        int ret = ad9361_set_rx_rf_gain(mAd9361, static_cast<uint8_t>(channel), static_cast<int32_t>(clamped));
        if (ret == 0)
            mRxGain[channel] = clamped;
        return (ret == 0) ? OpStatus::Success : OpStatus::Error;
    }

    if (direction == TRXDir::Tx && gain == eGainTypes::PAD)
    {
        // value is in dB (negative = attenuation), e.g., value=-20 means 20 dB atten
        double atten_db = std::max(0.0, std::min(89.0, -value));
        uint32_t atten_mdb = static_cast<uint32_t>(atten_db * 1000.0);
        int ret = ad9361_set_tx_attenuation(mAd9361, static_cast<uint8_t>(channel), atten_mdb);
        if (ret == 0)
            mTxAtten[channel] = atten_mdb;
        return (ret == 0) ? OpStatus::Success : OpStatus::Error;
    }

    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Low-pass filter (analog RF bandwidth)
// -------------------------------------------------------------------------
double LiteXM2SDR::GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/)
{
    if (moduleIndex != 0)
        return 0.0;

    uint32_t bw = 0;
    if (trx == TRXDir::Rx)
        ad9361_get_rx_rf_bandwidth(mAd9361, &bw);
    else
        ad9361_get_tx_rf_bandwidth(mAd9361, &bw);
    return static_cast<double>(bw);
}

OpStatus LiteXM2SDR::SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t /*channel*/, double lpf)
{
    if (moduleIndex != 0)
        return OpStatus::InvalidValue;

    int ret;
    if (trx == TRXDir::Rx)
    {
        ret = ad9361_set_rx_rf_bandwidth(mAd9361, static_cast<uint32_t>(lpf));
        if (ret == 0)
            mRxBandwidth = lpf;
    }
    else
    {
        ret = ad9361_set_tx_rf_bandwidth(mAd9361, static_cast<uint32_t>(lpf));
        if (ret == 0)
            mTxBandwidth = lpf;
    }
    return (ret == 0) ? OpStatus::Success : OpStatus::Error;
}

// -------------------------------------------------------------------------
// Antenna / RF port selection
// -------------------------------------------------------------------------
uint8_t LiteXM2SDR::GetAntenna(uint8_t /*moduleIndex*/, TRXDir trx, uint8_t channel)
{
    return (trx == TRXDir::Rx) ? mRxAntenna[channel] : mTxAntenna[channel];
}

OpStatus LiteXM2SDR::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    if (moduleIndex != 0 || channel >= MAX_CHANNELS)
        return OpStatus::InvalidValue;

    if (trx == TRXDir::Rx)
    {
        // AD9361 RX port: A_BALANCED=0, B_BALANCED=1, C_BALANCED=2
        int ret = ad9361_set_rx_rf_port_input(mAd9361, static_cast<uint32_t>(path));
        if (ret == 0)
            mRxAntenna[channel] = path;
        return (ret == 0) ? OpStatus::Success : OpStatus::Error;
    }
    else
    {
        // AD9361 TX port: TXA=0, TXB=1
        int ret = ad9361_set_tx_rf_port_output(mAd9361, static_cast<uint32_t>(path));
        if (ret == 0)
            mTxAntenna[channel] = path;
        return (ret == 0) ? OpStatus::Success : OpStatus::Error;
    }
}

// -------------------------------------------------------------------------
// Test signal (not implemented)
// -------------------------------------------------------------------------
ChannelConfig::Direction::TestSignal LiteXM2SDR::GetTestSignal(
    uint8_t /*moduleIndex*/, TRXDir /*direction*/, uint8_t /*channel*/)
{
    return ChannelConfig::Direction::TestSignal{};
}

OpStatus LiteXM2SDR::SetTestSignal(uint8_t /*moduleIndex*/,
    TRXDir /*direction*/,
    uint8_t /*channel*/,
    ChannelConfig::Direction::TestSignal /*signalConfiguration*/,
    int16_t /*dc_i*/,
    int16_t /*dc_q*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// DC offset (AD9361 has built-in tracking; manual override not implemented)
// -------------------------------------------------------------------------
bool LiteXM2SDR::GetDCOffsetMode(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/) { return true; }

OpStatus LiteXM2SDR::SetDCOffsetMode(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, bool /*isAutomatic*/)
{
    return OpStatus::NotImplemented;
}

complex64f_t LiteXM2SDR::GetDCOffset(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/)
{
    return { 0.0, 0.0 };
}

OpStatus LiteXM2SDR::SetDCOffset(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, const complex64f_t& /*offset*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// IQ balance (AD9361 has built-in calibration)
// -------------------------------------------------------------------------
complex64f_t LiteXM2SDR::GetIQBalance(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/)
{
    return { 1.0, 0.0 };
}

OpStatus LiteXM2SDR::SetIQBalance(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, const complex64f_t& /*balance*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Temperature (from XADC)
// -------------------------------------------------------------------------
double LiteXM2SDR::GetTemperature(uint8_t /*moduleIndex*/)
{
#ifdef CSR_XADC_TEMPERATURE_ADDR
    uint32_t raw = litepcie_readl(mFd, CSR_XADC_TEMPERATURE_ADDR);
    // XADC temperature conversion: T = (raw * 503.975 / 4096) - 273.15
    double temp_c = (static_cast<double>(raw) * 503.975 / 4096.0) - 273.15;
    return temp_c;
#else
    return 0.0;
#endif
}

// -------------------------------------------------------------------------
// Direct register access
// -------------------------------------------------------------------------
unsigned int LiteXM2SDR::ReadRegister(uint8_t /*moduleIndex*/, unsigned int address, bool useFPGA)
{
    if (useFPGA)
        return litepcie_readl(mFd, static_cast<uint32_t>(address));
    // AD9361 register read
    return m2sdr_ad9361_spi_read(reinterpret_cast<void*>(static_cast<intptr_t>(mFd)),
        static_cast<uint16_t>(address));
}

OpStatus LiteXM2SDR::WriteRegister(uint8_t /*moduleIndex*/, unsigned int address, unsigned int value, bool useFPGA)
{
    if (useFPGA)
        litepcie_writel(mFd, static_cast<uint32_t>(address), static_cast<uint32_t>(value));
    else
        m2sdr_ad9361_spi_write(reinterpret_cast<void*>(static_cast<intptr_t>(mFd)),
            static_cast<uint16_t>(address),
            static_cast<uint8_t>(value));
    return OpStatus::Success;
}

// -------------------------------------------------------------------------
// Config file load/save (not implemented)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::LoadConfig(uint8_t /*moduleIndex*/, const std::string& /*filename*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::SaveConfig(uint8_t /*moduleIndex*/, const std::string& /*filename*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Parameter get/set (bitfield access — mapped to AD9361 registers)
// -------------------------------------------------------------------------
uint16_t LiteXM2SDR::GetParameter(uint8_t moduleIndex, uint8_t /*channel*/, const std::string& /*parameterKey*/)
{
    return 0;
}

OpStatus LiteXM2SDR::SetParameter(
    uint8_t moduleIndex, uint8_t /*channel*/, const std::string& /*parameterKey*/, uint16_t /*value*/)
{
    return OpStatus::NotImplemented;
}

uint16_t LiteXM2SDR::GetParameter(
    uint8_t /*moduleIndex*/, uint8_t /*channel*/, uint16_t address, uint8_t msb, uint8_t lsb)
{
    uint8_t regVal =
        m2sdr_ad9361_spi_read(reinterpret_cast<void*>(static_cast<intptr_t>(mFd)), address);
    uint8_t mask = static_cast<uint8_t>((1 << (msb - lsb + 1)) - 1);
    return static_cast<uint16_t>((regVal >> lsb) & mask);
}

OpStatus LiteXM2SDR::SetParameter(
    uint8_t /*moduleIndex*/, uint8_t /*channel*/, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    void* conn = reinterpret_cast<void*>(static_cast<intptr_t>(mFd));
    uint8_t regVal = m2sdr_ad9361_spi_read(conn, address);
    uint8_t mask = static_cast<uint8_t>((1 << (msb - lsb + 1)) - 1);
    regVal = static_cast<uint8_t>((regVal & ~(mask << lsb)) | ((value & mask) << lsb));
    m2sdr_ad9361_spi_write(conn, address, regVal);
    return OpStatus::Success;
}

// -------------------------------------------------------------------------
// Synchronize / cache (no register cache on AD9361 in this driver)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::Synchronize(bool /*toChip*/) { return OpStatus::Success; }
void LiteXM2SDR::EnableCache(bool /*enable*/) {}

// -------------------------------------------------------------------------
// GFIR filters (replaced by AD9361 digital FIR — not exposed separately)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::ConfigureGFIR(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, ChannelConfig::Direction::GFIRFilter /*settings*/)
{
    return OpStatus::NotImplemented;
}

std::vector<double> LiteXM2SDR::GetGFIRCoefficients(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*gfirID*/)
{
    return {};
}

OpStatus LiteXM2SDR::SetGFIRCoefficients(
    uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*gfirID*/, std::vector<double> /*coefficients*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::SetGFIR(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, uint8_t /*gfirID*/, bool /*enabled*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Hardware timestamp (nanoseconds from time generator CSR)
// -------------------------------------------------------------------------
uint64_t LiteXM2SDR::GetHardwareTimestamp(uint8_t /*moduleIndex*/)
{
#ifdef CSR_TIME_GEN_CONTROL_ADDR
    // Latch current time
    litepcie_writel(mFd, CSR_TIME_GEN_CONTROL_ADDR, 1 << CSR_TIME_GEN_CONTROL_READ_OFFSET);
    uint64_t hi = litepcie_readl(mFd, CSR_TIME_GEN_READ_TIME_ADDR);
    uint64_t lo = litepcie_readl(mFd, CSR_TIME_GEN_READ_TIME_ADDR + 4);
    return (hi << 32) | lo;
#else
    return 0;
#endif
}

OpStatus LiteXM2SDR::SetHardwareTimestamp(uint8_t /*moduleIndex*/, const uint64_t now)
{
#ifdef CSR_TIME_GEN_CONTROL_ADDR
    litepcie_writel(mFd, CSR_TIME_GEN_WRITE_TIME_ADDR, static_cast<uint32_t>(now >> 32));
    litepcie_writel(mFd, CSR_TIME_GEN_WRITE_TIME_ADDR + 4, static_cast<uint32_t>(now & 0xFFFFFFFF));
    litepcie_writel(mFd, CSR_TIME_GEN_CONTROL_ADDR, 1 << CSR_TIME_GEN_CONTROL_WRITE_OFFSET);
    return OpStatus::Success;
#else
    return OpStatus::NotImplemented;
#endif
}

// -------------------------------------------------------------------------
// Memory operations (not implemented in initial version)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::UploadMemory(
    eMemoryDevice /*device*/, uint8_t /*moduleIndex*/, const char* /*data*/, size_t /*length*/, UploadMemoryCallback /*callback*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::MemoryWrite(std::shared_ptr<DataStorage> /*storage*/, Region /*region*/, const void* /*data*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::MemoryRead(std::shared_ptr<DataStorage> /*storage*/, Region /*region*/, void* /*data*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// GPIO
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;
#ifdef CSR_GPIO_OUT_ADDR
    uint32_t val = 0;
    for (size_t i = 0; i < std::min(bufLength, size_t(4)); ++i)
        val |= static_cast<uint32_t>(buffer[i]) << (i * 8);
    litepcie_writel(mFd, CSR_GPIO_OUT_ADDR, val);
    return OpStatus::Success;
#else
    return OpStatus::NotImplemented;
#endif
}

OpStatus LiteXM2SDR::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;
#ifdef CSR_GPIO_IN_ADDR
    uint32_t val = litepcie_readl(mFd, CSR_GPIO_IN_ADDR);
    for (size_t i = 0; i < std::min(bufLength, size_t(4)); ++i)
        buffer[i] = static_cast<uint8_t>((val >> (i * 8)) & 0xFF);
    return OpStatus::Success;
#else
    return OpStatus::NotImplemented;
#endif
}

OpStatus LiteXM2SDR::GPIODirWrite(const uint8_t* /*buffer*/, const size_t /*bufLength*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::GPIODirRead(uint8_t* /*buffer*/, const size_t /*bufLength*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// Custom parameters (not exposed in initial version)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::CustomParameterWrite(const std::vector<CustomParameterIO>& /*parameters*/)
{
    return OpStatus::NotImplemented;
}

OpStatus LiteXM2SDR::CustomParameterRead(std::vector<CustomParameterIO>& /*parameters*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// StreamCreate
// -------------------------------------------------------------------------
std::unique_ptr<lime::RFStream> LiteXM2SDR::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (moduleIndex != 0)
        return nullptr;

    auto stream = std::make_unique<LiteXM2SDRStreaming>(mFd, config);
    OpStatus status = stream->Setup(config);
    if (status != OpStatus::Success)
        return nullptr;
    return stream;
}

// -------------------------------------------------------------------------
// Calibrate (AD9361 has internal auto-calibration; not exposed here)
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::Calibrate(uint8_t /*moduleIndex*/, TRXDir /*trx*/, uint8_t /*channel*/, double /*bandwidth*/)
{
    return OpStatus::NotImplemented;
}

// -------------------------------------------------------------------------
// GetInternalChip (no LMS7002M or equivalent exposed)
// -------------------------------------------------------------------------
void* LiteXM2SDR::GetInternalChip(uint32_t /*index*/) { return nullptr; }

// -------------------------------------------------------------------------
// Deprecated streaming API — delegates to mStreamer
// -------------------------------------------------------------------------
OpStatus LiteXM2SDR::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    mStreamer = StreamCreate(config, moduleIndex);
    return mStreamer ? OpStatus::Success : OpStatus::Error;
}

void LiteXM2SDR::StreamStart(uint8_t /*moduleIndex*/)
{
    if (mStreamer)
        mStreamer->Start();
}

void LiteXM2SDR::StreamStop(uint8_t /*moduleIndex*/)
{
    if (mStreamer)
        mStreamer->Stop();
}

void LiteXM2SDR::StreamDestroy(uint8_t /*moduleIndex*/)
{
    mStreamer.reset();
}

uint32_t LiteXM2SDR::StreamRx(
    uint8_t /*moduleIndex*/, complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamRx(samples, count, meta, timeout);
}

uint32_t LiteXM2SDR::StreamRx(
    uint8_t /*moduleIndex*/, complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamRx(samples, count, meta, timeout);
}

uint32_t LiteXM2SDR::StreamRx(
    uint8_t /*moduleIndex*/, complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamRx(samples, count, meta, timeout);
}

uint32_t LiteXM2SDR::StreamTx(uint8_t /*moduleIndex*/,
    const complex32f_t* const* samples,
    uint32_t count,
    const StreamMeta* meta,
    std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamTx(samples, count, meta, timeout);
}

uint32_t LiteXM2SDR::StreamTx(uint8_t /*moduleIndex*/,
    const complex16_t* const* samples,
    uint32_t count,
    const StreamMeta* meta,
    std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamTx(samples, count, meta, timeout);
}

uint32_t LiteXM2SDR::StreamTx(uint8_t /*moduleIndex*/,
    const complex12_t* const* samples,
    uint32_t count,
    const StreamMeta* meta,
    std::chrono::microseconds timeout)
{
    if (!mStreamer)
        return 0;
    return mStreamer->StreamTx(samples, count, meta, timeout);
}

void LiteXM2SDR::StreamStatus(uint8_t /*moduleIndex*/, StreamStats* rx, StreamStats* tx)
{
    if (mStreamer)
        mStreamer->StreamStatus(rx, tx);
}

} // namespace lime
