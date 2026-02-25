/*
 * M2SDR board support for LimeSuiteNG.
 *
 * Copyright (c) 2024-2026 Lime Microsystems
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "M2SDR.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <unistd.h>

#include "limesuiteng/Logger.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/ToString.h"

#include "chips/AD9361/AD9361.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"
#include "comms/SPI/ISPI.h"
#include "comms/ISerialPort.h"
#include "FPGA/FPGA_common.h"
#include "protocols/LMS64C/SPI.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"
#include "DeviceTreeNode.h"
#include "CommonFunctions.h"

using namespace std::literals::string_literals;

namespace lime {

/* --------------------------------------------------------------------------
 * Null SPI: backed by nothing; used for the stub LMS7002M in mLMSChips[0].
 * TRXLooper reads LML1_TRXIQPULSE / LML1_SISODDR from cached register map
 * (always 0 = disabled / MIMO LVDS) – correct for the AD9361.
 * -------------------------------------------------------------------------- */
class NullSPI : public ISPI
{
  public:
    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override
    {
        if (MISO)
            std::fill(MISO, MISO + count, 0u);
        return OpStatus::Success;
    }
};

/* --------------------------------------------------------------------------
 * M2SDR CSR addresses  (from LimeSDR_GW csr.csv)
 * -------------------------------------------------------------------------- */
static constexpr uint32_t I2C_W_ADDR = 0xf0009000; /* i2c0_w: [0]=SCL,[1]=SDA,[2]=SCL_OE,[3]=SDA_OE */
static constexpr uint32_t I2C_R_ADDR = 0xf0009004; /* i2c0_r: [0]=SCL_IN,[1]=SDA_IN                 */

static constexpr uint8_t I2C_SCL     = (1 << 0);
static constexpr uint8_t I2C_SDA     = (1 << 1);
static constexpr uint8_t I2C_SCL_OE  = (1 << 2);
static constexpr uint8_t I2C_SDA_OE  = (1 << 3);

/* Si5351C I2C address and default config */
static constexpr uint8_t SI5351_I2C_ADDR = 0x60;

/* 38.4 MHz reference from 25 MHz XTAL on Si5351B/C */
static const uint8_t si5351_xo_38p4m_config[][2] = {
    { 0x02, 0x33 }, { 0x03, 0x00 }, { 0x04, 0x10 }, { 0x07, 0x01 },
    { 0x0F, 0x00 },
    { 0x10, 0x2D }, { 0x11, 0x2D }, { 0x12, 0x8C }, { 0x13, 0x8C },
    { 0x14, 0x2D }, { 0x15, 0x8C }, { 0x16, 0x8C }, { 0x17, 0x8C },
    { 0x22, 0x42 }, { 0x23, 0x40 }, { 0x24, 0x00 }, { 0x25, 0x0E },
    { 0x26, 0xE5 }, { 0x27, 0xF5 }, { 0x28, 0xBC }, { 0x29, 0xC0 },
    { 0x2A, 0x00 }, { 0x2B, 0x01 }, { 0x2C, 0x00 }, { 0x2D, 0x09 },
    { 0x2E, 0x00 }, { 0x2F, 0x00 }, { 0x30, 0x00 }, { 0x31, 0x00 },
    { 0x32, 0x00 }, { 0x33, 0x7D }, { 0x34, 0x00 }, { 0x35, 0x02 },
    { 0x36, 0x39 }, { 0x37, 0x00 }, { 0x38, 0x00 }, { 0x39, 0x2B },
    { 0x4A, 0x00 }, { 0x4B, 0x01 }, { 0x4C, 0x00 }, { 0x4D, 0x09 },
    { 0x4E, 0x00 }, { 0x4F, 0x00 }, { 0x50, 0x00 }, { 0x51, 0x00 },
    { 0x95, 0x00 }, { 0x96, 0x00 }, { 0x97, 0x00 }, { 0x98, 0x00 },
    { 0x99, 0x00 }, { 0x9A, 0x00 }, { 0x9B, 0x00 },
    { 0xA2, 0xF2 }, { 0xA3, 0xFD }, { 0xA4, 0x01 },
    { 0xA5, 0x00 }, { 0xA6, 0x00 }, { 0xA7, 0x00 },
    { 0xA8, 0x00 }, { 0xA9, 0x00 }, { 0xAA, 0x00 },
    { 0xB7, 0x12 },
};
static constexpr size_t SI5351_CFG_LEN =
    sizeof(si5351_xo_38p4m_config) / sizeof(si5351_xo_38p4m_config[0]);

/* ---------- removed: default_ad9361_init moved to AD9361.cpp ---------- */
#if 0
static AD9361_InitParam default_ad9361_init = {
    /* Device selection */
    ID_AD9361,
    /* id_no */ 0,
    /* reference_clk_rate */ 38400000LL,
    /* two_rx_two_tx_mode_enable */ 1,
    /* one_rx_one_tx_mode_use_rx_num */ 1,
    /* one_rx_one_tx_mode_use_tx_num */ 1,
    /* frequency_division_duplex_mode_enable */ 1,
    /* frequency_division_duplex_independent_mode_enable */ 0,
    /* tdd_use_dual_synth_mode_enable */ 0,
    /* tdd_skip_vco_cal_enable */ 0,
    /* tx_fastlock_delay_ns */ 0,
    /* rx_fastlock_delay_ns */ 0,
    /* rx_fastlock_pincontrol_enable */ 0,
    /* tx_fastlock_pincontrol_enable */ 0,
    /* external_rx_lo_enable */ 0,
    /* external_tx_lo_enable */ 0,
    /* dc_offset_tracking_update_event_mask */ 5,
    /* dc_offset_attenuation_high_range */ 6,
    /* dc_offset_attenuation_low_range */ 5,
    /* dc_offset_count_high_range */ 0x28,
    /* dc_offset_count_low_range */ 0x32,
    /* split_gain_table_mode_enable */ 0,
    /* trx_synthesizer_target_fref_overwrite_hz */ MAX_SYNTH_FREF,
    /* qec_tracking_slow_mode_enable */ 0,
    /* ensm_enable_pin_pulse_mode_enable */ 0,
    /* ensm_enable_txnrx_control_enable */ 0,
    /* rx_synthesizer_frequency_hz */ 2400000000LL,
    /* tx_synthesizer_frequency_hz */ 2400000000LL,
    /* tx_lo_powerdown_managed_enable */ 1,
    /* rx_path_clock_frequencies */ { 983040000, 245760000, 122880000, 61440000, 30720000, 30720000 },
    /* tx_path_clock_frequencies */ { 983040000, 122880000, 122880000, 61440000, 30720000, 30720000 },
    /* rf_rx_bandwidth_hz */ 18000000,
    /* rf_tx_bandwidth_hz */ 18000000,
    /* rx_rf_port_input_select */ 0,
    /* tx_rf_port_input_select */ 0,
    /* tx_attenuation_mdB */ 10000,
    /* update_tx_gain_in_alert_enable */ 0,
    /* xo_disable_use_ext_refclk_enable */ 0,
    /* dcxo_coarse_and_fine_tune */ { 8, 5920 },
    /* clk_output_mode_select */ CLKOUT_DISABLE,
    /* gc_rx1_mode */ 2, /* AGC */
    /* gc_rx2_mode */ 2,
    /* gc_adc_large_overload_thresh */ 58,
    /* gc_adc_ovr_sample_size */ 4,
    /* gc_adc_small_overload_thresh */ 47,
    /* gc_dec_pow_measurement_duration */ 8192,
    /* gc_dig_gain_enable */ 0,
    /* gc_lmt_overload_high_thresh */ 800,
    /* gc_lmt_overload_low_thresh */ 704,
    /* gc_low_power_thresh */ 24,
    /* gc_max_dig_gain */ 15,
    /* mgc_dec_gain_step */ 2,
    /* mgc_inc_gain_step */ 2,
    /* mgc_rx1_ctrl_inp_enable */ 0,
    /* mgc_rx2_ctrl_inp_enable */ 0,
    /* mgc_split_table_ctrl_inp_gain_mode */ 0,
    /* agc_adc_large_overload_exceed_counter */ 10,
    /* agc_adc_large_overload_inc_steps */ 2,
    /* agc_adc_lmt_small_overload_prevent_gain_inc_enable */ 0,
    /* agc_adc_small_overload_exceed_counter */ 10,
    /* agc_dig_gain_step_size */ 4,
    /* agc_dig_saturation_exceed_counter */ 3,
    /* agc_gain_update_interval_us */ 1000,
    /* agc_immed_gain_change_if_large_adc_overload_enable */ 0,
    /* agc_immed_gain_change_if_large_lmt_overload_enable */ 0,
    /* agc_inner_thresh_high */ 10,
    /* agc_inner_thresh_high_dec_steps */ 1,
    /* agc_inner_thresh_low */ 12,
    /* agc_inner_thresh_low_inc_steps */ 1,
    /* agc_lmt_overload_large_exceed_counter */ 10,
    /* agc_lmt_overload_large_inc_steps */ 2,
    /* agc_lmt_overload_small_exceed_counter */ 10,
    /* agc_outer_thresh_high */ 5,
    /* agc_outer_thresh_high_dec_steps */ 2,
    /* agc_outer_thresh_low */ 18,
    /* agc_outer_thresh_low_inc_steps */ 2,
    /* agc_attack_delay_extra_margin_us */ 1,
    /* agc_sync_for_gain_counter_enable */ 0,
    /* fagc_dec_pow_measuremnt_duration */ 64,
    /* fagc_state_wait_time_ns */ 260,
    /* fagc_allow_agc_gain_increase */ 0,
    /* fagc_lp_thresh_increment_time */ 5,
    /* fagc_lp_thresh_increment_steps */ 1,
    /* fagc_lock_level_lmt_gain_increase_en */ 1,
    /* fagc_lock_level_gain_increase_upper_limit */ 5,
    /* fagc_lpf_final_settling_steps */ 1,
    /* fagc_lmt_final_settling_steps */ 1,
    /* fagc_final_overrange_count */ 3,
    /* fagc_gain_increase_after_gain_lock_en */ 0,
    /* fagc_gain_index_type_after_exit_rx_mode */ 0,
    /* fagc_use_last_lock_level_for_set_gain_en */ 1,
    /* fagc_rst_gla_stronger_sig_thresh_exceeded_en */ 1,
    /* fagc_optimized_gain_offset */ 5,
    /* fagc_rst_gla_stronger_sig_thresh_above_ll */ 10,
    /* fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en */ 1,
    /* fagc_rst_gla_engergy_lost_goto_optim_gain_en */ 1,
    /* fagc_rst_gla_engergy_lost_sig_thresh_below_ll */ 10,
    /* fagc_energy_lost_stronger_sig_gain_lock_exit_cnt */ 8,
    /* fagc_rst_gla_large_adc_overload_en */ 1,
    /* fagc_rst_gla_large_lmt_overload_en */ 1,
    /* fagc_rst_gla_en_agc_pulled_high_en */ 0,
    /* fagc_rst_gla_if_en_agc_pulled_high_mode */ 0,
    /* fagc_power_measurement_duration_in_state5 */ 64,
    /* rssi_delay */ 1,
    /* rssi_duration */ 1000,
    /* rssi_restart_mode */ 3,
    /* rssi_unit_is_rx_samples_enable */ 0,
    /* rssi_wait */ 1,
    /* aux_adc_decimation */ 256,
    /* aux_adc_rate */ 40000000UL,
    /* aux_dac_manual_mode_enable */ 1,
    /* aux_dac1_default_value_mV */ 0,
    /* aux_dac1_active_in_rx_enable */ 0,
    /* aux_dac1_active_in_tx_enable */ 0,
    /* aux_dac1_active_in_alert_enable */ 0,
    /* aux_dac1_rx_delay_us */ 0,
    /* aux_dac1_tx_delay_us */ 0,
    /* aux_dac2_default_value_mV */ 0,
    /* aux_dac2_active_in_rx_enable */ 0,
    /* aux_dac2_active_in_tx_enable */ 0,
    /* aux_dac2_active_in_alert_enable */ 0,
    /* aux_dac2_rx_delay_us */ 0,
    /* aux_dac2_tx_delay_us */ 0,
    /* temp_sense_decimation */ 256,
    /* temp_sense_measurement_interval_ms */ 1000,
    /* temp_sense_offset_signed */ (int8_t)0xCE,
    /* temp_sense_periodic_measurement_enable */ 1,
    /* ctrl_outs_enable_mask */ 0xFF,
    /* ctrl_outs_index */ 0,
    /* elna_settling_delay_ns */ 0,
    /* elna_gain_mdB */ 0,
    /* elna_bypass_loss_mdB */ 0,
    /* elna_rx1_gpo0_control_enable */ 0,
    /* elna_rx2_gpo1_control_enable */ 0,
    /* elna_gaintable_all_index_enable */ 0,
    /* digital_interface_tune_skip_mode */ 0,
    /* digital_interface_tune_fir_disable */ 0,
    /* pp_tx_swap_enable */ 1,
    /* pp_rx_swap_enable */ 1,
    /* tx_channel_swap_enable */ 0,
    /* rx_channel_swap_enable */ 0,
    /* rx_frame_pulse_mode_enable */ 1,
    /* two_t_two_r_timing_enable */ 1,
    /* invert_data_bus_enable */ 0,
    /* invert_data_clk_enable */ 0,
    /* fdd_alt_word_order_enable */ 0,
    /* invert_rx_frame_enable */ 0,
    /* fdd_rx_rate_2tx_enable */ 0,
    /* swap_ports_enable */ 0,
    /* single_data_rate_enable */ 0,
    /* lvds_mode_enable */ 1,
    /* half_duplex_mode_enable */ 0,
    /* single_port_mode_enable */ 0,
    /* full_port_enable */ 0,
    /* full_duplex_swap_bits_enable */ 0,
    /* delay_rx_data */ 0,
    /* rx_data_clock_delay */ 2, /* RX_CLK_DELAY */
    /* rx_data_delay */ 3,       /* RX_DAT_DELAY */
    /* tx_fb_clock_delay */ 0,   /* TX_CLK_DELAY */
    /* tx_data_delay */ 5,       /* TX_DAT_DELAY */
    /* lvds_bias_mV */ 150,
    /* lvds_rx_onchip_termination_enable */ 1,
    /* rx1rx2_phase_inversion_en */ 0,
    /* lvds_invert1_control */ 0xFF,
    /* lvds_invert2_control */ 0x0F,
    /* gpo0_inactive_state_high_enable */ 0,
    /* gpo1_inactive_state_high_enable */ 0,
    /* gpo2_inactive_state_high_enable */ 0,
    /* gpo3_inactive_state_high_enable */ 0,
    /* gpo0_slave_rx_enable */ 0,
    /* gpo0_slave_tx_enable */ 0,
    /* gpo1_slave_rx_enable */ 0,
    /* gpo1_slave_tx_enable */ 0,
    /* gpo2_slave_rx_enable */ 0,
    /* gpo2_slave_tx_enable */ 0,
    /* gpo3_slave_rx_enable */ 0,
    /* gpo3_slave_tx_enable */ 0,
    /* gpo0_rx_delay_us */ 0,
    /* gpo0_tx_delay_us */ 0,
    /* gpo1_rx_delay_us */ 0,
    /* gpo1_tx_delay_us */ 0,
    /* gpo2_rx_delay_us */ 0,
    /* gpo2_tx_delay_us */ 0,
    /* gpo3_rx_delay_us */ 0,
    /* gpo3_tx_delay_us */ 0,
    /* low_high_gain_threshold_mdB */ 37000,
    /* low_gain_dB */ 0,
    /* high_gain_dB */ 24,
    /* tx_mon_track_en */ 0,
    /* one_shot_mode_en */ 0,
    /* tx_mon_delay */ 511,
    /* tx_mon_duration */ 8192,
    /* tx1_mon_front_end_gain */ 2,
    /* tx2_mon_front_end_gain */ 2,
    /* tx1_mon_lo_cm */ 48,
    /* tx2_mon_lo_cm */ 48,
    /* gpio_resetb */ -1,
    /* gpio_sync */ -1,
    /* gpio_cal_sw1 */ -1,
    /* gpio_cal_sw2 */ -1,
    /* ad9361_rfpll_ext_recalc_rate */ nullptr,
    /* ad9361_rfpll_ext_round_rate */ nullptr,
    /* ad9361_rfpll_ext_set_rate */ nullptr,
};
#endif

/* ========================================================================== */

M2SDR::M2SDR(std::shared_ptr<LimePCIe> controlPort, std::shared_ptr<LimePCIe> streamPort)
    : mControlPort(controlPort)
    , mStreamPort(streamPort)
    , mSampleRate(30720000.0)
    , mRxBandwidth(18e6)
    , mTxBandwidth(18e6)
{
    mRxFreq[0] = mRxFreq[1] = 2400e6;
    mTxFreq[0] = mTxFreq[1] = 2400e6;

    /* Control pipe via LMS64C protocol */
    mSerialPort = std::make_shared<PCIE_CSR_Pipe>(controlPort);

    /* FPGA object: uses BRDSPI_WR/RD routes.
     * The firmware's regremap.c translates these 16-bit "SPI" addresses to
     * the correct LiteX CSR addresses (0x0007→ch_en, 0x0008→reg08, 0x000A→reg10, …) */
    auto fpgaSPI = std::make_shared<LMS64C_SPI>(
        mSerialPort, LMS64CProtocol::Command::BRDSPI_WR, LMS64CProtocol::Command::BRDSPI_RD, 0, 0);
    auto nullSPI  = std::make_shared<NullSPI>();
    mFPGA = std::make_unique<FPGA>(fpgaSPI, nullSPI);

    /* Stub LMS7002M backed by NullSPI.
     * TRXLooper reads LML1_TRXIQPULSE/LML1_SISODDR from cached register map
     * which defaults to 0 — correct for AD9361 MIMO LVDS. */
    mLMSChips.emplace_back(std::make_unique<LMS7002M>(nullSPI));

    /* One streaming slot */
    mStreamers.resize(1);

    mAD9361 = std::make_unique<AD9361>();

    BuildDescriptor();
}

M2SDR::~M2SDR() = default;

/* --------------------------------------------------------------------------
 * Descriptor
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::BuildDescriptor()
{
    mDeviceDescriptor.name = "M2SDR"s;
    mDeviceDescriptor.expansionName = ""s;

    RFSOCDescriptor soc;
    soc.name = "AD9361"s;
    soc.channelCount = 2;

    soc.pathNames[TRXDir::Rx] = { "RX"s };
    soc.pathNames[TRXDir::Tx] = { "TX"s };

    /* AD9361 freq range: 70 MHz – 6 GHz RX, 47 MHz – 6 GHz TX */
    soc.frequencyRange       = { 70e6, 6000e6, 0 };
    soc.samplingRateRange    = { 2.083e6, 61.44e6, 0 };

    soc.lowPassFilterRange[TRXDir::Rx] = { 200e3, 56e6, 0 };
    soc.lowPassFilterRange[TRXDir::Tx] = { 1.25e6, 40e6, 0 };

    /* AD9361 gain: RX 0–76 dB (UNKNOWN maps to full-range gain) */
    soc.gains[TRXDir::Rx].insert(eGainTypes::UNKNOWN);
    soc.gains[TRXDir::Tx].insert(eGainTypes::UNKNOWN);
    soc.gainRange[TRXDir::Rx][eGainTypes::UNKNOWN] = { 0.0, 76.0 };
    /* TX attenuation: 0 – 89.75 dB, we represent it as a negative gain */
    soc.gainRange[TRXDir::Tx][eGainTypes::UNKNOWN] = { -89.75, 0.0 };

    mDeviceDescriptor.rfSOC.push_back(soc);

    mDeviceDescriptor.socTree = std::make_shared<DeviceTreeNode>(this, "SDRDevice"s, "M2SDR"s);
    auto fpgaNode = std::make_shared<DeviceTreeNode>(mFPGA.get(), "FPGA_AD9361"s, "FPGA"s);
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>(mAD9361.get(), "AD9361"s));
    mDeviceDescriptor.socTree->children.push_back(fpgaNode);

    return OpStatus::Success;
}

/* --------------------------------------------------------------------------
 * Bitbang I2C helpers (LiteX i2c0_w / i2c0_r)
 *
 * i2c0_w bit layout: [0]=SCL_OUT [1]=SDA_OUT [2]=SCL_OE [3]=SDA_OE
 * i2c0_r bit layout: [0]=SCL_IN  [1]=SDA_IN
 * -------------------------------------------------------------------------- */
static uint8_t g_i2c_w = 0;

static inline void i2c_write(ISerialPort& port, uint8_t val)
{
    g_i2c_w = val;
    LMS64CProtocol::CSRegIoWrite(port, I2C_W_ADDR, val);
    usleep(5); /* ~100 kHz */
}

static inline uint8_t i2c_read(ISerialPort& port)
{
    OpStatus st;
    return static_cast<uint8_t>(LMS64CProtocol::CSRegIoRead(port, I2C_R_ADDR, &st));
}

void M2SDR::I2C_Start()
{
    /* SCL=1 SDA=1 (both driven high) */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | I2C_SCL | I2C_SDA);
    usleep(5);
    /* SDA=0 while SCL=1 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | I2C_SCL);
    usleep(5);
    /* SCL=0 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE);
}

void M2SDR::I2C_Stop()
{
    /* SCL=0 SDA=0 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE);
    usleep(5);
    /* SCL=1 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | I2C_SCL);
    usleep(5);
    /* SDA=1 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | I2C_SCL | I2C_SDA);
    usleep(5);
}

void M2SDR::I2C_WriteBit(bool bit)
{
    /* Set SDA while SCL=0 */
    uint8_t sda = bit ? I2C_SDA : 0;
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | sda);
    usleep(2);
    /* SCL=1 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | I2C_SCL | sda);
    usleep(5);
    /* SCL=0 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SDA_OE | sda);
    usleep(2);
}

bool M2SDR::I2C_ReadBit()
{
    /* Release SDA (SDA_OE=0), SCL=0 */
    i2c_write(*mSerialPort, I2C_SCL_OE);
    usleep(2);
    /* SCL=1 */
    i2c_write(*mSerialPort, I2C_SCL_OE | I2C_SCL);
    usleep(5);
    uint8_t r = i2c_read(*mSerialPort);
    bool bit = (r & I2C_SDA) != 0;
    /* SCL=0 */
    i2c_write(*mSerialPort, I2C_SCL_OE);
    usleep(2);
    return bit;
}

/* Returns true if ACK received (SDA low on 9th clock) */
bool M2SDR::I2C_WriteByte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
        I2C_WriteBit((byte >> i) & 1);
    bool nack = I2C_ReadBit(); /* ACK if low */
    return !nack;
}

bool M2SDR::Si5351_Write(uint8_t i2cAddr, uint8_t regAddr, uint8_t data)
{
    I2C_Start();
    bool ok = I2C_WriteByte((i2cAddr << 1) | 0); /* write */
    if (ok) ok = I2C_WriteByte(regAddr);
    if (ok) ok = I2C_WriteByte(data);
    I2C_Stop();
    return ok;
}

bool M2SDR::Si5351_Read(uint8_t i2cAddr, uint8_t regAddr, uint8_t& data)
{
    I2C_Start();
    bool ok = I2C_WriteByte((i2cAddr << 1) | 0); /* write addr */
    if (!ok) { I2C_Stop(); return false; }
    ok = I2C_WriteByte(regAddr);
    if (!ok) { I2C_Stop(); return false; }
    /* Repeated start */
    I2C_Start();
    ok = I2C_WriteByte((i2cAddr << 1) | 1); /* read */
    if (!ok) { I2C_Stop(); return false; }
    /* Read byte – NACK to terminate single-byte read */
    data = 0;
    for (int i = 7; i >= 0; i--)
        data |= (I2C_ReadBit() ? 1 : 0) << i;
    I2C_WriteBit(true); /* NACK */
    I2C_Stop();
    return true;
}

OpStatus M2SDR::InitSi5351()
{
    lime::info("M2SDR: Initialising Si5351C clock chip");

    /* Wait for SYS_INIT = 0 (device ready) */
    uint8_t status = 0xFF;
    for (int i = 0; i < 100; i++)
    {
        if (Si5351_Read(SI5351_I2C_ADDR, 0, status) && (status & 0x80) == 0)
            break;
        usleep(1000);
    }
    if (status & 0x80)
    {
        lime::warning("M2SDR: Si5351 SYS_INIT not cleared");
        return OpStatus::Error;
    }

    /* Disable all outputs while reconfiguring */
    uint8_t dis = 0xFF;
    Si5351_Write(SI5351_I2C_ADDR, 3, dis);

    /* Power down all output drivers */
    for (int reg = 16; reg <= 23; reg++)
        Si5351_Write(SI5351_I2C_ADDR, reg, 0x80);

    /* Clear interrupt mask */
    Si5351_Write(SI5351_I2C_ADDR, 2, 0x00);

    /* Apply pre-computed register map */
    for (size_t i = 0; i < SI5351_CFG_LEN; i++)
        Si5351_Write(SI5351_I2C_ADDR, si5351_xo_38p4m_config[i][0],
                                      si5351_xo_38p4m_config[i][1]);

    /* Apply PLL soft reset (PLLA | PLLB) */
    Si5351_Write(SI5351_I2C_ADDR, 177, 0xAC);

    /* Wait for PLL lock */
    for (int i = 0; i < 100; i++)
    {
        if (Si5351_Read(SI5351_I2C_ADDR, 1, status) && (status & 0x60) == 0)
            break;
        usleep(1000);
    }
    if (status & 0x60)
        lime::warning("M2SDR: Si5351 PLL lock timeout (status=0x%02X)", status);

    /* Enable all outputs */
    Si5351_Write(SI5351_I2C_ADDR, 3, 0x00);

    lime::info("M2SDR: Si5351C initialised");
    return OpStatus::Success;
}

/* --------------------------------------------------------------------------
 * Init / Reset
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::Init()
{
    lime::info("M2SDR: Init");

    /* 1. Initialise Si5351C to produce 38.4 MHz for AD9361 */
    OpStatus st = InitSi5351();
    if (st != OpStatus::Success)
        lime::warning("M2SDR: Si5351 init failed – continuing anyway");

    usleep(10000); /* let the clock settle */

    /* 2. Initialise AD9361 */
    st = mAD9361->Init(*mSerialPort);
    if (st != OpStatus::Success)
    {
        lime::error("M2SDR: AD9361 init failed");
        return st;
    }

    lime::info("M2SDR: Init complete");
    return OpStatus::Success;
}

OpStatus M2SDR::Reset()
{
    return Init();
}

/* --------------------------------------------------------------------------
 * Configure (high-level SDRConfig)
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::Configure(const SDRConfig& config, uint8_t moduleIndex)
{
    if (!mAD9361->IsInitialised())
    {
        OpStatus st = Init();
        if (st != OpStatus::Success)
            return st;
    }

    for (uint8_t channelIdx = 0; channelIdx < 2; channelIdx++)
    {
        const ChannelConfig& chanCfg = config.channel[channelIdx];

        /* RX */
        if (chanCfg.rx.enabled)
        {
            if (chanCfg.rx.centerFrequency > 0)
                SetFrequency(moduleIndex, TRXDir::Rx, channelIdx, chanCfg.rx.centerFrequency);

            if (chanCfg.rx.sampleRate > 0)
                SetSampleRate(moduleIndex, TRXDir::Rx, channelIdx, chanCfg.rx.sampleRate, 1);

            if (chanCfg.rx.lpf > 0)
                SetLowPassFilter(moduleIndex, TRXDir::Rx, channelIdx, chanCfg.rx.lpf);
        }

        /* TX */
        if (chanCfg.tx.enabled)
        {
            if (chanCfg.tx.centerFrequency > 0)
                SetFrequency(moduleIndex, TRXDir::Tx, channelIdx, chanCfg.tx.centerFrequency);

            if (chanCfg.tx.sampleRate > 0)
                SetSampleRate(moduleIndex, TRXDir::Tx, channelIdx, chanCfg.tx.sampleRate, 1);

            if (chanCfg.tx.lpf > 0)
                SetLowPassFilter(moduleIndex, TRXDir::Tx, channelIdx, chanCfg.tx.lpf);
        }
    }

    return OpStatus::Success;
}

/* --------------------------------------------------------------------------
 * Clock frequency (not applicable — clock is fixed by Si5351)
 * -------------------------------------------------------------------------- */
double M2SDR::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return 0.0;
}

OpStatus M2SDR::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return OpStatus::NotImplemented;
}

/* --------------------------------------------------------------------------
 * Frequency
 * -------------------------------------------------------------------------- */
double M2SDR::GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (channel > 1)
        return 0;
    return trx == TRXDir::Rx ? mRxFreq[channel] : mTxFreq[channel];
}

OpStatus M2SDR::SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency)
{
    if (!mAD9361->IsInitialised())
        return OpStatus::NotImplemented;

    OpStatus st;
    if (trx == TRXDir::Rx)
    {
        st = mAD9361->SetRxLO(static_cast<uint64_t>(frequency));
        if (st == OpStatus::Success)
            mRxFreq[0] = mRxFreq[1] = frequency; /* shared LO */
    }
    else
    {
        st = mAD9361->SetTxLO(static_cast<uint64_t>(frequency));
        if (st == OpStatus::Success)
            mTxFreq[0] = mTxFreq[1] = frequency;
    }
    return st;
}

/* --------------------------------------------------------------------------
 * Sample rate
 * -------------------------------------------------------------------------- */
double M2SDR::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
    uint32_t* rf_samplerate)
{
    if (rf_samplerate)
        *rf_samplerate = static_cast<uint32_t>(mSampleRate);
    return mSampleRate;
}

OpStatus M2SDR::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
    double sampleRate, uint8_t oversample)
{
    if (!mAD9361->IsInitialised())
        return OpStatus::NotImplemented;

    OpStatus st = mAD9361->SetSampleRate(static_cast<uint32_t>(sampleRate));
    if (st == OpStatus::Success)
        mSampleRate = sampleRate;
    return st;
}

/* --------------------------------------------------------------------------
 * Gain
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel,
    eGainTypes gain, double& value)
{
    if (!mAD9361->IsInitialised())
        return OpStatus::NotImplemented;
    if (channel > 1)
        return OpStatus::InvalidValue;

    if (direction == TRXDir::Rx)
    {
        int32_t g = 0;
        OpStatus st = mAD9361->GetRxGain(channel, g);
        value = static_cast<double>(g);
        return st;
    }
    else
    {
        uint32_t atten_mdb = 0;
        OpStatus st = mAD9361->GetTxAttenuation(channel, atten_mdb);
        /* Return as negative dB (0 = max power, -89.75 = min power) */
        value = -static_cast<double>(atten_mdb) / 1000.0;
        return st;
    }
}

OpStatus M2SDR::SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel,
    eGainTypes gain, double value)
{
    if (!mAD9361->IsInitialised())
        return OpStatus::NotImplemented;
    if (channel > 1)
        return OpStatus::InvalidValue;

    if (direction == TRXDir::Rx)
    {
        int32_t g = static_cast<int32_t>(std::round(value));
        g = std::clamp(g, 0, 76);
        return mAD9361->SetRxGain(channel, g);
    }
    else
    {
        /* value is in dB; negative = attenuation */
        double atten_db = std::clamp(-value, 0.0, 89.75);
        uint32_t atten_mdb = static_cast<uint32_t>(std::round(atten_db * 1000.0));
        return mAD9361->SetTxAttenuation(channel, atten_mdb);
    }
}

/* --------------------------------------------------------------------------
 * Low-pass filter (RF bandwidth)
 * -------------------------------------------------------------------------- */
double M2SDR::GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return trx == TRXDir::Rx ? mRxBandwidth : mTxBandwidth;
}

OpStatus M2SDR::SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf)
{
    if (!mAD9361->IsInitialised())
        return OpStatus::NotImplemented;

    if (trx == TRXDir::Rx)
    {
        OpStatus st = mAD9361->SetRxBandwidth(static_cast<uint32_t>(lpf));
        if (st == OpStatus::Success)
            mRxBandwidth = lpf;
        return st;
    }
    else
    {
        OpStatus st = mAD9361->SetTxBandwidth(static_cast<uint32_t>(lpf));
        if (st == OpStatus::Success)
            mTxBandwidth = lpf;
        return st;
    }
}

/* --------------------------------------------------------------------------
 * Antenna (passthrough – AD9361 has one port pair per direction)
 * -------------------------------------------------------------------------- */
uint8_t M2SDR::GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus M2SDR::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    return OpStatus::Success;
}

/* --------------------------------------------------------------------------
 * Channel enable
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    return OpStatus::Success;
}

/* --------------------------------------------------------------------------
 * Status queries
 * -------------------------------------------------------------------------- */
bool M2SDR::GetCGENLocked(uint8_t moduleIndex)
{
    return mAD9361->IsInitialised();
}

bool M2SDR::GetSXLocked(uint8_t moduleIndex, TRXDir trx)
{
    return mAD9361->IsInitialised();
}

double M2SDR::GetTemperature(uint8_t moduleIndex)
{
    return 0.0; /* could read AD9361 internal temp sensor */
}

/* --------------------------------------------------------------------------
 * Register access
 * -------------------------------------------------------------------------- */
unsigned int M2SDR::ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA)
{
    if (useFPGA)
    {
        int val = mFPGA->ReadRegister(address);
        return val >= 0 ? static_cast<unsigned int>(val) : 0u;
    }
    /* AD9361 register read would require SPI – not exposed directly */
    return 0;
}

OpStatus M2SDR::WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value,
    bool useFPGA)
{
    if (useFPGA)
        return mFPGA->WriteRegister(address, value);
    return OpStatus::NotImplemented;
}

/* --------------------------------------------------------------------------
 * Streaming
 * -------------------------------------------------------------------------- */
std::unique_ptr<lime::RFStream> M2SDR::StreamCreate(const StreamConfig& config,
    uint8_t moduleIndex)
{
    if (!mStreamPort)
    {
        lime::warning("M2SDR: stream port not available");
        return nullptr;
    }

    auto rxdma = std::make_shared<LimePCIeDMA>(mStreamPort, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<LimePCIeDMA>(mStreamPort, DataTransferDirection::HostToDevice);

    auto streamer = std::make_unique<TRXLooper>(
        rxdma, txdma, mFPGA.get(), mLMSChips.at(0).get(), 0);

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);

    /* Provide the current sample rate so TRXLooper doesn't need to query
     * the stub LMS7002M's GetSampleRate() (which would return 0). */
    StreamConfig cfg = config;
    if (cfg.hintSampleRate == 0)
        cfg.hintSampleRate = mSampleRate;

    OpStatus st = streamer->Setup(cfg);
    if (st != OpStatus::Success)
        return nullptr;

    return streamer;
}

/* --------------------------------------------------------------------------
 * GetInternalChip
 * -------------------------------------------------------------------------- */
void* M2SDR::GetInternalChip(uint32_t index)
{
    if (index == 0)
        return mAD9361.get();
    return nullptr;
}

/* --------------------------------------------------------------------------
 * Stubs for LMS7002M-specific features
 * -------------------------------------------------------------------------- */
OpStatus M2SDR::Calibrate(uint8_t, TRXDir, uint8_t, double)
{
    return OpStatus::NotImplemented;
}

OpStatus M2SDR::ConfigureGFIR(uint8_t, TRXDir, uint8_t,
    ChannelConfig::Direction::GFIRFilter)
{
    return OpStatus::NotImplemented;
}

std::vector<double> M2SDR::GetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t)
{
    return {};
}

OpStatus M2SDR::SetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t, std::vector<double>)
{
    return OpStatus::NotImplemented;
}

OpStatus M2SDR::SetGFIR(uint8_t, TRXDir, uint8_t, uint8_t, bool)
{
    return OpStatus::NotImplemented;
}

OpStatus M2SDR::UploadMemory(eMemoryDevice, uint8_t, const char*, size_t,
    UploadMemoryCallback)
{
    return OpStatus::NotImplemented;
}

} // namespace lime
