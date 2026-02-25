/*
 * C++ wrapper for the Analog Devices AD9361 no-OS driver.
 *
 * Copyright (c) 2024-2026 Lime Microsystems
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "AD9361.h"

/* no-OS C driver headers */
extern "C" {
#include "ad9361_api.h"
#include "ad9361.h"
}

#include "comms/ISerialPort.h"
#include "limesuiteng/Logger.h"

/* Declared in platform.cpp */
void m2sdr_platform_set_port(lime::ISerialPort* port);

using namespace lime;

/*
 * Default AD9361 init parameters for M2SDR hardware.
 * FDD, 2RX/2TX, 30.72 MSPS, 38.4 MHz refclk, LVDS MIMO.
 */
static AD9361_InitParam default_ad9361_init = {
    /* Device selection */
    ID_AD9361,
    /* id_no */ 0,
    /* reference_clk_rate */ 38400000UL,
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
    /* rx_synthesizer_frequency_hz */ 2400000000ULL,
    /* tx_synthesizer_frequency_hz */ 2400000000ULL,
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
    /* gc_rx1_mode */ 2, /* FAST_AGC */
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
    /* temp_sense_offset_signed */ static_cast<int8_t>(0xCE),
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

/* ========================================================================== */

AD9361::AD9361()
    : mPhy(nullptr)
    , mPort(nullptr)
    , mCurrentSampleRate(30720000)
    , mCurrentRxBw(18000000)
    , mCurrentTxBw(18000000)
{
}

AD9361::~AD9361()
{
    if (mPhy)
    {
        /* The no-OS driver has no standard cleanup; free the top-level struct */
        free(mPhy);
        mPhy = nullptr;
    }
}

OpStatus AD9361::Init(ISerialPort& controlPort)
{
    mPort = &controlPort;

    /* Provide the control port to the platform layer */
    m2sdr_platform_set_port(&controlPort);

    /* do_init=1 performs full hardware initialisation */
    int ret = ad9361_init(&mPhy, &default_ad9361_init, 1);
    if (ret != 0)
    {
        lime::error("AD9361: init failed (%d)", ret);
        mPhy = nullptr;
        return OpStatus::Error;
    }

    mCurrentSampleRate = default_ad9361_init.rx_path_clock_frequencies[5];
    return OpStatus::Success;
}

OpStatus AD9361::SetRxLO(uint64_t freq_hz)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_rx_lo_freq(mPhy, freq_hz);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetRxLO(uint64_t& freq_hz) const
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_get_rx_lo_freq(mPhy, &freq_hz);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::SetTxLO(uint64_t freq_hz)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_tx_lo_freq(mPhy, freq_hz);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetTxLO(uint64_t& freq_hz) const
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_get_tx_lo_freq(mPhy, &freq_hz);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::SetSampleRate(uint32_t rate_hz)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_rx_sampling_freq(mPhy, rate_hz);
    if (ret != 0)
        return OpStatus::Error;
    ret = ad9361_set_tx_sampling_freq(mPhy, rate_hz);
    if (ret != 0)
        return OpStatus::Error;
    mCurrentSampleRate = rate_hz;
    return OpStatus::Success;
}

OpStatus AD9361::GetSampleRate(uint32_t& rate_hz) const
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    uint32_t rx = 0;
    int ret = ad9361_get_rx_sampling_freq(mPhy, &rx);
    if (ret != 0)
        return OpStatus::Error;
    rate_hz = rx;
    return OpStatus::Success;
}

OpStatus AD9361::SetRxBandwidth(uint32_t bw_hz)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_rx_rf_bandwidth(mPhy, bw_hz);
    if (ret == 0)
        mCurrentRxBw = bw_hz;
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetRxBandwidth(uint32_t& bw_hz) const
{
    bw_hz = mCurrentRxBw;
    return OpStatus::Success;
}

OpStatus AD9361::SetTxBandwidth(uint32_t bw_hz)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_tx_rf_bandwidth(mPhy, bw_hz);
    if (ret == 0)
        mCurrentTxBw = bw_hz;
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetTxBandwidth(uint32_t& bw_hz) const
{
    bw_hz = mCurrentTxBw;
    return OpStatus::Success;
}

OpStatus AD9361::SetRxGain(uint8_t ch, int32_t gain_db)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_rx_rf_gain(mPhy, ch, gain_db);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetRxGain(uint8_t ch, int32_t& gain_db) const
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_get_rx_rf_gain(mPhy, ch, &gain_db);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::SetTxAttenuation(uint8_t ch, uint32_t atten_mdb)
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_set_tx_attenuation(mPhy, ch, atten_mdb);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus AD9361::GetTxAttenuation(uint8_t ch, uint32_t& atten_mdb) const
{
    if (!mPhy)
        return OpStatus::NotImplemented;
    int ret = ad9361_get_tx_attenuation(mPhy, ch, &atten_mdb);
    return ret == 0 ? OpStatus::Success : OpStatus::Error;
}
