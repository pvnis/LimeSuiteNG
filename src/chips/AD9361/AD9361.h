#pragma once

/*
 * C++ wrapper for the Analog Devices AD9361 no-OS driver.
 *
 * Copyright (c) 2024-2026 Lime Microsystems
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cstdint>

#include "limesuiteng/OpStatus.h"

/* Forward declarations of internal C driver types */
struct ad9361_rf_phy;

namespace lime {

class ISerialPort;

/*
 * AD9361 – C++ wrapper around the Analog Devices no-OS driver.
 *
 * Call Init() once to power on and calibrate the chip, then use
 * the setter/getter methods for runtime parameter changes.
 *
 * The default initialisation parameters target the M2SDR hardware
 * (FDD, 2RX/2TX, 30.72 MSPS, 38.4 MHz refclk, LVDS MIMO).
 */
class AD9361
{
  public:
    AD9361();
    ~AD9361();

    /* Forbid copy/move */
    AD9361(const AD9361&) = delete;
    AD9361& operator=(const AD9361&) = delete;

    /*
     * Initialise the AD9361 using the supplied control port and
     * built-in default parameters for the M2SDR hardware.
     * The port must remain valid for the lifetime of this object.
     */
    OpStatus Init(ISerialPort& controlPort);

    /* Frequency control */
    OpStatus SetRxLO(uint64_t freq_hz);
    OpStatus GetRxLO(uint64_t& freq_hz) const;
    OpStatus SetTxLO(uint64_t freq_hz);
    OpStatus GetTxLO(uint64_t& freq_hz) const;

    /* Sample-rate control (applies to both RX and TX) */
    OpStatus SetSampleRate(uint32_t rate_hz);
    OpStatus GetSampleRate(uint32_t& rate_hz) const;

    /* RF bandwidth */
    OpStatus SetRxBandwidth(uint32_t bw_hz);
    OpStatus GetRxBandwidth(uint32_t& bw_hz) const;
    OpStatus SetTxBandwidth(uint32_t bw_hz);
    OpStatus GetTxBandwidth(uint32_t& bw_hz) const;

    /*
     * RX gain.
     * ch: 0 = RX1, 1 = RX2.
     * gain_db: 0–76 dB in MGC mode.
     */
    OpStatus SetRxGain(uint8_t ch, int32_t gain_db);
    OpStatus GetRxGain(uint8_t ch, int32_t& gain_db) const;

    /*
     * TX attenuation (expressed as positive millidB, 0–89 500 mdB).
     * ch: 0 = TX1, 1 = TX2.
     */
    OpStatus SetTxAttenuation(uint8_t ch, uint32_t atten_mdb);
    OpStatus GetTxAttenuation(uint8_t ch, uint32_t& atten_mdb) const;

    /* True after a successful Init() */
    bool IsInitialised() const { return mPhy != nullptr; }

    /* Access to raw no-OS handle (for advanced use) */
    struct ad9361_rf_phy* GetPhy() const { return mPhy; }

  private:
    struct ad9361_rf_phy* mPhy;
    ISerialPort* mPort;
    uint32_t mCurrentSampleRate;
    uint32_t mCurrentRxBw;
    uint32_t mCurrentTxBw;
};

} // namespace lime
