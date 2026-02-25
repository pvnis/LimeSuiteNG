#pragma once

/*
 * M2SDR board support for LimeSuiteNG.
 *
 * M2SDR is an M.2 form-factor PCIe SDR using the AD9361 RFIC and Si5351C
 * clock chip.  It uses the same LiteX / LimeDFB gateware family as XTRX/sSDR.
 *
 * Copyright (c) 2024-2026 Lime Microsystems
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "boards/LMS7002M_SDRDevice.h"

#include <memory>
#include <cstdint>

namespace lime {

class LimePCIe;
class ISerialPort;
class ISPI;
class AD9361;

/*
 * M2SDR board class.
 *
 * Inherits LMS7002M_SDRDevice to reuse the streaming infrastructure
 * (TRXLooper, mStreamers, deprecated StreamSetup/Start/Stop/Rx/Tx).
 * A stub LMS7002M (backed by a no-op SPI) is held in mLMSChips[0] solely
 * so that TRXLooper can read LML1_TRXIQPULSE / LML1_SISODDR defaults (0).
 * All RF-control methods are overridden to drive the AD9361.
 */
class M2SDR : public LMS7002M_SDRDevice
{
  public:
    M2SDR() = delete;
    M2SDR(std::shared_ptr<LimePCIe> controlPort, std::shared_ptr<LimePCIe> streamPort);
    ~M2SDR() override;

    /* --- SDRDevice pure-virtual overrides --- */

    OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) override;
    OpStatus Init() override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    double GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency) override;

    double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        uint32_t* rf_samplerate = nullptr) override;
    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        double sampleRate, uint8_t oversample) override;

    OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel,
        eGainTypes gain, double& value) override;
    OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel,
        eGainTypes gain, double value) override;

    double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) override;

    uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        bool enable) override;

    bool GetCGENLocked(uint8_t moduleIndex) override;
    bool GetSXLocked(uint8_t moduleIndex, TRXDir trx) override;
    double GetTemperature(uint8_t moduleIndex) override;

    OpStatus Reset() override;

    /* Streaming */
    std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config,
        uint8_t moduleIndex) override;

    /* Register access (CSR passthrough) */
    unsigned int ReadRegister(uint8_t moduleIndex, unsigned int address,
        bool useFPGA = false) override;
    OpStatus WriteRegister(uint8_t moduleIndex, unsigned int address,
        unsigned int value, bool useFPGA = false) override;

    void* GetInternalChip(uint32_t index) override;

    /* Stub overrides for LMS7002M-specific features not present on AD9361 */
    OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        double bandwidth) override;
    OpStatus ConfigureGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        ChannelConfig::Direction::GFIRFilter settings) override;
    std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx,
        uint8_t channel, uint8_t gfirID) override;
    OpStatus SetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        uint8_t gfirID, std::vector<double> coefficients) override;
    OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel,
        uint8_t gfirID, bool enabled) override;

    OpStatus UploadMemory(eMemoryDevice device, uint8_t moduleIndex,
        const char* data, size_t length, UploadMemoryCallback callback) override;

  private:
    /* Bitbang I2C for Si5351 clock chip initialisation */
    void I2C_Start();
    void I2C_Stop();
    void I2C_WriteBit(bool bit);
    bool I2C_ReadBit();
    bool I2C_WriteByte(uint8_t byte);
    bool Si5351_Write(uint8_t i2cAddr, uint8_t regAddr, uint8_t data);
    bool Si5351_Read(uint8_t i2cAddr, uint8_t regAddr, uint8_t& data);
    OpStatus InitSi5351();

    OpStatus BuildDescriptor();

    std::shared_ptr<LimePCIe>  mControlPort;
    std::shared_ptr<LimePCIe>  mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;

    std::unique_ptr<AD9361> mAD9361;

    double mRxFreq[2]; /* cached RX LO per channel (Hz) */
    double mTxFreq[2]; /* cached TX LO per channel (Hz) */
    double mSampleRate;
    double mRxBandwidth;
    double mTxBandwidth;
};

} // namespace lime
