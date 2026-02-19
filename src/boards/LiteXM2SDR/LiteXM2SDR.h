#pragma once

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/RFStream.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct ad9361_rf_phy;

namespace lime {

/// @brief LimeSuiteNG support for the LiteX-M2SDR board (AD9361 RFIC, PCIe).
class LiteXM2SDR : public SDRDevice
{
  public:
    LiteXM2SDR() = delete;
    explicit LiteXM2SDR(const std::string& devPath);
    ~LiteXM2SDR() override;

    /// Read serial/DNA from an already-open device fd (used by factory).
    static std::string ReadSerial(int fd);

    // -------------------------------------------------------------------------
    // SDRDevice interface
    // -------------------------------------------------------------------------
    const SDRDescriptor& GetDescriptor() const override;

    OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) override;
    OpStatus Init() override;
    OpStatus Reset() override;
    OpStatus GetGPSLock(GPS_Lock* status) override;

    OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable) override;

    double GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    double GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset) override;
    OpStatus SetNCOFrequency(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset = -1.0) override;
    double GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    int GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv) override;

    double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate = nullptr) override;
    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value) override;
    OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value) override;

    double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) override;

    uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel) override;
    OpStatus SetTestSignal(uint8_t moduleIndex,
        TRXDir direction,
        uint8_t channel,
        ChannelConfig::Direction::TestSignal signalConfiguration,
        int16_t dc_i = 0,
        int16_t dc_q = 0) override;

    bool GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic) override;
    complex64f_t GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset) override;

    complex64f_t GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance) override;

    bool GetCGENLocked(uint8_t moduleIndex) override;
    bool GetSXLocked(uint8_t moduleIndex, TRXDir trx) override;

    double GetTemperature(uint8_t moduleIndex) override;

    unsigned int ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA = false) override;
    OpStatus WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA = false) override;

    OpStatus LoadConfig(uint8_t moduleIndex, const std::string& filename) override;
    OpStatus SaveConfig(uint8_t moduleIndex, const std::string& filename) override;

    uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey) override;
    OpStatus SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value) override;
    uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb) override;
    OpStatus SetParameter(
        uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value) override;

    OpStatus Synchronize(bool toChip) override;
    void EnableCache(bool enable) override;

    OpStatus ConfigureGFIR(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings) override;
    std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID) override;
    OpStatus SetGFIRCoefficients(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients) override;
    OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled) override;

    uint64_t GetHardwareTimestamp(uint8_t moduleIndex) override;
    OpStatus SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;

    OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override;

    OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth) override;
    void* GetInternalChip(uint32_t index) override;

    // Deprecated streaming API (delegates to mStreamer)
    OpStatus StreamSetup(const StreamConfig& config, uint8_t moduleIndex) override;
    void StreamStart(uint8_t moduleIndex) override;
    void StreamStop(uint8_t moduleIndex) override;
    void StreamDestroy(uint8_t moduleIndex) override;
    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    void StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx) override;

    // Expose fd for streaming
    int GetFd() const { return mFd; }

  private:
    void BuildDescriptor();
    OpStatus InitHardware(bool doInit);

    int mFd;
    uint8_t mSpiId;
    struct ad9361_rf_phy* mAd9361;
    SDRDescriptor mDeviceDescriptor;
    std::unique_ptr<lime::RFStream> mStreamer; // for deprecated Stream* API

    // Per-channel/direction state cache
    static constexpr int MAX_CHANNELS = 2;
    double mRxFreq;
    double mTxFreq;
    double mRxSampleRate;
    double mTxSampleRate;
    double mRxGain[MAX_CHANNELS]; // dB (0..73)
    double mTxAtten[MAX_CHANNELS]; // milli-dB (0..89000)
    double mRxBandwidth;
    double mTxBandwidth;
    uint8_t mRxAntenna[MAX_CHANNELS];
    uint8_t mTxAntenna[MAX_CHANNELS];
    bool mChannelEnabled[MAX_CHANNELS][2]; // [channel][0=rx,1=tx]

    std::mutex mMutex;
};

} // namespace lime
