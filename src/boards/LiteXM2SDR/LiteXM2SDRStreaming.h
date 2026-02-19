#pragma once

#include <cstdint>
#include <chrono>
#include <poll.h>
#include <vector>

#include "limesuiteng/RFStream.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"

// Forward declarations from litepcie
struct litepcie_dma_ctrl;

namespace lime {

/// @brief RFStream implementation for LiteXM2SDR using litepcie DMA.
///
/// The AD9361 in LVDS 2T2R mode produces 12-bit I/Q samples sign-extended to
/// 16-bit.  The DMA buffer layout is interleaved:
///   [I_ch0, Q_ch0, I_ch1, Q_ch1, ...] with each element an int16_t.
class LiteXM2SDRStreaming : public RFStream
{
  public:
    LiteXM2SDRStreaming(int fd, const StreamConfig& config);
    ~LiteXM2SDRStreaming() override;

    // RFStream interface
    uint64_t GetHardwareTimestamp() const override;
    OpStatus Setup(const StreamConfig& config) override;
    const StreamConfig& GetConfig() const override;
    OpStatus Start() override;
    OpStatus StageStart() override;
    void Stop() override;
    void Teardown() override;

    uint32_t StreamRx(lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;

    uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) override;

    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

    uint32_t Receive(lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta) override;
    uint32_t Receive(lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta) override;
    uint32_t Receive(lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta) override;

    uint32_t Transmit(const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;
    uint32_t Transmit(const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;
    uint32_t Transmit(const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;

  private:
    // Internal receive/transmit helpers
    uint32_t DoStreamRx(void* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout, bool isF32, bool is12);
    uint32_t DoStreamTx(const void* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout, bool isF32, bool is12);

    // Deinterleave from DMA buffer into user channel buffers
    void DeinterleaveRx(const int16_t* buf, void* const* dst, uint32_t sampleCount, bool isF32, bool is12) const;
    // Interleave from user channel buffers into DMA buffer
    void InterleaveRx(const void* const* src, int16_t* buf, uint32_t sampleCount, bool isF32, bool is12) const;

    int mFd;
    StreamConfig mConfig;
    bool mStarted;

    // DMA state — RX
    bool mHasRx;
    char* mRxBuf;             // mmap'd DMA RX buffers
    uint64_t mRxBufSize;      // bytes per DMA buffer
    uint64_t mRxBufCount;     // number of DMA buffers
    int64_t mRxHwCount;       // hardware buffer counter
    int64_t mRxSwCount;       // software (kernel) buffer counter
    int64_t mRxUserCount;     // user-space buffer counter

    // DMA state — TX
    bool mHasTx;
    char* mTxBuf;             // mmap'd DMA TX buffers
    uint64_t mTxBufSize;      // bytes per DMA buffer
    uint64_t mTxBufCount;     // number of DMA buffers
    int64_t mTxHwCount;
    int64_t mTxSwCount;
    int64_t mTxUserCount;

    int mNumChannels; // 1 or 2

    // Statistics
    StreamStats mRxStats;
    StreamStats mTxStats;
};

} // namespace lime
