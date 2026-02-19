/*
 * LiteXM2SDRStreaming — RFStream implementation using litepcie DMA.
 *
 * The DMA ring buffer holds interleaved int16_t I/Q samples for 2 channels:
 *   [ I_ch0, Q_ch0, I_ch1, Q_ch1, ... ]
 * The AD9361 natively produces 12-bit samples; the gateware sign-extends them
 * to 16-bit before DMA.  Both 12-bit and 16-bit user formats are supported.
 */

#include "LiteXM2SDRStreaming.h"
#include "limesuiteng/Logger.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <algorithm>

// LitePCIe kernel API (IOCTL definitions, DMA info structs):
#include "litepcie.h"
// LitePCIe user-space DMA control:
#include "litepcie_dma.h"
// LitePCIe register read/write:
#include "litepcie_helpers.h"

namespace lime {

// ---- normalisation factor for 12-bit → float conversion ----------------
static constexpr float kRxScale = 1.0f / 2048.0f; // 12-bit signed → [-1, 1)
static constexpr float kTxScale = 2048.0f;          // float [-1,1) → 12-bit

// =========================================================================
//  Constructor / Destructor
// =========================================================================

LiteXM2SDRStreaming::LiteXM2SDRStreaming(int fd, const StreamConfig& config)
    : mFd(fd)
    , mConfig(config)
    , mStarted(false)
    , mHasRx(false)
    , mRxBuf(nullptr)
    , mRxBufSize(0)
    , mRxBufCount(0)
    , mRxHwCount(0)
    , mRxSwCount(0)
    , mRxUserCount(0)
    , mHasTx(false)
    , mTxBuf(nullptr)
    , mTxBufSize(0)
    , mTxBufCount(0)
    , mTxHwCount(0)
    , mTxSwCount(0)
    , mTxUserCount(0)
    , mNumChannels(1)
{
}

LiteXM2SDRStreaming::~LiteXM2SDRStreaming()
{
    Stop();
    Teardown();
}

// =========================================================================
//  GetHardwareTimestamp
// =========================================================================
uint64_t LiteXM2SDRStreaming::GetHardwareTimestamp() const
{
#ifdef CSR_TIME_GEN_CONTROL_ADDR
    litepcie_writel(mFd, CSR_TIME_GEN_CONTROL_ADDR, 1 << CSR_TIME_GEN_CONTROL_READ_OFFSET);
    uint64_t hi = litepcie_readl(mFd, CSR_TIME_GEN_READ_TIME_ADDR);
    uint64_t lo = litepcie_readl(mFd, CSR_TIME_GEN_READ_TIME_ADDR + 4);
    return (hi << 32) | lo;
#else
    return 0;
#endif
}

// =========================================================================
//  Setup
// =========================================================================
OpStatus LiteXM2SDRStreaming::Setup(const StreamConfig& config)
{
    mConfig = config;

    // Determine directions from config
    mHasRx = config.channels.count(TRXDir::Rx) && !config.channels.at(TRXDir::Rx).empty();
    mHasTx = config.channels.count(TRXDir::Tx) && !config.channels.at(TRXDir::Tx).empty();

    if (!mHasRx && !mHasTx)
    {
        lime::ReportError(OpStatus::InvalidValue, "LiteXM2SDRStreaming: no channels configured");
        return OpStatus::InvalidValue;
    }

    // Number of active channels (both directions share the same count)
    if (mHasRx)
        mNumChannels = static_cast<int>(config.channels.at(TRXDir::Rx).size());
    else if (mHasTx)
        mNumChannels = static_cast<int>(config.channels.at(TRXDir::Tx).size());
    mNumChannels = std::max(1, std::min(2, mNumChannels));

    // Query DMA buffer layout from kernel driver
    struct litepcie_ioctl_mmap_dma_info info{};
    if (ioctl(mFd, LITEPCIE_IOCTL_MMAP_DMA_INFO, &info) < 0)
    {
        lime::ReportError(OpStatus::Error, "LiteXM2SDRStreaming: LITEPCIE_IOCTL_MMAP_DMA_INFO failed: %s", strerror(errno));
        return OpStatus::Error;
    }

    // Request DMA access
    struct litepcie_ioctl_lock lock{};
    lock.dma_reader_request = mHasTx ? 1 : 0;
    lock.dma_writer_request = mHasRx ? 1 : 0;
    if (ioctl(mFd, LITEPCIE_IOCTL_LOCK, &lock) < 0)
    {
        lime::ReportError(OpStatus::Error, "LiteXM2SDRStreaming: DMA lock failed: %s", strerror(errno));
        return OpStatus::Error;
    }
    if (lock.dma_reader_status == 0 && mHasTx)
    {
        lime::ReportError(OpStatus::Error, "LiteXM2SDRStreaming: TX DMA not available");
        return OpStatus::Error;
    }

    // mmap RX (DMA Writer) buffers
    if (mHasRx)
    {
        mRxBufSize = info.dma_rx_buf_size;
        mRxBufCount = info.dma_rx_buf_count;
        size_t total = static_cast<size_t>(mRxBufSize * mRxBufCount);
        mRxBuf = static_cast<char*>(mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, mFd, static_cast<off_t>(info.dma_rx_buf_offset)));
        if (mRxBuf == MAP_FAILED)
        {
            mRxBuf = nullptr;
            lime::ReportError(OpStatus::Error, "LiteXM2SDRStreaming: RX mmap failed: %s", strerror(errno));
            return OpStatus::Error;
        }
    }

    // mmap TX (DMA Reader) buffers
    if (mHasTx)
    {
        mTxBufSize = info.dma_tx_buf_size;
        mTxBufCount = info.dma_tx_buf_count;
        size_t total = static_cast<size_t>(mTxBufSize * mTxBufCount);
        mTxBuf = static_cast<char*>(mmap(nullptr, total, PROT_WRITE, MAP_SHARED, mFd, static_cast<off_t>(info.dma_tx_buf_offset)));
        if (mTxBuf == MAP_FAILED)
        {
            mTxBuf = nullptr;
            lime::ReportError(OpStatus::Error, "LiteXM2SDRStreaming: TX mmap failed: %s", strerror(errno));
            return OpStatus::Error;
        }
        // Pre-fill TX buffers with silence
        std::memset(mTxBuf, 0, total);
    }

    return OpStatus::Success;
}

const StreamConfig& LiteXM2SDRStreaming::GetConfig() const { return mConfig; }

// =========================================================================
//  Start / StageStart / Stop / Teardown
// =========================================================================
OpStatus LiteXM2SDRStreaming::Start()
{
    if (mStarted)
        return OpStatus::Success;

    mRxHwCount = mRxSwCount = mRxUserCount = 0;
    mTxHwCount = mTxSwCount = mTxUserCount = 0;

    if (mHasRx)
    {
        litepcie_dma_writer(mFd, 1, &mRxHwCount, &mRxSwCount);
    }
    if (mHasTx)
    {
        litepcie_dma_reader(mFd, 1, &mTxHwCount, &mTxSwCount);
    }

    mStarted = true;
    return OpStatus::Success;
}

OpStatus LiteXM2SDRStreaming::StageStart()
{
    // For LiteXM2SDR there is no separate "stage" phase; just Start().
    return Start();
}

void LiteXM2SDRStreaming::Stop()
{
    if (!mStarted)
        return;

    if (mHasRx)
        litepcie_dma_writer(mFd, 0, &mRxHwCount, &mRxSwCount);
    if (mHasTx)
        litepcie_dma_reader(mFd, 0, &mTxHwCount, &mTxSwCount);

    mStarted = false;
}

void LiteXM2SDRStreaming::Teardown()
{
    Stop();

    // Release DMA access
    struct litepcie_ioctl_lock lock{};
    lock.dma_reader_release = mHasTx ? 1 : 0;
    lock.dma_writer_release = mHasRx ? 1 : 0;
    ioctl(mFd, LITEPCIE_IOCTL_LOCK, &lock);

    if (mRxBuf)
    {
        munmap(mRxBuf, static_cast<size_t>(mRxBufSize * mRxBufCount));
        mRxBuf = nullptr;
    }
    if (mTxBuf)
    {
        munmap(mTxBuf, static_cast<size_t>(mTxBufSize * mTxBufCount));
        mTxBuf = nullptr;
    }
}

// =========================================================================
//  Sample format conversion helpers
// =========================================================================

/// Deinterleave DMA buffer → per-channel user buffers.
///
/// DMA layout: [I0 Q0 I1 Q1] repeated, each value is int16_t (12-bit sign-ext).
void LiteXM2SDRStreaming::DeinterleaveRx(const int16_t* buf, void* const* dst, uint32_t count, bool isF32, bool is12) const
{
    const int stride = 2; // I + Q per channel per sample
    if (isF32)
    {
        complex32f_t* const* out = reinterpret_cast<complex32f_t* const*>(dst);
        for (uint32_t n = 0; n < count; ++n)
        {
            for (int ch = 0; ch < mNumChannels; ++ch)
            {
                out[ch][n].real(static_cast<float>(buf[n * stride * mNumChannels + ch * stride + 0]) * kRxScale);
                out[ch][n].imag(static_cast<float>(buf[n * stride * mNumChannels + ch * stride + 1]) * kRxScale);
            }
        }
    }
    else
    {
        // Both complex16_t and complex12_t share the same int16_t layout
        complex16_t* const* out = reinterpret_cast<complex16_t* const*>(dst);
        for (uint32_t n = 0; n < count; ++n)
        {
            for (int ch = 0; ch < mNumChannels; ++ch)
            {
                out[ch][n].real(buf[n * stride * mNumChannels + ch * stride + 0]);
                out[ch][n].imag(buf[n * stride * mNumChannels + ch * stride + 1]);
            }
        }
    }
}

/// Interleave per-channel user buffers → DMA buffer.
void LiteXM2SDRStreaming::InterleaveRx(const void* const* src, int16_t* buf, uint32_t count, bool isF32, bool is12) const
{
    const int stride = 2;
    if (isF32)
    {
        const complex32f_t* const* in = reinterpret_cast<const complex32f_t* const*>(src);
        for (uint32_t n = 0; n < count; ++n)
        {
            for (int ch = 0; ch < mNumChannels; ++ch)
            {
                float rawI = in[ch][n].real() * kTxScale;
                float rawQ = in[ch][n].imag() * kTxScale;
                buf[n * stride * mNumChannels + ch * stride + 0] = static_cast<int16_t>(std::max(-2048.0f, std::min(2047.0f, rawI)));
                buf[n * stride * mNumChannels + ch * stride + 1] = static_cast<int16_t>(std::max(-2048.0f, std::min(2047.0f, rawQ)));
            }
        }
    }
    else
    {
        const complex16_t* const* in = reinterpret_cast<const complex16_t* const*>(src);
        for (uint32_t n = 0; n < count; ++n)
        {
            for (int ch = 0; ch < mNumChannels; ++ch)
            {
                buf[n * stride * mNumChannels + ch * stride + 0] = in[ch][n].real();
                buf[n * stride * mNumChannels + ch * stride + 1] = in[ch][n].imag();
            }
        }
    }
}

// =========================================================================
//  Internal DoStreamRx / DoStreamTx
// =========================================================================

uint32_t LiteXM2SDRStreaming::DoStreamRx(
    void* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout, bool isF32, bool is12)
{
    if (!mHasRx || !mRxBuf)
        return 0;

    // Samples per DMA buffer (one "buffer" = one DMA transfer of mRxBufSize bytes)
    // Layout: mNumChannels channels × 2 int16_t (I+Q) per sample
    const uint32_t samplesPerBuf = static_cast<uint32_t>(mRxBufSize) / (static_cast<uint32_t>(mNumChannels) * 2 * sizeof(int16_t));

    uint32_t samplesReturned = 0;

    while (samplesReturned < count)
    {
        // Check for an available DMA buffer
        int64_t available = mRxHwCount - mRxUserCount;
        if (available <= 0)
        {
            // Poll hardware counter
            litepcie_dma_writer(mFd, 1, &mRxHwCount, &mRxSwCount);
            available = mRxHwCount - mRxUserCount;
        }

        if (available <= 0)
        {
            if (timeout.count() <= 0)
                break;

            // Wait via poll
            struct pollfd pfd{};
            pfd.fd = mFd;
            pfd.events = POLLIN;
            int ms = static_cast<int>(timeout.count() / 1000);
            int ret = poll(&pfd, 1, ms);
            if (ret <= 0)
                break;

            litepcie_dma_writer(mFd, 1, &mRxHwCount, &mRxSwCount);
            available = mRxHwCount - mRxUserCount;
            if (available <= 0)
                break;

            // Zero timeout after first wait to avoid blocking forever
            timeout = std::chrono::microseconds(0);
        }

        // Detect overflow: hardware too far ahead of user
        if ((mRxHwCount - mRxSwCount) > static_cast<int64_t>(mRxBufCount / 2))
        {
            // Drain: advance sw_count to hw_count
            struct litepcie_ioctl_mmap_dma_update upd{};
            upd.sw_count = mRxHwCount;
            ioctl(mFd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &upd);
            mRxSwCount = mRxHwCount;
            mRxUserCount = mRxHwCount;
            mRxStats.overrun++;
            continue;
        }

        // Determine how many samples to copy from this buffer
        uint32_t toRead = std::min(count - samplesReturned, samplesPerBuf);

        // Get pointer to current DMA buffer
        uint64_t bufIdx = static_cast<uint64_t>(mRxUserCount) % mRxBufCount;
        const int16_t* dmaBuf = reinterpret_cast<const int16_t*>(mRxBuf + bufIdx * mRxBufSize);

        // Build offset output pointers
        // We need per-channel pointers offset by samplesReturned
        void* chPtrs[2];
        if (isF32)
        {
            complex32f_t* const* fSamples = reinterpret_cast<complex32f_t* const*>(samples);
            for (int ch = 0; ch < mNumChannels; ++ch)
                chPtrs[ch] = fSamples[ch] ? (fSamples[ch] + samplesReturned) : nullptr;
        }
        else
        {
            complex16_t* const* iSamples = reinterpret_cast<complex16_t* const*>(samples);
            for (int ch = 0; ch < mNumChannels; ++ch)
                chPtrs[ch] = iSamples[ch] ? (iSamples[ch] + samplesReturned) : nullptr;
        }

        DeinterleaveRx(dmaBuf, chPtrs, toRead, isF32, is12);

        samplesReturned += toRead;
        mRxUserCount++;

        // Inform kernel we've consumed this buffer
        struct litepcie_ioctl_mmap_dma_update upd{};
        upd.sw_count = mRxUserCount;
        ioctl(mFd, LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE, &upd);
        mRxSwCount = mRxUserCount;
    }

    if (meta)
    {
        meta->timestamp = 0; // timestamp support requires DMA header mode
        meta->waitForTimestamp = false;
        meta->flushPartialPacket = false;
    }

    mRxStats.bytesTransferred += samplesReturned * mNumChannels * 2 * static_cast<int64_t>(sizeof(int16_t));
    return samplesReturned;
}

uint32_t LiteXM2SDRStreaming::DoStreamTx(
    const void* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout, bool isF32, bool is12)
{
    if (!mHasTx || !mTxBuf)
        return 0;

    const uint32_t samplesPerBuf = static_cast<uint32_t>(mTxBufSize) / (static_cast<uint32_t>(mNumChannels) * 2 * sizeof(int16_t));

    uint32_t samplesSent = 0;

    while (samplesSent < count)
    {
        int64_t pending = mTxUserCount - mTxHwCount;

        if (pending >= static_cast<int64_t>(mTxBufCount))
        {
            litepcie_dma_reader(mFd, 1, &mTxHwCount, &mTxSwCount);
            pending = mTxUserCount - mTxHwCount;
        }

        if (pending >= static_cast<int64_t>(mTxBufCount))
        {
            if (timeout.count() <= 0)
                break;

            struct pollfd pfd{};
            pfd.fd = mFd;
            pfd.events = POLLOUT;
            int ms = static_cast<int>(timeout.count() / 1000);
            int ret = poll(&pfd, 1, ms);
            if (ret <= 0)
                break;

            litepcie_dma_reader(mFd, 1, &mTxHwCount, &mTxSwCount);
            pending = mTxUserCount - mTxHwCount;
            if (pending >= static_cast<int64_t>(mTxBufCount))
                break;

            timeout = std::chrono::microseconds(0);
        }

        uint32_t toWrite = std::min(count - samplesSent, samplesPerBuf);

        uint64_t bufIdx = static_cast<uint64_t>(mTxUserCount) % mTxBufCount;
        int16_t* dmaBuf = reinterpret_cast<int16_t*>(mTxBuf + bufIdx * mTxBufSize);

        // Build offset input pointers
        const void* chPtrs[2];
        if (isF32)
        {
            const complex32f_t* const* fSamples = reinterpret_cast<const complex32f_t* const*>(samples);
            for (int ch = 0; ch < mNumChannels; ++ch)
                chPtrs[ch] = fSamples[ch] ? (fSamples[ch] + samplesSent) : nullptr;
        }
        else
        {
            const complex16_t* const* iSamples = reinterpret_cast<const complex16_t* const*>(samples);
            for (int ch = 0; ch < mNumChannels; ++ch)
                chPtrs[ch] = iSamples[ch] ? (iSamples[ch] + samplesSent) : nullptr;
        }

        InterleaveRx(chPtrs, dmaBuf, toWrite, isF32, is12);

        // Zero-pad if we didn't fill the whole buffer
        if (toWrite < samplesPerBuf)
        {
            size_t offset = static_cast<size_t>(toWrite) * static_cast<size_t>(mNumChannels) * 2 * sizeof(int16_t);
            size_t remaining = static_cast<size_t>(samplesPerBuf - toWrite) * static_cast<size_t>(mNumChannels) * 2 * sizeof(int16_t);
            std::memset(reinterpret_cast<uint8_t*>(dmaBuf) + offset, 0, remaining);
        }

        mTxUserCount++;
        samplesSent += toWrite;

        // Submit buffer to kernel
        struct litepcie_ioctl_mmap_dma_update upd{};
        upd.sw_count = mTxUserCount;
        ioctl(mFd, LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE, &upd);
        mTxSwCount = mTxUserCount;
    }

    mTxStats.bytesTransferred += samplesSent * mNumChannels * 2 * static_cast<int64_t>(sizeof(int16_t));
    return samplesSent;
}

// =========================================================================
//  StreamRx overloads
// =========================================================================
uint32_t LiteXM2SDRStreaming::StreamRx(
    lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamRx(reinterpret_cast<void* const*>(samples), count, meta, timeout, true, false);
}

uint32_t LiteXM2SDRStreaming::StreamRx(
    lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamRx(reinterpret_cast<void* const*>(samples), count, meta, timeout, false, false);
}

uint32_t LiteXM2SDRStreaming::StreamRx(
    lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamRx(reinterpret_cast<void* const*>(samples), count, meta, timeout, false, true);
}

// =========================================================================
//  StreamTx overloads
// =========================================================================
uint32_t LiteXM2SDRStreaming::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamTx(reinterpret_cast<const void* const*>(samples), count, meta, timeout, true, false);
}

uint32_t LiteXM2SDRStreaming::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamTx(reinterpret_cast<const void* const*>(samples), count, meta, timeout, false, false);
}

uint32_t LiteXM2SDRStreaming::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return DoStreamTx(reinterpret_cast<const void* const*>(samples), count, meta, timeout, false, true);
}

// =========================================================================
//  Receive / Transmit (newer RFStream API, delegates to Stream*)
// =========================================================================
uint32_t LiteXM2SDRStreaming::Receive(lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    StreamMeta sm{};
    uint32_t got = StreamRx(samples, count, &sm, DEFAULT_TIMEOUT);
    if (meta)
    {
        meta->hasTimestamp = false;
        meta->timestamp = lime::Timespec{};
    }
    return got;
}

uint32_t LiteXM2SDRStreaming::Receive(lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    StreamMeta sm{};
    uint32_t got = StreamRx(samples, count, &sm, DEFAULT_TIMEOUT);
    if (meta)
        meta->hasTimestamp = false;
    return got;
}

uint32_t LiteXM2SDRStreaming::Receive(lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    StreamMeta sm{};
    uint32_t got = StreamRx(samples, count, &sm, DEFAULT_TIMEOUT);
    if (meta)
        meta->hasTimestamp = false;
    return got;
}

uint32_t LiteXM2SDRStreaming::Transmit(const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* /*meta*/)
{
    return StreamTx(samples, count, nullptr, DEFAULT_TIMEOUT);
}

uint32_t LiteXM2SDRStreaming::Transmit(const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* /*meta*/)
{
    return StreamTx(samples, count, nullptr, DEFAULT_TIMEOUT);
}

uint32_t LiteXM2SDRStreaming::Transmit(const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* /*meta*/)
{
    return StreamTx(samples, count, nullptr, DEFAULT_TIMEOUT);
}

// =========================================================================
//  StreamStatus
// =========================================================================
void LiteXM2SDRStreaming::StreamStatus(StreamStats* rx, StreamStats* tx)
{
    if (rx)
        *rx = mRxStats;
    if (tx)
        *tx = mTxStats;
}

} // namespace lime
