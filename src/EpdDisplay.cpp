#include "EpdDisplay.h"
#include "esp_heap_caps.h"

namespace
{
    constexpr int LEFT = 1, RIGHT = 2, BOTH = 3;
    constexpr int W = EpdDisplay::WIDTH;
    constexpr int H = EpdDisplay::HEIGHT;
    constexpr int ROW_BYTES = W / 8;                       // one full row of the framebuffer
    constexpr int HALF_ROW = W / 16;                       // one CoG's share of a row
    constexpr int ROWS_PER_CHUNK = 64;                     // rows converted + queued per DMA transfer
    constexpr int CHUNK_BYTES = ROWS_PER_CHUNK * HALF_ROW; // 3840 (<= 4096, the reference firmware's piece size)
    constexpr int STAGE_BYTES = 512;                       // aligned staging area for commands (LUT is 400 bytes)
    constexpr size_t DMA_ALIGN = 64;                       // cache-line multiple; keeps the IDF driver from bouncing buffers
    constexpr spi_host_device_t EPD_HOST = SPI2_HOST;      // FSPI

    // Session timings (ms) from the reference firmware (epd.c: PDi Wide_Large waits for this CoG family).
    // The session opens in ~350 ms with these; the old 200..500 ms waits were ~10x too conservative.
    constexpr uint32_t RESET_HIGH_MS = 2;
    constexpr uint32_t RESET_LOW_MS = 4;
    constexpr uint32_t RESET_POST_MS = 20;
    constexpr uint32_t POWER_MS = 50;
    constexpr uint32_t STROBE_HI_MS = 2;
    constexpr uint32_t STROBE_LO_MS = 10; // 0xB0 latches 10 ms after the last strobe

    uint8_t revTab[256]; // bit-reverse lookup

    // Calm transition table (reference firmware epd.c, panel_lab experiments 05 and 11-14).
    // Table index = (old << 1) | new, so only changed pixels are driven: table 1 (white->black) gets VDH,
    // table 2 (black->white) gets VDL for `drive` frames, then GND. Every group = one frame, phase c only;
    // 10 groups = 10 frames = ~237 ms. The last frame must stay GND, hence drive <= 9.
    void calmTable(uint8_t *t, int drive)
    {
        static const uint8_t group[8] = {0, 1, 1, 0, 1, 0, 1, 1};
        for (int k = 0; k < 50; k++)
            memcpy(t + 8 * k, group, 8);
        for (int f = 0; f < drive; f++)
        {
            t[(10 + f) * 8] = 1 << 2; // VDH in phase c
            t[(20 + f) * 8] = 2 << 2; // VDL
        }
    }

    // Full short waveform (fast mode, and the clean pass in calm mode): 5 tables (paper, ink, red, unused, VCOM)
    // x 10 groups x 8 bytes. A pull of this LUT runs ~1070 ms whatever the table asks for.
    const uint8_t LUT[400] = {
        // table 0: paper (white)
        0x99,
        0x0A,
        0x0A,
        0x01,
        0x0A,
        0x0A,
        0x01,
        0x03,
        0xAA,
        0x2D,
        0x03,
        0x01,
        0x2D,
        0x03,
        0x01,
        0x03,
        0xAA,
        0x2D,
        0x03,
        0x01,
        0x2D,
        0x03,
        0x01,
        0x03,
        0xAA,
        0x2D,
        0x03,
        0x01,
        0x2D,
        0x03,
        0x01,
        0x03,
        0xAA,
        0x2D,
        0x03,
        0x01,
        0x2D,
        0x03,
        0x01,
        0x03,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        // table 1: ink (black)
        0x66,
        0x0A,
        0x0A,
        0x01,
        0x0A,
        0x0A,
        0x01,
        0x03,
        0x44,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x44,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x44,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x44,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        // table 2: red
        0xEE,
        0x0A,
        0x0A,
        0x01,
        0x0A,
        0x0A,
        0x01,
        0x03,
        0xAA,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0xFF,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        // table 3: unused
        0x00,
        0x0A,
        0x0A,
        0x01,
        0x0A,
        0x0A,
        0x01,
        0x03,
        0x00,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x00,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x00,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x00,
        0x2D,
        0x01,
        0x01,
        0x2D,
        0x01,
        0x01,
        0x03,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        // table 4: VCOM
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x00,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
        0x01,
    };
} // namespace

// ===========================================================================
// Coroutine helpers. A coroutine is a bool member function: false = "not done, call me again",
// true = finished. Its position is a line number in its own `pt` member, so anything that must
// survive a yield has to be a member, never a plain local.
// Use at most one CO_ macro per source line.
// ===========================================================================
#define CO_BEGIN(pt) \
    switch (pt)      \
    {                \
    case 0:
#define CO_END(pt) \
    }              \
    pt = 0;        \
    return true
#define CO_WAIT_MS(pt, wait_ms)               \
    do                                        \
    {                                         \
        coT0_ = micros();                     \
        coDur_ = (uint32_t)(wait_ms) * 1000u; \
        __attribute__((fallthrough));         \
    case __LINE__:                            \
        pt = __LINE__;                        \
        if (micros() - coT0_ < coDur_)        \
            return false;                     \
    } while (0)
#define CO_WAIT_BUSY(pt, wait_ms)     \
    do                                \
    {                                 \
        busyBegin(wait_ms);           \
        __attribute__((fallthrough)); \
    case __LINE__:                    \
        pt = __LINE__;                \
        if (!busyPoll())              \
            return false;             \
    } while (0)
#define CO_AWAIT(pt, sub)             \
    do                                \
    {                                 \
        __attribute__((fallthrough)); \
    case __LINE__:                    \
        pt = __LINE__;                \
        if (!(sub))                   \
            return false;             \
    } while (0)
#define CMD(cog, index, ...)                \
    do                                      \
    {                                       \
        const uint8_t d_[] = {__VA_ARGS__}; \
        cmd(cog, index, d_, sizeof d_);     \
    } while (0)

// ===========================================================================
// Setup / teardown
// ===========================================================================
EpdDisplay::~EpdDisplay()
{
    if (spi_)
        spi_bus_remove_device(spi_);
    if (busOwned_)
        spi_bus_free(EPD_HOST);
    free(fb_);
    free(shown_);
    heap_caps_free(chunk_);
}

bool EpdDisplay::begin(const EpdPins &pins, const EpdConfig &config)
{
    pins_ = pins;
    cfg_ = config;
    if (cfg_.slowWaveform)
    {
        cfg_.firstPulls = 1;
        cfg_.framesPerSession = 0;
        pullTimeoutMs_ = 120000;
    }
    else
    {
        pullTimeoutMs_ = 5000;
    }
    calm_ = cfg_.calmWaveform && !cfg_.slowWaveform;
    if (cfg_.calmDriveFrames < 1)
        cfg_.calmDriveFrames = 1;
    if (cfg_.calmDriveFrames > 9)
        cfg_.calmDriveFrames = 9;

    if (!fb_)
        fb_ = (uint8_t *)malloc(FRAMEBUFFER_BYTES);
    if (calm_ && !shown_)
        shown_ = (uint8_t *)malloc(FRAMEBUFFER_BYTES);
    if (!chunk_)
        chunk_ = (uint8_t *)heap_caps_aligned_alloc(DMA_ALIGN, 2 * CHUNK_BYTES + STAGE_BYTES,
                                                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!fb_ || !chunk_ || (calm_ && !shown_))
        return false;
    memset(fb_, 0, FRAMEBUFFER_BYTES);
    if (shown_)
        memset(shown_, 0, FRAMEBUFFER_BYTES);
    shownValid_ = false;

    for (int i = 0; i < 256; i++)
    {
        uint8_t b = i;
        b = b >> 4 | b << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        revTab[i] = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    }

    pinMode(pins_.dc, OUTPUT);
    digitalWrite(pins_.dc, HIGH);
    pinMode(pins_.rst, OUTPUT);
    digitalWrite(pins_.rst, HIGH);
    pinMode(pins_.csLeft, OUTPUT);
    digitalWrite(pins_.csLeft, HIGH);
    pinMode(pins_.csRight, OUTPUT);
    digitalWrite(pins_.csRight, HIGH);
    pinMode(pins_.busyLeft, INPUT);
    pinMode(pins_.busyRight, INPUT);

    if (!spi_)
    {
        spi_bus_config_t bus = {};
        bus.mosi_io_num = pins_.mosi;
        bus.miso_io_num = -1;
        bus.sclk_io_num = pins_.sck;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = 4096;
        if (spi_bus_initialize(EPD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK)
            return false;
        busOwned_ = true;
        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = (int)cfg_.spiHz;
        dev.mode = 0;
        dev.spics_io_num = -1; // both CS lines are driven by hand
        dev.queue_size = 2;
        if (spi_bus_add_device(EPD_HOST, &dev, &spi_) != ESP_OK)
            return false;
    }

    job_ = JOB_NONE;
    sessionOpen_ = false;
    sessionCalm_ = false;
    closeAfter_ = reopen_ = false;
    fbInUse_ = false;
    return true;
}

// ===========================================================================
// Public control API
// ===========================================================================
bool EpdDisplay::busy() const { return job_ != JOB_NONE; }
bool EpdDisplay::canDraw() const { return fb_ && !fbInUse_; }

void EpdDisplay::invalidate() { shownValid_ = false; }

bool EpdDisplay::refresh()
{
    if (!fb_ || job_ != JOB_NONE)
        return false;
    // Nothing changed since the last frame the panel showed: no session, no pull, no wear.
    if (calm_ && shownValid_ && memcmp(fb_, shown_, FRAMEBUFFER_BYTES) == 0)
        return true;
    startJob();
    return true;
}

// Pick the job for the framebuffer as it is now. Called by refresh() and again after a reopen close.
void EpdDisplay::startJob()
{
    ldBlack_ = fb_;
    fbInUse_ = true;
    if (calm_)
    {
        const bool needClean = !shownValid_; // old frame unknown: one clean pass with the full LUT
        if (sessionOpen_)
        {
            if (needClean)
            { // an open calm session can't clean; close it and reopen with the full LUT
                reopen_ = true;
                job_ = JOB_CLOSE;
                return;
            }
            ldSend11_ = false; // 0x11 already holds the frame on the film
            ld11_ = nullptr;
            pullsLeft_ = 1;
            job_ = JOB_FRAME;
            return;
        }
        sessionCalm_ = !needClean;
        closeAfter_ = needClean; // the clean session only exists to show this one frame
        ldSend11_ = true;
        ld11_ = needClean ? nullptr : shown_;
        pullsLeft_ = needClean ? cfg_.cleanPulls : 1;
        job_ = JOB_SESSION;
        return;
    }
    sessionCalm_ = false;
    if (!sessionOpen_)
    {
        ldSend11_ = true;
        ld11_ = nullptr;
        pullsLeft_ = cfg_.firstPulls;
        job_ = JOB_SESSION;
    }
    else
    {
        ldSend11_ = cfg_.clearRedEachFrame;
        ld11_ = nullptr;
        pullsLeft_ = cfg_.framePulls;
        job_ = JOB_FRAME;
    }
}

bool EpdDisplay::powerOff()
{
    if (!fb_ || job_ != JOB_NONE)
        return false;
    if (sessionOpen_)
        job_ = JOB_CLOSE;
    return true;
}

void EpdDisplay::tick()
{
    bool done;
    switch (job_)
    {
    case JOB_SESSION:
        done = sessionCo();
        break;
    case JOB_FRAME:
        done = frameCo();
        break;
    case JOB_CLOSE:
        done = closeCo();
        break;
    default:
        if (sessionOpen_ && cfg_.autoPowerOffMs && millis() - idleT0_ >= cfg_.autoPowerOffMs)
            job_ = JOB_CLOSE;
        return;
    }
    if (done)
        finishJob();
}

void EpdDisplay::finishJob()
{
    switch (job_)
    {
    case JOB_SESSION:
        sessionOpen_ = true;
        framesDone_ = 0;
        if (calm_ && !sessionCalm_)
            shownValid_ = true; // the clean pass left exactly the framebuffer on the film
        break;
    case JOB_FRAME:
        framesDone_++;
        break;
    case JOB_CLOSE:
        sessionOpen_ = false;
        idleT0_ = millis();
        job_ = JOB_NONE;
        if (reopen_)
        {
            reopen_ = false;
            startJob(); // framebuffer is still held (fbInUse_)
        }
        return;
    default:
        return;
    }
    idleT0_ = millis();
    if (closeAfter_)
    {
        closeAfter_ = false;
        job_ = JOB_CLOSE;
    }
    else
    {
        // Session used up: power down right away; the next refresh() opens a fresh one.
        job_ = (framesDone_ >= cfg_.framesPerSession) ? JOB_CLOSE : JOB_NONE;
    }
}

// ===========================================================================
// Drawing
// ===========================================================================
void EpdDisplay::clear(bool ink)
{
    if (fb_)
        memset(fb_, ink ? 0xFF : 0x00, FRAMEBUFFER_BYTES);
}

void EpdDisplay::setPixel(int x, int y, bool ink)
{
    if (!fb_ || x < 0 || y < 0 || x >= W || y >= H)
        return;
    uint8_t m = 0x80 >> (x & 7);
    if (ink)
        fb_[y * ROW_BYTES + (x >> 3)] |= m;
    else
        fb_[y * ROW_BYTES + (x >> 3)] &= ~m;
}

// Byte-wise: edge masks for the partial bytes, memset for the middle.
void EpdDisplay::fillRect(int x, int y, int w, int h, bool ink)
{
    if (!fb_)
        return;
    if (x < 0)
    {
        w += x;
        x = 0;
    }
    if (y < 0)
    {
        h += y;
        y = 0;
    }
    if (x + w > W)
        w = W - x;
    if (y + h > H)
        h = H - y;
    if (w <= 0 || h <= 0)
        return;

    const int x1 = x + w - 1;
    const int b0 = x >> 3, b1 = x1 >> 3;
    const uint8_t m0 = 0xFF >> (x & 7);
    const uint8_t m1 = 0xFF << (7 - (x1 & 7));
    for (int j = y; j < y + h; j++)
    {
        uint8_t *row = fb_ + j * ROW_BYTES;
        if (b0 == b1)
        {
            const uint8_t m = m0 & m1;
            row[b0] = ink ? (row[b0] | m) : (row[b0] & ~m);
        }
        else
        {
            row[b0] = ink ? (row[b0] | m0) : (row[b0] & ~m0);
            if (b1 > b0 + 1)
                memset(row + b0 + 1, ink ? 0xFF : 0x00, b1 - b0 - 1);
            row[b1] = ink ? (row[b1] | m1) : (row[b1] & ~m1);
        }
    }
}

// ===========================================================================
// Low-level SPI
// ===========================================================================
// Everything is copied through the aligned DMA staging area so the IDF driver never has to bounce a buffer.
void EpdDisplay::spiTx(const uint8_t *d, size_t n)
{
    uint8_t *stage = chunk_ + 2 * CHUNK_BYTES;
    while (n)
    {
        size_t k = n < STAGE_BYTES ? n : STAGE_BYTES;
        memmove(stage, d, k);
        spi_transaction_t t = {};
        t.length = 8 * k;
        t.tx_buffer = stage;
        spi_device_polling_transmit(spi_, &t);
        d += k;
        n -= k;
    }
}

void EpdDisplay::select(int cog)
{
    digitalWrite(pins_.csLeft, !(cog & LEFT));
    digitalWrite(pins_.csRight, !(cog & RIGHT));
}

void EpdDisplay::deselect()
{
    digitalWrite(pins_.csLeft, HIGH);
    digitalWrite(pins_.csRight, HIGH);
}

// Index byte with DC low, then data with DC high (blocking; only for short transfers, nothing may be queued).
void EpdDisplay::cmd(int cog, uint8_t index, const uint8_t *data, size_t n)
{
    digitalWrite(pins_.dc, LOW);
    select(cog);
    spiTx(&index, 1);
    deselect();
    digitalWrite(pins_.dc, HIGH);
    select(cog);
    spiTx(data, n);
    deselect();
}

// Bit-reverse `rows` rows of this CoG's half of the plane into dst (strSrc_ NULL = white), advancing strRow_.
void EpdDisplay::fillChunk(uint8_t *dst, int rows)
{
    if (!strSrc_)
    {
        memset(dst, 0, rows * HALF_ROW);
    }
    else
    {
        const uint8_t *s = strSrc_ + (size_t)strRow_ * ROW_BYTES;
        for (int r = 0; r < rows; r++, s += ROW_BYTES, dst += HALF_ROW)
            for (int x = 0; x < HALF_ROW; x += 4)
            {
                dst[x] = revTab[s[x]];
                dst[x + 1] = revTab[s[x + 1]];
                dst[x + 2] = revTab[s[x + 2]];
                dst[x + 3] = revTab[s[x + 3]];
            }
    }
    strRow_ += rows;
}

// Start one plane into one CoG's RAM: RAM_RW pointer, index byte, then leave CS low for the data stream.
// Left CoG = bytes 0..59 of each row, right CoG = bytes 60..119. plane NULL = all white.
void EpdDisplay::streamBegin(int cog, uint8_t index, const uint8_t *plane)
{
    CMD(cog, 0x12, 0x3B, 0x00, 0x14); // RAM_RW
    digitalWrite(pins_.dc, LOW);
    select(cog);
    spiTx(&index, 1);
    deselect();
    digitalWrite(pins_.dc, HIGH);
    select(cog);
    strSrc_ = plane ? plane + (cog == RIGHT ? HALF_ROW : 0) : nullptr;
    strRow_ = 0;
    inflight_ = 0;
    slot_ = 0;
}

// Keep two DMA transfers in flight, converting the next chunk while the previous one clocks out.
// Returns true (and releases CS) once the whole half plane has been sent.
bool EpdDisplay::streamPoll()
{
    spi_transaction_t *done;
    while (inflight_ > 0 && spi_device_get_trans_result(spi_, &done, 0) == ESP_OK)
        inflight_--;
    while (inflight_ < 2 && strRow_ < H)
    {
        const int rows = (H - strRow_) < ROWS_PER_CHUNK ? (H - strRow_) : ROWS_PER_CHUNK;
        uint8_t *d = chunk_ + slot_ * CHUNK_BYTES;
        fillChunk(d, rows);
        spi_transaction_t &t = trans_[slot_];
        memset(&t, 0, sizeof t);
        t.length = 8 * rows * HALF_ROW;
        t.tx_buffer = d;
        if (spi_device_queue_trans(spi_, &t, 0) != ESP_OK)
        {
            strRow_ = H; // give up on this plane rather than hang
            break;
        }
        slot_ ^= 1;
        inflight_++;
    }
    if (inflight_ == 0 && strRow_ >= H)
    {
        deselect();
        return true;
    }
    return false;
}

void EpdDisplay::busyBegin(uint32_t timeoutMs)
{
    busyT0_ = millis();
    busyTimeout_ = timeoutMs;
    busyL_ = -1;
    busyR_ = cfg_.useBusyRight ? -1 : 0;
    busyTimedOut_ = false;
}

bool EpdDisplay::busyPoll()
{ // true when both CoGs are ready, or on timeout
    uint32_t t = millis() - busyT0_;
    if (busyL_ < 0 && digitalRead(pins_.busyLeft))
        busyL_ = (int)t;
    if (busyR_ < 0 && digitalRead(pins_.busyRight))
        busyR_ = (int)t;
    if (busyL_ >= 0 && busyR_ >= 0)
        return true;
    if (t > busyTimeout_)
    {
        busyTimedOut_ = true;
        return true;
    }
    return false;
}

// ===========================================================================
// Panel coroutines
// ===========================================================================

// Frame into CoG RAM: 0x10 = new frame, 0x11 = old frame (calm) / white (fast) / skipped.
// The DMA stream runs in the background; this yields until each half plane has been sent.
bool EpdDisplay::loadCo()
{
    CO_BEGIN(ptLoad_);
    CMD(BOTH, 0x13, 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02); // DUW: 60 bytes x 672 lines
    CMD(BOTH, 0x90, 0x00, 0x3B, 0x00, 0xA9);             // DRFW: whole panel
    for (cogI_ = LEFT; cogI_ <= RIGHT; cogI_++)
    {
        streamBegin(cogI_, 0x10, ldBlack_);
        CO_AWAIT(ptLoad_, streamPoll());
        if (ldSend11_)
        {
            streamBegin(cogI_, 0x11, ld11_);
            CO_AWAIT(ptLoad_, streamPoll());
        }
    }
    if (calm_)
        memcpy(shown_, ldBlack_, FRAMEBUFFER_BYTES); // what the film will show; next session's 0x11
    fbInUse_ = false;                                // the framebuffer is free again
    CO_END(ptLoad_);
}

bool EpdDisplay::dcdcCo()
{
    CO_BEGIN(ptDcdc_);
    CMD(BOTH, 0x51, 0x50, 0x01);
    for (v_ = 1; v_ <= 4; v_++)
    {
        CMD(BOTH, 0x09, 0x1F);
        CMD(BOTH, 0x51, 0x50, (uint8_t)v_);
        CMD(BOTH, 0x09, 0x9F);
        CO_WAIT_MS(ptDcdc_, 2);
    }
    for (v_ = 1; v_ <= 10; v_++)
    {
        CMD(BOTH, 0x09, 0x1F);
        CMD(BOTH, 0x51, 0x0A, (uint8_t)v_);
        CMD(BOTH, 0x09, 0x9F);
        CO_WAIT_MS(ptDcdc_, 2);
    }
    for (v_ = 3; v_ <= 10; v_++)
    {
        CMD(BOTH, 0x09, 0x7F);
        CMD(BOTH, 0x51, 0x0A, (uint8_t)v_);
        CMD(BOTH, 0x09, 0xFF);
        CO_WAIT_MS(ptDcdc_, 2);
    }
    for (v_ = 9; v_ >= 2; v_--)
    {
        CMD(BOTH, 0x09, 0x7F);
        CMD(BOTH, 0x51, (uint8_t)v_, 0x0A);
        CMD(BOTH, 0x09, 0xFF);
        CO_WAIT_MS(ptDcdc_, 2);
    }
    CMD(BOTH, 0x09, 0xFF);
    CO_WAIT_MS(ptDcdc_, 100);
    CO_WAIT_BUSY(ptDcdc_, 10000); // never trigger into a booster that is still ramping
    CO_END(ptDcdc_);
}

// Run the latched waveform pullsLeft_ times on what is in RAM (full LUT ~1.1 s each, calm ~0.24 s).
bool EpdDisplay::pullCo()
{
    CO_BEGIN(ptPull_);
    while (pullsLeft_ > 0)
    {
        CMD(BOTH, 0x15, 0x3C);
        CO_WAIT_MS(ptPull_, 5); // BUSY drops a few ms after the trigger; don't read the still-high line as done
        CO_WAIT_BUSY(ptPull_, pullTimeoutMs_);
        if (cfg_.debug)
            Serial.printf("  pull: BUSY left %d ms, right %d ms%s\n", busyL_, busyR_, busyTimedOut_ ? " (TIMEOUT)" : "");
        pullsLeft_--;
    }
    CO_END(ptPull_);
}

// Reset, load, register sequence, waveform, DC-DC up. ldBlack_/ld11_/ldSend11_ must be set by the caller.
bool EpdDisplay::openCo()
{
    CO_BEGIN(ptOpen_);
    digitalWrite(pins_.rst, HIGH);
    CO_WAIT_MS(ptOpen_, RESET_HIGH_MS);
    digitalWrite(pins_.rst, LOW);
    CO_WAIT_MS(ptOpen_, RESET_LOW_MS);
    digitalWrite(pins_.rst, HIGH);
    CO_WAIT_MS(ptOpen_, RESET_POST_MS);
    deselect();
    CO_WAIT_MS(ptOpen_, 5);
    CMD(BOTH, 0x70, 0x71, 0x36); // booster setting (otherwise a refresh draws ~2 A)

    CO_AWAIT(ptOpen_, loadCo());

    CMD(BOTH, 0x05, 0x7D);
    CO_WAIT_MS(ptOpen_, POWER_MS);
    CMD(BOTH, 0x05, 0x00);
    CO_WAIT_MS(ptOpen_, 1);
    CMD(BOTH, 0xD8, 0x80); // MS_SYNC
    CMD(BOTH, 0xD6, 0x00); // BVSS
    CMD(BOTH, 0xA7, 0x10);
    CO_WAIT_MS(ptOpen_, STROBE_HI_MS);
    CMD(BOTH, 0xA7, 0x00);
    CO_WAIT_MS(ptOpen_, STROBE_LO_MS);
    CMD(BOTH, 0x03, 0x00, 0x11); // OSC
    for (cogI_ = LEFT; cogI_ <= RIGHT; cogI_++)
    {
        CMD(cogI_, 0x44, 0x00);
        CMD(cogI_, 0x45, 0x80);
        CMD(cogI_, 0xA7, 0x10);
        CO_WAIT_MS(ptOpen_, STROBE_HI_MS);
        CMD(cogI_, 0xA7, 0x00);
        CO_WAIT_MS(ptOpen_, STROBE_LO_MS);
        CMD(cogI_, 0x44, 0x06);
        CMD(cogI_, 0x45, (uint8_t)(cfg_.slowWaveform ? 0x82 : 0x84)); // 25 C stock / 26 C short waveform
        CMD(cogI_, 0xA7, 0x10);
        CO_WAIT_MS(ptOpen_, STROBE_HI_MS);
        CMD(cogI_, 0xA7, 0x00);
        CO_WAIT_MS(ptOpen_, STROBE_LO_MS); // this strobe commits the OTP waveform; 0xB0 latches 10 ms after it
    }
    CMD(BOTH, 0x60, 0x25); // TCON
    CMD(LEFT, 0x61, 0x01); // STV_DIR (master only)
    CMD(BOTH, 0x02, 0x00); // VCOM
    if (!cfg_.slowWaveform)
    { // one chance per session: rewriting it re-latches the slow one. Staged in the DMA buffer (the LUT is in flash).
        if (sessionCalm_)
            calmTable(chunk_, cfg_.calmDriveFrames);
        else
            memcpy(chunk_, LUT, sizeof LUT);
        cmd(BOTH, 0xB0, chunk_, sizeof LUT);
    }
    CO_AWAIT(ptOpen_, dcdcCo());
    CO_END(ptOpen_);
}

bool EpdDisplay::closeCo()
{
    CO_BEGIN(ptClose_);
    CMD(BOTH, 0x09, 0x7F); // DC-DC soft shutdown
    CMD(BOTH, 0x05, 0x7D);
    CMD(BOTH, 0x09, 0x00);
    CO_WAIT_MS(ptClose_, 200);
    CO_WAIT_BUSY(ptClose_, 2000);
    digitalWrite(pins_.dc, LOW);
    digitalWrite(pins_.csLeft, LOW);
    digitalWrite(pins_.rst, LOW);
    digitalWrite(pins_.csLeft, HIGH);
    CO_END(ptClose_);
}

bool EpdDisplay::sessionCo()
{ // open + first frame
    CO_BEGIN(ptSession_);
    CO_AWAIT(ptSession_, openCo());
    CO_AWAIT(ptSession_, pullCo());
    CO_END(ptSession_);
}

bool EpdDisplay::frameCo()
{ // later frame in an open session
    CO_BEGIN(ptFrame_);
    CO_AWAIT(ptFrame_, loadCo());
    CO_AWAIT(ptFrame_, pullCo());
    CO_END(ptFrame_);
}