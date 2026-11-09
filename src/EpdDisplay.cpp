#include "EpdDisplay.h"

namespace
{
constexpr int LEFT = 1, RIGHT = 2, BOTH = 3;
constexpr int W = EpdDisplay::WIDTH;
constexpr int H = EpdDisplay::HEIGHT;

// Session-open delays (ms). Lower them one at a time and check the image.
constexpr uint32_t RESET_PRE_MS = 200;
constexpr uint32_t RESET_HIGH_MS = 20;
constexpr uint32_t RESET_LOW_MS = 200;
constexpr uint32_t RESET_POST_MS = 200;
constexpr uint32_t POWER_MS = 200;
constexpr uint32_t STROBE_MS = 100;
constexpr uint32_t LUT_SETTLE_MS = 500; // before the 0xB0 LUT write (required)

uint8_t revTab[256]; // bit-reverse lookup

// Short waveform: 5 tables (paper, ink, red, unused, VCOM) x 10 groups x 8 bytes.
const uint8_t LUT[400] = {
    // table 0: paper (white)
    0x99, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 1: ink (black)
    0x66, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 2: red
    0xEE, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0xAA, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    // table 3: unused
    0x00, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 4: VCOM
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
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
#define CO_YIELD(pt)   \
    do                 \
    {                  \
        pt = __LINE__; \
        return false;  \
    case __LINE__:;    \
    } while (0)
#define CO_WAIT_MS(pt, ms)            \
    do                                \
    {                                 \
        coT0_ = millis();             \
        coDur_ = (ms);                \
        __attribute__((fallthrough)); \
    case __LINE__:                    \
        pt = __LINE__;                \
        if (millis() - coT0_ < coDur_) \
            return false;             \
    } while (0)
#define CO_WAIT_BUSY(pt, ms)          \
    do                                \
    {                                 \
        busyBegin(ms);                \
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
    free(fb_);
    free(buf_);
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

    if (!fb_)
        fb_ = (uint8_t *)malloc(FRAMEBUFFER_BYTES);
    if (!buf_)
        buf_ = (uint8_t *)malloc(HALF_BYTES);
    if (!fb_ || !buf_)
        return false;
    memset(fb_, 0, FRAMEBUFFER_BYTES);

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
    spi_.begin(pins_.sck, -1, pins_.mosi, -1);

    job_ = JOB_NONE;
    sessionOpen_ = false;
    fbInUse_ = false;
    return true;
}

// ===========================================================================
// Public control API
// ===========================================================================
bool EpdDisplay::busy() const { return job_ != JOB_NONE; }
bool EpdDisplay::canDraw() const { return fb_ && !fbInUse_; }

bool EpdDisplay::refresh()
{
    if (!fb_ || job_ != JOB_NONE)
        return false;
    ldBlack_ = fb_;
    fbInUse_ = true;
    if (!sessionOpen_)
    {
        ldWithRed_ = true;
        pullsLeft_ = cfg_.firstPulls;
        job_ = JOB_SESSION;
    }
    else
    {
        ldWithRed_ = cfg_.clearRedEachFrame;
        pullsLeft_ = cfg_.framePulls;
        job_ = JOB_FRAME;
    }
    return true;
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
        break;
    case JOB_FRAME:
        framesDone_++;
        break;
    case JOB_CLOSE:
        sessionOpen_ = false;
        job_ = JOB_NONE;
        return;
    default:
        return;
    }
    idleT0_ = millis();
    // Session used up: power down right away; the next refresh() opens a fresh one.
    job_ = (framesDone_ >= cfg_.framesPerSession) ? JOB_CLOSE : JOB_NONE;
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
        fb_[y * (W / 8) + (x >> 3)] |= m;
    else
        fb_[y * (W / 8) + (x >> 3)] &= ~m;
}

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
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
        {
            uint8_t m = 0x80 >> (i & 7);
            if (ink)
                fb_[j * (W / 8) + (i >> 3)] |= m;
            else
                fb_[j * (W / 8) + (i >> 3)] &= ~m;
        }
}

// ===========================================================================
// Low-level SPI (blocking, but each call is short)
// ===========================================================================
void EpdDisplay::cmd(int cog, uint8_t index, const uint8_t *data, size_t n)
{
    spi_.beginTransaction(SPISettings(cfg_.spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(pins_.dc, LOW);
    digitalWrite(pins_.csLeft, !(cog & LEFT));
    digitalWrite(pins_.csRight, !(cog & RIGHT));
    spi_.writeBytes(&index, 1);
    digitalWrite(pins_.csLeft, HIGH);
    digitalWrite(pins_.csRight, HIGH);
    digitalWrite(pins_.dc, HIGH);
    digitalWrite(pins_.csLeft, !(cog & LEFT));
    digitalWrite(pins_.csRight, !(cog & RIGHT));
    while (n)
    {
        size_t k = n < 4096 ? n : 4096;
        spi_.writeBytes(data, k);
        data += k;
        n -= k;
    }
    digitalWrite(pins_.csLeft, HIGH);
    digitalWrite(pins_.csRight, HIGH);
    spi_.endTransaction();
}

// plane NULL = all white (clears that RAM).
void EpdDisplay::sendHalf(int cog, uint8_t index, const uint8_t *plane)
{
    if (plane)
    {
        const uint8_t *src = plane + (cog == RIGHT ? W / 16 : 0);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W / 16; x++)
                buf_[y * (W / 16) + x] = revTab[src[y * (W / 8) + x]];
    }
    else
    {
        memset(buf_, 0, HALF_BYTES);
    }
    CMD(cog, 0x12, 0x3B, 0x00, 0x14); // RAM_RW
    cmd(cog, index, buf_, HALF_BYTES);
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

// Frame into CoG RAM, one half-plane per tick (yields between SPI bursts).
bool EpdDisplay::loadCo()
{
    CO_BEGIN(ptLoad_);
    CMD(BOTH, 0x13, 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02); // DUW: 60 bytes x 672 lines
    CMD(BOTH, 0x90, 0x00, 0x3B, 0x00, 0xA9);             // DRFW: whole panel
    for (cogI_ = LEFT; cogI_ <= RIGHT; cogI_++)
    {
        sendHalf(cogI_, 0x10, ldBlack_);
        CO_YIELD(ptLoad_);
        if (ldWithRed_)
        {
            sendHalf(cogI_, 0x11, NULL);
            CO_YIELD(ptLoad_);
        }
    }
    fbInUse_ = false; // the framebuffer is free again
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

// Run the waveform pullsLeft_ times on what is in RAM (~1.2 s each).
bool EpdDisplay::pullCo()
{
    CO_BEGIN(ptPull_);
    while (pullsLeft_ > 0)
    {
        CMD(BOTH, 0x15, 0x3C);
        CO_WAIT_MS(ptPull_, 5);
        CO_WAIT_BUSY(ptPull_, pullTimeoutMs_);
        if (cfg_.debug)
            Serial.printf("  pull: BUSY left %d ms, right %d ms%s\n", busyL_, busyR_, busyTimedOut_ ? " (TIMEOUT)" : "");
        pullsLeft_--;
    }
    CO_END(ptPull_);
}

// Reset, load, register sequence, short waveform, DC-DC up. ldBlack_ must be set by the caller.
bool EpdDisplay::openCo()
{
    CO_BEGIN(ptOpen_);
    CO_WAIT_MS(ptOpen_, RESET_PRE_MS);
    digitalWrite(pins_.rst, HIGH);
    CO_WAIT_MS(ptOpen_, RESET_HIGH_MS);
    digitalWrite(pins_.rst, LOW);
    CO_WAIT_MS(ptOpen_, RESET_LOW_MS);
    digitalWrite(pins_.rst, HIGH);
    CO_WAIT_MS(ptOpen_, RESET_POST_MS);
    digitalWrite(pins_.csLeft, HIGH);
    digitalWrite(pins_.csRight, HIGH);
    CO_WAIT_MS(ptOpen_, 5);
    CMD(BOTH, 0x70, 0x71, 0x36); // booster setting (otherwise a refresh draws ~2 A)

    ldWithRed_ = true; // a new session always zeroes the red RAM
    CO_AWAIT(ptOpen_, loadCo());

    CMD(BOTH, 0x05, 0x7D);
    CO_WAIT_MS(ptOpen_, POWER_MS);
    CMD(BOTH, 0x05, 0x00);
    CO_WAIT_MS(ptOpen_, 10);
    CMD(BOTH, 0xD8, 0x80); // MS_SYNC
    CMD(BOTH, 0xD6, 0x00); // BVSS
    CMD(BOTH, 0xA7, 0x10);
    CO_WAIT_MS(ptOpen_, STROBE_MS);
    CMD(BOTH, 0xA7, 0x00);
    CO_WAIT_MS(ptOpen_, STROBE_MS);
    CMD(BOTH, 0x03, 0x00, 0x11); // OSC
    for (cogI_ = LEFT; cogI_ <= RIGHT; cogI_++)
    {
        CMD(cogI_, 0x44, 0x00);
        CMD(cogI_, 0x45, 0x80);
        CMD(cogI_, 0xA7, 0x10);
        CO_WAIT_MS(ptOpen_, STROBE_MS);
        CMD(cogI_, 0xA7, 0x00);
        CO_WAIT_MS(ptOpen_, STROBE_MS);
        CMD(cogI_, 0x44, 0x06);
        CMD(cogI_, 0x45, (uint8_t)(cfg_.slowWaveform ? 0x82 : 0x84)); // 25 C stock / 26 C short waveform
        CMD(cogI_, 0xA7, 0x10);
        CO_WAIT_MS(ptOpen_, STROBE_MS);
        CMD(cogI_, 0xA7, 0x00);
        CO_WAIT_MS(ptOpen_, cfg_.slowWaveform ? STROBE_MS : LUT_SETTLE_MS); // 0xB0 must come after this settles
    }
    CMD(BOTH, 0x60, 0x25); // TCON
    CMD(LEFT, 0x61, 0x01); // STV_DIR (master only)
    CMD(BOTH, 0x02, 0x00); // VCOM
    if (!cfg_.slowWaveform)
        cmd(BOTH, 0xB0, LUT, sizeof LUT); // once per session
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
