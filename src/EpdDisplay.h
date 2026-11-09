// EpdDisplay - non-blocking driver for the 9.7" 960x672 black/white/red e-paper (two CoGs).
//
// Black/white only. Two waveforms:
//   * calm (default): the old frame sits in RAM 0x11, the new one in 0x10, and only pixels that
//     changed are driven. ~240 ms per frame, no flashing. The driver remembers what the panel shows,
//     so refresh() with an unchanged framebuffer costs nothing.
//   * fast (calmWaveform = false): the original short LUT, ~1 s per pull, rewrites everything.
//
// Usage:
//     EpdDisplay epd;
//
//     void setup() { epd.begin(); }
//
//     void loop() {
//         epd.tick();                          // must be called often; never blocks long
//         if (epd.canDraw()) {                 // framebuffer is free to modify
//             epd.clear();
//             epd.fillRect(100, 100, 200, 50);
//         }
//         if (!epd.busy() && wantNewFrame) epd.refresh();
//     }
//
// Nothing here calls delay(). All waiting is done with micros() deadlines and BUSY-pin polling
// inside small coroutines advanced by tick(). Plane data goes out over DMA in 3840-byte chunks that
// are converted (bit-reversed) just in time, so tick() never blocks for more than ~100 us.
//
// Power sessions are handled for you: the first refresh() powers the panel up, later refreshes
// reuse the session, and the driver powers down by itself after `framesPerSession` frames (and
// optionally after `autoPowerOffMs` of idling). The next refresh() simply opens a new session.
//
// NOTE: this driver now talks to the ESP-IDF SPI master driver directly (SPI2_HOST / FSPI), so the
// Arduino `SPI` object on FSPI must not be used at the same time.
#pragma once

#include <Arduino.h>
#include "driver/spi_master.h"

struct EpdPins
{
    int csLeft = 13;
    int csRight = 3;
    int busyLeft = 12;
    int busyRight = 8;
    int dc = 10;
    int rst = 11;
    int sck = 21;
    int mosi = 47;
};

struct EpdConfig
{
    uint32_t spiHz = 20000000;     // the reference firmware runs 20 MHz
    bool calmWaveform = true;      // changed-pixels-only waveform (see above); needs one extra 80 KB buffer
    int calmDriveFrames = 9;       // 1..9: frames of drive voltage per changed pixel (9 = full-strength)
    int cleanPulls = 3;            // calm mode: passes of the full LUT used for the first frame / after invalidate()
    int firstPulls = 0;            // fast mode: passes on the first frame of a session (1 = ghosty, 3 = clean)
    int framePulls = 1;            // fast mode: passes on every later frame
    int framesPerSession = 20;     // frames after the first before the panel is powered down; 0 = every frame
    uint32_t autoPowerOffMs = 0;   // power down after this long idle with a session open; 0 = never
    bool clearRedEachFrame = true; // fast mode: rewrite the (empty) red RAM each frame; prevents ghosting
    bool slowWaveform = false;     // panel's own ~30 s waveform (no custom LUT); forces 1 pull and a fresh session per frame
    bool useBusyRight = true;      // also wait on the right CoG's BUSY pin (the reference firmware waits on both)
    bool debug = false;            // print per-refresh timing to Serial
};

class EpdDisplay
{
public:
    static constexpr int WIDTH = 960;
    static constexpr int HEIGHT = 672;
    static constexpr size_t FRAMEBUFFER_BYTES = WIDTH * HEIGHT / 8; // 1 bpp, MSB = leftmost pixel, 1 = ink

    EpdDisplay() = default;
    EpdDisplay(const EpdDisplay &) = delete;
    EpdDisplay &operator=(const EpdDisplay &) = delete;
    ~EpdDisplay();

    // Allocates buffers and configures pins/SPI. Returns false if allocation or SPI setup fails.
    bool begin(const EpdPins &pins = EpdPins(), const EpdConfig &config = EpdConfig());

    // ---- Panel control -------------------------------------------------------------------
    void tick();          // advance the panel state machine; call every loop()
    bool busy() const;    // a panel job is running; refresh() will be rejected
    bool canDraw() const; // framebuffer is not being read by the panel and may be modified
    bool refresh();       // show the framebuffer; returns false if busy() (framebuffer is then untouched)
                          // calm mode: returns true immediately (no job) if nothing changed
    bool powerOff();      // end the session now (booster off); false if busy()
    bool poweredOn() const { return sessionOpen_; }
    void invalidate();    // calm mode: forget what the panel shows; the next refresh() does a full clean pass

    // ---- Drawing (1 = ink/black, 0 = paper/white) ---------------------------------------
    int width() const { return WIDTH; }
    int height() const { return HEIGHT; }
    void clear(bool ink = false);
    void setPixel(int x, int y, bool ink = true);
    void fillRect(int x, int y, int w, int h, bool ink = true);
    uint8_t *framebuffer() { return fb_; } // raw access for your own graphics code / bitmaps

private:
    enum Job : uint8_t
    {
        JOB_NONE,
        JOB_SESSION, // power up + show first frame
        JOB_FRAME,   // show a frame in an open session
        JOB_CLOSE    // power down
    };

    // low-level
    void cmd(int cog, uint8_t index, const uint8_t *data, size_t n);
    void spiTx(const uint8_t *d, size_t n);
    void select(int cog);
    void deselect();
    void streamBegin(int cog, uint8_t index, const uint8_t *plane);
    bool streamPoll();
    void fillChunk(uint8_t *dst, int rows);
    void busyBegin(uint32_t timeoutMs);
    bool busyPoll();
    void startJob();
    void finishJob();

    // coroutines: return true when finished, false = call again
    bool loadCo();
    bool dcdcCo();
    bool pullCo();
    bool openCo();
    bool closeCo();
    bool sessionCo();
    bool frameCo();

    EpdPins pins_;
    EpdConfig cfg_;
    spi_device_handle_t spi_ = nullptr;
    bool busOwned_ = false;
    uint32_t pullTimeoutMs_ = 5000;

    uint8_t *fb_ = nullptr;    // black plane (what you draw into)
    uint8_t *shown_ = nullptr; // calm mode: copy of what the panel shows (becomes RAM 0x11 on a new session)
    uint8_t *chunk_ = nullptr; // 2 x DMA-capable chunk buffers for streaming a half plane
    bool calm_ = false;        // calm waveform in use
    bool shownValid_ = false;  // shown_ really matches the film

    // job state
    Job job_ = JOB_NONE;
    bool sessionOpen_ = false;
    bool sessionCalm_ = false; // the open (or opening) session latched the calm LUT
    bool closeAfter_ = false;  // close the session as soon as this job is done
    bool reopen_ = false;      // after the close job, start the pending frame as a new session
    bool fbInUse_ = false;
    int framesDone_ = 0;
    uint32_t idleT0_ = 0;

    // job parameters
    const uint8_t *ldBlack_ = nullptr; // -> RAM 0x10
    const uint8_t *ld11_ = nullptr;    // -> RAM 0x11 (nullptr = white), only if ldSend11_
    bool ldSend11_ = false;
    int pullsLeft_ = 0;

    // streaming state
    const uint8_t *strSrc_ = nullptr;
    int strRow_ = 0, inflight_ = 0, slot_ = 0;
    spi_transaction_t trans_[2];

    // coroutine state (everything that must survive a yield lives here)
    int ptLoad_ = 0, ptDcdc_ = 0, ptPull_ = 0, ptOpen_ = 0, ptClose_ = 0, ptSession_ = 0, ptFrame_ = 0;
    int cogI_ = 0, v_ = 0;
    uint32_t coT0_ = 0, coDur_ = 0;
    int busyL_ = -1, busyR_ = -1; // ms each CoG stayed busy (-1 = not yet)
    uint32_t busyT0_ = 0, busyTimeout_ = 0;
    bool busyTimedOut_ = false;
};
