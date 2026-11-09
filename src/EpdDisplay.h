// EpdDisplay - non-blocking driver for the 9.7" 960x672 black/white/red e-paper (two CoGs).
//
// Fast short-waveform refresh, black/white only.
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
// Nothing here calls delay(). All waiting is done with millis() deadlines and BUSY-pin polling
// inside small coroutines advanced by tick(). The only blocking operation is one SPI burst
// (half a plane, ~20 ms at 16 MHz), so loop() is never stalled longer than that.
//
// Power sessions are handled for you: the first refresh() powers the panel up, later refreshes
// reuse the session, and the driver powers down by itself after `framesPerSession` frames (and
// optionally after `autoPowerOffMs` of idling). The next refresh() simply opens a new session.
#pragma once

#include <Arduino.h>
#include <SPI.h>

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
    uint32_t spiHz = 16000000;     // reference design uses 8 MHz
    int firstPulls = 0;            // waveform passes on the first frame of a session (1 = fastest/ghosty, 3 = clean)
    int framePulls = 1;            // waveform passes on every later frame
    int framesPerSession = 20;     // frames after the first before the panel is powered down; 0 = every frame
    uint32_t autoPowerOffMs = 0;   // power down after this long idle with a session open; 0 = never
    bool clearRedEachFrame = true; // rewrite the (empty) red RAM each fast frame; this is what prevents ghosting
    bool slowWaveform = false;     // panel's own ~30 s waveform (no custom LUT); forces 1 pull and a fresh session per frame
    bool useBusyRight = false;     // also wait on the right CoG's BUSY pin
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

    // Allocates buffers and configures pins/SPI. Returns false if allocation fails.
    bool begin(const EpdPins &pins = EpdPins(), const EpdConfig &config = EpdConfig());

    // ---- Panel control -------------------------------------------------------------------
    void tick();          // advance the panel state machine; call every loop()
    bool busy() const;    // a panel job is running; refresh() will be rejected
    bool canDraw() const; // framebuffer is not being read by the panel and may be modified
    bool refresh();       // show the framebuffer; returns false if busy() (framebuffer is then untouched)
    bool powerOff();      // end the session now (booster off); false if busy()
    bool poweredOn() const { return sessionOpen_; }

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

    static constexpr size_t HALF_BYTES = FRAMEBUFFER_BYTES / 2; // one CoG's share of a plane

    // low-level
    void cmd(int cog, uint8_t index, const uint8_t *data, size_t n);
    void sendHalf(int cog, uint8_t index, const uint8_t *plane);
    void busyBegin(uint32_t timeoutMs);
    bool busyPoll();
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
    SPIClass spi_{FSPI};
    uint32_t pullTimeoutMs_ = 5000;

    uint8_t *fb_ = nullptr;  // black plane
    uint8_t *buf_ = nullptr; // scratch: one CoG's half plane

    // job state
    Job job_ = JOB_NONE;
    bool sessionOpen_ = false;
    bool fbInUse_ = false;
    int framesDone_ = 0;
    uint32_t idleT0_ = 0;

    // job parameters
    const uint8_t *ldBlack_ = nullptr;
    bool ldWithRed_ = false;
    int pullsLeft_ = 0;

    // coroutine state (everything that must survive a yield lives here)
    int ptLoad_ = 0, ptDcdc_ = 0, ptPull_ = 0, ptOpen_ = 0, ptClose_ = 0, ptSession_ = 0, ptFrame_ = 0;
    int cogI_ = 0, loadCog_ = 0, v_ = 0;
    uint32_t coT0_ = 0, coDur_ = 0;
    int busyL_ = -1, busyR_ = -1; // ms each CoG stayed busy (-1 = not yet)
    uint32_t busyT0_ = 0, busyTimeout_ = 0;
    bool busyTimedOut_ = false;
};
