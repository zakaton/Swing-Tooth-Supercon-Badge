#include <Arduino.h>
#include <NimBLEDevice.h>
#include <FastLED.h>

#include "EpdDisplay.h"

bool needsDraw = true;
bool needsClear = true;
bool needsLight = false;

EpdDisplay epd;

static constexpr uint8_t LED_PIN = GPIO_NUM_38;
static constexpr uint8_t LED_COUNT = 8;
static constexpr uint8_t LED_BRIGHTNESS = 64;

float latestX = 0.0f;
float latestY = 0.0f;

CRGB leds[LED_COUNT];

void setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LED_COUNT)
        return;

    leds[index] = CRGB(r, g, b);
    FastLED.show();
}

void setLedColor(uint8_t index, const CRGB &color)
{
    if (index >= LED_COUNT)
        return;

    leds[index] = color;
    FastLED.show();
}

void setupLeds()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear(true);
}

static constexpr uint8_t BUTTON_PINS[] = {
    GPIO_NUM_40,
    GPIO_NUM_39,
    GPIO_NUM_15,
    GPIO_NUM_16};

static constexpr size_t BUTTON_COUNT =
    sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);

volatile bool buttonPressed[BUTTON_COUNT] = {false};

// Interrupt callbacks
void IRAM_ATTR buttonISR0() { buttonPressed[0] = true; }
void IRAM_ATTR buttonISR1() { buttonPressed[1] = true; }
void IRAM_ATTR buttonISR2() { buttonPressed[2] = true; }
void IRAM_ATTR buttonISR3() { buttonPressed[3] = true; }

void setupButtons()
{
    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    attachInterrupt(BUTTON_PINS[0], buttonISR0, FALLING);
    attachInterrupt(BUTTON_PINS[1], buttonISR1, FALLING);
    attachInterrupt(BUTTON_PINS[2], buttonISR2, FALLING);
    attachInterrupt(BUTTON_PINS[3], buttonISR3, FALLING);
}

// BLE configuration
static constexpr char DEVICE_NAME[] = "Swing Tooth";
static constexpr char SERVICE_UUID[] = "F1A00001-0000-1000-8000-00805F9B34FB";
static constexpr char CHAR_UUID[] = "F1A00002-0000-1000-8000-00805F9B34FB";

constexpr uint16_t NUMBER_OF_POINTS = 1000;
uint16_t pointCount = 0;
uint16_t pointIndex = 0;
float xArray[NUMBER_OF_POINTS]{0};
float yArray[NUMBER_OF_POINTS]{0};

class WriteCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite(
        NimBLECharacteristic *characteristic,
        NimBLEConnInfo &connInfo) override
    {
        std::string value = characteristic->getValue();

        if (value.size() != 8)
        {
            Serial.printf("Invalid payload size: %u\n",
                          static_cast<unsigned>(value.size()));
            return;
        }

        float x;
        float y;

        memcpy(&x, value.data(), sizeof(float));
        memcpy(&y, value.data() + sizeof(float), sizeof(float));

        // Serial.printf("X: %.3f, Y: %.3f\n", x, y);

        xArray[pointIndex] = x;
        yArray[pointIndex] = y;

        pointCount++;
        pointIndex++;
        pointIndex %= NUMBER_OF_POINTS;
        pointCount = min(pointCount, NUMBER_OF_POINTS);

        latestX = x;
        latestY = y;

        needsDraw = true;
        needsLight = true;
    }
};

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onDisconnect(NimBLEServer *server,
                      NimBLEConnInfo &connInfo,
                      int reason) override
    {
        Serial.println("BLE disconnected. Restarting advertising...");
        NimBLEDevice::startAdvertising();
    }
};

void setup()
{
    Serial.begin(115200);
    delay(500);

    EpdConfig cfg;
    cfg.debug = false;
    if (!epd.begin(EpdPins(), cfg))
        Serial.println("EPD init failed (out of memory)");

    setupButtons();
    setupLeds();

    // Initialize BLE
    NimBLEDevice::init(DEVICE_NAME);

    // Request maximum MTU
    NimBLEDevice::setMTU(517);

    // Create server
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    // Create service
    NimBLEService *service = server->createService(SERVICE_UUID);

    // Create writable characteristic
    NimBLECharacteristic *characteristic =
        service->createCharacteristic(
            CHAR_UUID,
            NIMBLE_PROPERTY::WRITE);

    characteristic->setCallbacks(new WriteCallbacks());

    // Start service
    service->start();

    // Start advertising
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("BLE server advertising");
}

bool nextDrawn = false;

void drawFrame()
{
    if (needsClear)
    {
        epd.clear();
        needsClear = false;
    }

    uint16_t _pointIndex = pointIndex;
    for (uint16_t i = 0; i < pointCount; i++)
    {
        uint16_t x = (epd.WIDTH / 2) + (xArray[i] * 20);
        uint16_t y = (epd.HEIGHT / 2) + (yArray[i] * 50);
        uint16_t w = 20;
        uint16_t h = 20;

        if (_pointIndex == 0)
        {
            _pointIndex = NUMBER_OF_POINTS - 1;
        }
        else
        {
            _pointIndex--;
        }

        epd.fillRect(x, y, w, h);
    }
    pointIndex = 0;
    pointCount = 0;
}

void loop()
{
    epd.tick(); // advance the panel state machine (never blocks long)

    if (needsLight)
    {
        needsLight = false;
        FastLED.clear(true);
        float normalized = 1 - ((max(-30.0f, min(30.0f, latestX)) + 30.0f) / 60.0f);
        uint8_t index = normalized * (LED_COUNT - 1);

        float normalizedVelocity = (max(0.0f, min(2.0f, latestY))) / 2.0f;

        leds[index] = blend(
            CRGB::Green,
            CRGB::Red,
            (uint8_t)(normalizedVelocity * 255.0f));
        FastLED.show();
        // setLedColor(index, CRGB::Red);
    }

    // Draw the next frame as soon as the panel has finished reading the framebuffer,
    // overlapping drawing with the ~1 s refresh.
    if (epd.canDraw() && !nextDrawn && needsDraw)
    {
        needsDraw = false;
        drawFrame();
        nextDrawn = true;
    }
    // Start the refresh as soon as the panel is idle.
    if (nextDrawn && !epd.busy() && epd.refresh())
        nextDrawn = false;

    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        if (buttonPressed[i])
        {
            noInterrupts();
            buttonPressed[i] = false;
            interrupts();

            Serial.printf("Button %u pressed (GPIO %u)\n",
                          i,
                          BUTTON_PINS[i]);
            needsClear = true;
            needsDraw = true;
        }
    }
}