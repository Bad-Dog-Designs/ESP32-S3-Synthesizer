#include "ADC.h"
#include <Arduino.h>

// ================================================================
// ADC.cpp
// 16-channel CD74HC4067 multiplexer on A0, plus A6/A7 as
// analog-threshold buttons.
//
// Pipeline fix (vs original):
//   The original code had a one-channel offset because the mux was
//   never pre-selected in ADC_Init(), and the nextChannel variable
//   caused reads to lag one step behind the mux address.
//   Fix: setMux(0) in ADC_Init(), simplified single-variable advance
//   in ADC_Task() so read and mux address are always in sync.
// ================================================================

// ---- Pin mapping ----
static const uint8_t muxPins[4] = { 4, 5, 6, 7 };   // S0..S3
#define ADC_PIN        A0
#define ADC_EXTRA_0    A6
#define ADC_EXTRA_1    A7

// ---- Config ----
// Threshold: minimum change in counts to register as a new value.
// Raised from 4 → 10 to suppress idle noise on mux channels.
#define ADC_THRESHOLD        10

// Settle delay after changing mux address (microseconds).
// With 1nF on the ADC_MUX line and ~2.5kΩ pot source impedance,
// RC ≈ 2.5µs. 20µs = ~8 time constants → >99% settled.
// Was 5µs (too short — produced ~470 centre instead of ~512).
#define ADC_SETTLE_US        20

// Oversampling: number of reads to average per channel.
// Was 4 to combat noise from the original 100nF mux output cap.
// With the cap replaced by 1nF and ADC_SETTLE_US=20 giving >99%
// settling, noise is well controlled and oversampling is no longer
// needed. Single sample restores full scan speed (~1.8ms per cycle).
#define ADC_OVERSAMPLE        1

// Analog threshold for A6/A7 used as digital buttons.
#define ADC_BUTTON_THRESHOLD 100

// ---- State ----
static ADC_Channel_t channels[ADC_NUM_CHANNELS];
static uint8_t       currentChannel = 0;

// ---- Extra button state ----
static uint8_t extraButtons     = 0;
static uint8_t lastExtraButtons = 0;
static uint8_t extraPressed     = 0;
static uint8_t extraReleased    = 0;

// ================================================================
// Internal helpers
// ================================================================

static inline void setMux(uint8_t ch)
{
    for (uint8_t i = 0; i < 4; i++)
        digitalWrite(muxPins[i], (ch >> i) & 0x01);
}

// Take ADC_OVERSAMPLE readings from a pin and return the average.
static uint16_t sampleAvg(uint8_t pin)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_OVERSAMPLE; i++)
        sum += analogRead(pin);
    return (uint16_t)(sum / ADC_OVERSAMPLE);
}

// ================================================================
// Public API
// ================================================================

void ADC_Init(void)
{
    for (uint8_t i = 0; i < 4; i++)
        pinMode(muxPins[i], OUTPUT);

    for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++)
    {
        channels[i].value     = 0;
        channels[i].lastValue = 0;
        channels[i].changed   = false;
    }

    // Pre-select channel 0 so the very first ADC_Task() read is correct.
    // Without this the mux is in an undefined state on the first read,
    // causing a one-channel offset across all 16 mux channels.
    currentChannel = 0;
    setMux(0);
    delayMicroseconds(ADC_SETTLE_US);
}

// ----------------------------------------------------------------
// ADC_Task — call once per loop().
//
// One channel is read per call (non-blocking).
// Pipeline: mux was set to currentChannel at the end of the
// previous call (or in ADC_Init for channel 0), so the input has
// had a full loop() cycle to settle before we sample it.
// ----------------------------------------------------------------
void ADC_Task(void)
{
    uint16_t val;

    // ---- Read current channel ----
    if (currentChannel < 16)
    {
        val = sampleAvg(ADC_PIN);
    }
    else if (currentChannel == 16)
    {
        val = sampleAvg(ADC_EXTRA_0);
    }
    else // 17
    {
        val = sampleAvg(ADC_EXTRA_1);
    }

    // ---- Store and detect change ----
    ADC_Channel_t *ch = &channels[currentChannel];
    ch->value = val;

    if (abs((int)val - (int)ch->lastValue) > ADC_THRESHOLD)
    {
        ch->changed   = true;
        ch->lastValue = val;
    }

    // ---- Advance to next channel ----
    currentChannel++;
    if (currentChannel >= ADC_NUM_CHANNELS)
        currentChannel = 0;

    // ---- Pre-select next mux channel so it can settle ----
    // A6/A7 (channels 16/17) are direct ADC pins — no mux setup needed.
    if (currentChannel < 16)
    {
        setMux(currentChannel);
        delayMicroseconds(ADC_SETTLE_US);
    }

    // ---- Extra analog button edge detection ----
    // Done here (after mux work) so it happens once per ADC_Task call.
    uint8_t state = 0;
    if (channels[16].value < ADC_BUTTON_THRESHOLD) state |= (1 << 0);
    if (channels[17].value < ADC_BUTTON_THRESHOLD) state |= (1 << 1);

    uint8_t changed  = state ^ lastExtraButtons;
    extraPressed     = changed & state;
    extraReleased    = changed & ~state;
    lastExtraButtons = state;
    extraButtons     = state;
}

// ----------------------------------------------------------------
bool ADC_GetChannelChanged(uint8_t ch)
{
    if (ch >= ADC_NUM_CHANNELS) return false;
    bool c = channels[ch].changed;
    channels[ch].changed = false;
    return c;
}

uint16_t ADC_GetChannelValue(uint8_t ch)
{
    if (ch >= ADC_NUM_CHANNELS) return 0;
    return channels[ch].value;
}

uint8_t ADC_GetExtraPressed(void)  { return extraPressed;  }
uint8_t ADC_GetExtraReleased(void) { return extraReleased; }
uint8_t ADC_GetExtraState(void)    { return extraButtons;  }
