#include "ADC.h"
#include "IO.h"
#include "Board.h"
#include "CAN_Comm.h"
#include "param_map.h"

// ================================================================
// main.ino — Commons PCB, ATmega328p
// Filter / Ring Mod / LFO / ADSR / Unison controls
// ================================================================

// ----------------------------------------------------------------
// Debug modes — comment out both for production
//
// DEBUG_MODE     : button press / LED mapping
// DEBUG_ADC_CAL  : ADC raw value reporting (CAN suppressed)
//                  Use DEBUG_ADC_CHANNEL=-1 for all channels,
//                  or a specific channel number to focus.
//                  Run this to calibrate CH5/CH10/CH15 switch ranges.
// ----------------------------------------------------------------
//#define DEBUG_MODE
//#define DEBUG_ADC_CAL

#define DEBUG_ADC_CHANNEL    -1
#define CAL_PRINT_THRESHOLD  15

// ================================================================
// DEBUG_ADC_CAL
// ================================================================
#ifdef DEBUG_ADC_CAL

static const char * const chLabel[16] =
{
    "Unison Detune    ",  //  0
    "ADSR Release     ",  //  1
    "ADSR Sustain     ",  //  2
    "ADSR Decay       ",  //  3
    "ADSR Attack      ",  //  4
    "LFO1 Waveform    ",  //  5  *** calibrate switch ***
    "LFO1 Delay       ",  //  6
    "LFO1 Rate        ",  //  7
    "LFO2 Rate        ",  //  8
    "LFO2 Delay       ",  //  9
    "LFO2 Waveform    ",  // 10  *** calibrate switch ***
    "Env/Track Amount ",  // 11
    "Filter Resonance ",  // 12
    "Filter Frequency ",  // 13
    "Ring Mod Level   ",  // 14
    "Ring Mod Pairs   ",  // 15  *** calibrate switch ***
};

static uint16_t calLastPrinted[16];
static void calInit() { for (uint8_t i=0;i<16;i++) calLastPrinted[i]=9999; }

static void calPrintChannel(uint8_t ch, uint16_t raw)
{
    Serial.print(F("CH"));
    if (ch < 10) Serial.print('0');
    Serial.print(ch);
    Serial.print(F("  ["));
    Serial.print(chLabel[ch]);
    Serial.print(F("]  raw="));
    if (raw < 1000) Serial.print(' ');
    if (raw  < 100) Serial.print(' ');
    if (raw   < 10) Serial.print(' ');
    Serial.print(raw);
    Serial.print(F("  ("));
    Serial.print((raw * 100UL) / 1023);
    Serial.println(F("%)"));
}
#endif // DEBUG_ADC_CAL

// ================================================================
// DEBUG_MODE (button / LED mapping)
// ================================================================
#ifdef DEBUG_MODE

static const char * const btnNames[BTN_COUNT] =
{
    "FILTER_MODE", "MOD_MUTE", "RING_ENABLE",
    "LFO2_TRIG", "ENV_TRACK", "POLY_MODE", "LFO1_TRIG"
};

static void debugPrintIC3(uint16_t ledWord)
{
    uint8_t gpa = ledWord & 0xFF;
    uint8_t gpb = ledWord >> 8;
    Serial.print(F("  IC3: 0x"));
    if (ledWord < 0x1000) Serial.print('0');
    if (ledWord < 0x0100) Serial.print('0');
    if (ledWord < 0x0010) Serial.print('0');
    Serial.print(ledWord, HEX);
    Serial.print(F("  GPA:"));
    for (int8_t i=7;i>=0;i--) Serial.print((gpa>>i)&1);
    Serial.print(F("  GPB:"));
    for (int8_t i=7;i>=0;i--) Serial.print((gpb>>i)&1);
    Serial.println();
}

static void debugPrintButton(Button_t btn)
{
    const BTN_Map_t *e = &btnMap[btn];
    Serial.println(F("----"));
    Serial.print(F("BTN: ")); Serial.println(btnNames[btn]);
    switch (e->mode)
    {
        case BTN_MODE_TOGGLE:
            Serial.print(F("  TOGGLE state: "));
            Serial.println(CAN_GetToggleState(btn) ? F("ON (Env)") : F("OFF (Track)"));
            break;
        case BTN_MODE_MOMENTARY:
            Serial.println(F("  MOMENTARY"));
            break;
        case BTN_MODE_CYCLER:
        {
            uint8_t s = CAN_GetCyclerState(btn);
            Serial.print(F("  CYCLER state: ")); Serial.print(s);
            if (btn == BTN_FILTER_MODE)
            {
                if (s==0) Serial.println(F(" (Lowpass)"));
                else if (s==1) Serial.println(F(" (Bandpass)"));
                else Serial.println(F(" (Highpass)"));
            }
            else if (btn == BTN_POLY_MODE)
            {
                if (s==0) Serial.println(F(" (Poly)"));
                else if (s==1) Serial.println(F(" (Duo)"));
                else Serial.println(F(" (Unison)"));
            }
            break;
        }
        default: break;
    }
    debugPrintIC3(CAN_GetLEDs());
}
#endif // DEBUG_MODE

// ================================================================
// setup / loop
// ================================================================

void setup()
{
    Serial.begin(115200);

    Board_Init();
    ADC_Init();
    IO_Init();

    if (!CAN_Init())
    {
        Serial.println(F("CAN init failed"));
        while (true) { IO_SetLEDs(0xFFFF); delay(300); IO_SetLEDs(0x0000); delay(300); }
    }

    Board_SendAnnouncement();

    Serial.print(F("CAN init OK  Board ID="));
    Serial.println(Board_GetID());

#ifdef DEBUG_ADC_CAL
    calInit();
    Serial.println(F("ADC CAL MODE (CAN suppressed)"));
    Serial.println(F("Calibrate CH5/CH10/CH15 switch positions."));
    Serial.println(F("Step through each position, note stable raw values."));
    Serial.println(F("Update lfoWaveformRanges[] and ringModPairsRanges[]"));
    Serial.println(F("in CAN_Comm.cpp with midpoint-based windows."));
#endif

    IO_SetLEDs(CAN_GetLEDs());
}

void loop()
{
    ADC_Task();
    IO_Task();

#ifndef DEBUG_ADC_CAL
    CAN_Task();
#endif

    // ================================================================
    // ADC calibration mode
    // ================================================================
#ifdef DEBUG_ADC_CAL

    for (uint8_t ch = 0; ch < 16; ch++)
    {
        if (DEBUG_ADC_CHANNEL >= 0 && ch != (uint8_t)DEBUG_ADC_CHANNEL)
            continue;

        uint16_t raw = ADC_GetChannelValue(ch);
        int32_t diff = (int32_t)raw - (int32_t)calLastPrinted[ch];
        if (diff < 0) diff = -diff;
        if (diff >= CAL_PRINT_THRESHOLD)
        {
            calLastPrinted[ch] = raw;
            calPrintChannel(ch, raw);
        }
    }

#else
    // ================================================================
    // Normal operation
    // ================================================================

    // Continuous ADC channels (all except ranged switch channels)
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        if (ch == 5 || ch == 10 || ch == 15) continue; // handled by CAN_ProcessRangedChannels
        if (ADC_GetChannelChanged(ch))
            CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
    }

    // Switch channels — polled every loop for settle timer
    CAN_ProcessRangedChannels();

    // Buttons
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        if (IO_IsButtonPressed((Button_t)b))
        {
            CAN_ProcessButton((Button_t)b);
#ifdef DEBUG_MODE
            IO_SetLEDs(CAN_GetLEDs());
            debugPrintButton((Button_t)b);
#endif
        }
    }

    IO_SetLEDs(CAN_GetLEDs());

#endif // DEBUG_ADC_CAL
}
