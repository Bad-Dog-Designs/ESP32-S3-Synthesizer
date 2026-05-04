#include "ADC.h"
#include "IO.h"
#include "Board.h"
#include "CAN_Comm.h"
#include "param_map.h"

// ================================================================
// main.ino  —  Oscillator board, ATmega328p
// ================================================================

// ----------------------------------------------------------------
// Debug modes — comment out both for production
//
// DEBUG_MODE     : button press / LED mapping
// DEBUG_ADC_CAL  : ADC raw value reporting (CAN suppressed)
//                  Set DEBUG_ADC_CHANNEL to a channel number to
//                  watch one channel, or -1 for all.
// ----------------------------------------------------------------
// #define DEBUG_MODE
// #define DEBUG_ADC_CAL

#define DEBUG_ADC_CHANNEL    -1
#define CAL_PRINT_THRESHOLD  15
#define CAL_DETUNE_ZERO_LOW  DETUNE_ZERO_BAND_LOW
#define CAL_DETUNE_ZERO_HIGH DETUNE_ZERO_BAND_HIGH

// ================================================================
// DEBUG_ADC_CAL
// ================================================================
#ifdef DEBUG_ADC_CAL

static const char * const chLabel[16] =
{
    "OSC1 ChaosR      ",  //  0
    "OSC1 PulseWidth  ",  //  1
    "OSC1 Fold/Bias   ",  //  2
    "OSC1 SH Rate/Rng ",  //  3
    "OSC1 Level       ",  //  4
    "OSC1 Waveform    ",  //  5
    "OSC1 Detune/Phase",  //  6
    "OSC1 Octave      ",  //  7
    "OSC2 Waveform    ",  //  8
    "OSC2 Octave      ",  //  9
    "OSC2 Detune/Phase",  // 10
    "OSC2 PulseWidth  ",  // 11
    "OSC2 ChaosR      ",  // 12
    "OSC2 Fold/Bias   ",  // 13
    "OSC2 SH Rate/Rng ",  // 14
    "OSC2 Level       ",  // 15
};

static uint16_t calLastPrinted[16];

static void calInit()
{
    for (uint8_t i = 0; i < 16; i++) calLastPrinted[i] = 9999;
}

static void calPrintChannel(uint8_t ch, uint16_t raw)
{
    Serial.print(F("CH"));
    if (ch < 10) Serial.print('0');
    Serial.print(ch);
    Serial.print(F("  ["));
    Serial.print(chLabel[ch]);
    Serial.print(F("]  raw="));
    if (raw < 1000) Serial.print(' ');
    if (raw <  100) Serial.print(' ');
    if (raw <   10) Serial.print(' ');
    Serial.print(raw);

    int16_t dist = (int16_t)raw - 512;
    Serial.print(F("  ("));
    if (dist >= 0) Serial.print('+');
    Serial.print(dist);
    Serial.print(F(")  "));

    if (ch == 6 || ch == 10)
    {
        if (raw >= CAL_DETUNE_ZERO_LOW && raw <= CAL_DETUNE_ZERO_HIGH)
        {
            Serial.print(F("[IN ZERO BAND] LED=SOLID"));
        }
        else
        {
            uint16_t edge = (raw < CAL_DETUNE_ZERO_LOW)
                            ? (CAL_DETUNE_ZERO_LOW - raw)
                            : (raw - CAL_DETUNE_ZERO_HIGH);
            uint32_t maxD = (raw < CAL_DETUNE_ZERO_LOW)
                            ? CAL_DETUNE_ZERO_LOW
                            : (1023 - CAL_DETUNE_ZERO_HIGH);
            uint32_t period = DETUNE_FLASH_SLOW_MS -
                              ((uint32_t)edge * (DETUNE_FLASH_SLOW_MS - DETUNE_FLASH_FAST_MS)) / maxD;
            Serial.print(F("flash ~")); Serial.print(period); Serial.print(F("ms"));
        }
    }

    Serial.print(F("  (")); Serial.print((raw * 100UL) / 1023); Serial.println(F("%)"));
}

#endif // DEBUG_ADC_CAL

// ================================================================
// DEBUG_MODE (button / LED mapping)
// ================================================================
#ifdef DEBUG_MODE

static const char * const btnNames[BTN_COUNT] =
{
    "OSC1_SH", "OSC1_FOLD", "OSC1_CLONE", "OSC1_SOLO", "OSC1_SYNC",
    "OSC2_SYNC", "OSC2_SOLO", "OSC2_CLONE", "OSC2_FOLD", "OSC2_SH"
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
    for (int8_t i = 7; i >= 0; i--) Serial.print((gpa >> i) & 1);
    Serial.print(F("  GPB:"));
    for (int8_t i = 7; i >= 0; i--) Serial.print((gpb >> i) & 1);
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
            Serial.println(CAN_GetToggleState(btn) ? F("ON") : F("OFF"));
            break;
        case BTN_MODE_MOMENTARY:
            Serial.println(F("  MOMENTARY"));
            break;
        case BTN_MODE_MODIFIER:
        {
            uint8_t s = CAN_GetModifierState(btn);
            Serial.print(F("  MODIFIER state ")); Serial.print(s);
            if (s==0) Serial.println(F(" (off)"));
            else if (s==1) Serial.println(F(" (on, primary)"));
            else Serial.println(F(" (on, secondary)"));
            break;
        }
    }
    debugPrintIC3(CAN_GetLEDs());
}

static void debugPrintExtra(uint8_t which)
{
    Serial.println(F("----"));
    Serial.print(F("A")); Serial.print(which == 0 ? '6' : '7');
    Serial.print(F(" OSC")); Serial.print(which == 0 ? '1' : '2');
    Serial.print(F(": ")); Serial.println(CAN_GetExtraState(which) ? F("PHASE") : F("DETUNE"));
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
    Serial.print(Board_GetID());
    Serial.print(F("  OSC offset="));
    Serial.println(Board_GetOscOffset());

#ifdef DEBUG_ADC_CAL
    calInit();
    Serial.println(F("ADC CAL MODE (CAN suppressed)"));
    Serial.print(F("Zero band: ")); Serial.print(CAL_DETUNE_ZERO_LOW);
    Serial.print(F("-")); Serial.println(CAL_DETUNE_ZERO_HIGH);
#endif

    IO_SetLEDs(CAN_GetLEDs());
    IO_SetIC4LEDs(CAN_GetIC4LEDs());
}

void loop()
{
    ADC_Task();
    IO_Task();

#ifndef DEBUG_ADC_CAL
    CAN_Task();
    CAN_LEDTask();
    CAN_ModifierHoldTask();
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

    for (uint8_t ch = 0; ch < 16; ch++)
    {
        // Ranged channels (switches) are handled by CAN_ProcessRangedChannels()
        // every loop — their settle timer must not be gated on change detection.
        if (ch == 5 || ch == 7 || ch == 8 || ch == 9) continue;

        if (ADC_GetChannelChanged(ch))
            CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
    }

    // Selector switches — called every loop so 40ms settle timer can elapse
    CAN_ProcessRangedChannels();

    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        if (IO_IsButtonPressed((Button_t)b))
        {
            CAN_ProcessButton((Button_t)b);
#ifdef DEBUG_MODE
            IO_SetLEDs(CAN_GetLEDs());
            IO_SetIC4LEDs(CAN_GetIC4LEDs());
            debugPrintButton((Button_t)b);
#endif
        }
    }

    // Modifier button release — short press toggles primary/secondary while ON
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        if (IO_IsButtonReleased((Button_t)b))
            CAN_ProcessButtonRelease((Button_t)b);
    }

    uint8_t extra = ADC_GetExtraPressed();
    if (extra & 0x01)
    {
        CAN_ProcessExtraButton(0);
#ifdef DEBUG_MODE
        IO_SetLEDs(CAN_GetLEDs()); debugPrintExtra(0);
#endif
    }
    if (extra & 0x02)
    {
        CAN_ProcessExtraButton(1);
#ifdef DEBUG_MODE
        IO_SetLEDs(CAN_GetLEDs()); debugPrintExtra(1);
#endif
    }

    IO_SetLEDs(CAN_GetLEDs());
    IO_SetIC4LEDs(CAN_GetIC4LEDs());

#endif // DEBUG_ADC_CAL
}
