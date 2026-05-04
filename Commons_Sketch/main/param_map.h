#pragma once

// ================================================================
// param_map.h — Commons PCB
// The single "truth table" for all hardware → CAN mappings.
// ================================================================

#include "synth_params.h"
#include "IO.h"
#include <stdint.h>

// ================================================================
// Encoding types
// ================================================================
typedef enum
{
    ENCODE_LINEAR,   // 10-bit → 0-65535, ESP32 decodes linearly
    ENCODE_EXP,      // 10-bit → 0-65535, ESP32 decodes exponentially
    ENCODE_RANGED,   // Resistor-ladder switch — matched to StepRange_t table
    ENCODE_DETUNE,   // Unused on Commons — kept for shared CAN_Comm.cpp compat
} EncodeType_t;

// ================================================================
// Resistor ladder step range
// ================================================================
typedef struct
{
    uint16_t adcMin;
    uint16_t adcMax;
    uint16_t canValue;
} StepRange_t;

// ================================================================
// Switch settle time (ENCODE_RANGED channels)
// ================================================================
#define SWITCH_SETTLE_MS   40

// ================================================================
// Pot catch thresholds (dual-function channels, e.g. CH11 Env/Track)
//
// CATCH_WINDOW: if the pot is already within this many ADC counts of
//   the last-sent value when the button is pressed, skip catch entirely
//   and start sending immediately. Prevents any delay when the two pot
//   positions happen to be close.
//
// CATCH_MOVE_THRESHOLD: if the pot moves this many counts from its
//   position at button-press time, release the catch and start sending
//   from wherever the pot now is. The user is clearly intending to
//   adjust — a hard cross would take too long. The brief discontinuity
//   at release is bounded by this value (~8% of full range).
// ================================================================
#define CATCH_WINDOW          50   // ADC counts (~5% of range)
#define CATCH_MOVE_THRESHOLD  80   // ADC counts (~8% of range)

// ================================================================
// Detune zero band — not used on Commons PCB.
// Defined to satisfy shared CAN_Comm.cpp compilation.
// ================================================================
#define DETUNE_ZERO_BAND_LOW   480
#define DETUNE_ZERO_BAND_HIGH  545
#define DETUNE_FLASH_SLOW_MS   800
#define DETUNE_FLASH_FAST_MS    60

// ================================================================
// A single CAN parameter target + its encoding
// ================================================================
typedef struct
{
    uint8_t      canId;
    uint8_t      target;
    uint8_t      param;
    EncodeType_t encoding;
    uint8_t      steps;
} ParamDef_t;

// ================================================================
// ADC channel entry
// shiftBtn: -1 = single function, Button_t = shift button index
// No EXTRA_BTN_A6/A7 on Commons (those pins are LED outputs)
// ================================================================
typedef struct
{
    ParamDef_t primary;
    ParamDef_t secondary;
    int8_t     shiftBtn;
} ADC_Map_t;

// ================================================================
// Button modes
// ================================================================
typedef enum
{
    // Latching 2-state. Sends valueOn/valueOff. Owns local LED(s).
    BTN_MODE_TOGGLE,

    // Latching 2-state. No CAN message. Shifts pot mapping and
    // updates local indicator LEDs only. Used for Env/Track —
    // identical behaviour to A6/A7 Detune/Phase on the Oscillator board.
    BTN_MODE_LOCAL_TOGGLE,

    // Non-latching. Sends valueOn on press. No local LED.
    BTN_MODE_MOMENTARY,

    // 3-state modifier (not used on Commons — kept for shared code compat)
    BTN_MODE_MODIFIER,

    // N-state cycler: each press advances state 0→1→2→...→0.
    // Sends current state index as canValue.
    // No local LEDs — ESP32 confirms state via CAN ACK.
    BTN_MODE_CYCLER,

} BtnMode_t;

// ================================================================
// Button map entry
//
// CYCLER: cyclerStates = number of states (2 or 3).
//         Sends canValue = current state index (0, 1, 2...).
//
// TOGGLE with paired LEDs: ledBit = LED when ON,
//                          ledBitSecondary = LED when OFF.
//                          (-1 = no LED for that state)
// ================================================================
typedef struct
{
    BtnMode_t mode;
    uint8_t   canId;
    uint8_t   target;
    uint8_t   param;
    uint16_t  valueOn;          // sent on enable / momentary press
    uint16_t  valueOff;         // sent on disable (TOGGLE/MODIFIER only)
    int8_t    ledBit;           // local LED for TOGGLE ON / MODIFIER state 1
    int8_t    ledBitSecondary;  // local LED for TOGGLE OFF / MODIFIER state 2
    uint8_t   cyclerStates;     // CYCLER only: number of states
} BTN_Map_t;

// ================================================================
// IC3 LED word layout (16-bit → IO_SetLEDs)
// bits  7..0 = IC3 GPA7..GPA0
// bits 15..8 = IC3 GPB7..GPB0
//
// GPA0     : unused
// GPA1     : Led Unison       — REMOTE (ESP32 confirms Poly Mode=2)
// GPA2     : Led Duo          — REMOTE (ESP32 confirms Poly Mode=1)
// GPA3     : Led Poly         — REMOTE (ESP32 confirms Poly Mode=0)
// GPA4     : Led LFO1 Retrig  — REMOTE (ESP32 via ACK, driven by IO_SetRetrigLEDs)
// GPA5     : Led LFO2 Retrig  — REMOTE (ESP32 via ACK, driven by IO_SetRetrigLEDs)
// GPA6     : unused
// GPA7     : unused
//
// GPB0     : unused
// GPB1     : Led Lowpass      — REMOTE (ESP32 confirms Filter Mode=0)
// GPB2     : Led Bandpass     — REMOTE (ESP32 confirms Filter Mode=1)
// GPB3     : Led Highpass     — REMOTE (ESP32 confirms Filter Mode=2)
// GPB4     : Led Mod Mute     — REMOTE (ESP32 confirms mute state)
// GPB5     : Led Track Select — LOCAL  (ATmega owns — Env/Track OFF)
// GPB6     : Led Env Select   — LOCAL  (ATmega owns — Env/Track ON)
// GPB7     : Led Ring Enabled — REMOTE (ESP32 confirms ring mod enabled)
//
// Note: GPA4/GPA5 (LFO retrigger LEDs) are written by IO_SetRetrigLEDs()
// and merged in IO_SetLEDs() — they are excluded from LED_IC3_LOCAL_MASK
// and LED_IC3_REMOTE_MASK to avoid double-writing.
// ================================================================

// Local bits — ATmega controlled
#define LED_BIT_TRACK_SELECT   13   // IC3 GPB5 — lit when Env/Track button OFF
#define LED_BIT_ENV_SELECT     14   // IC3 GPB6 — lit when Env/Track button ON

// Remote bits — set by ESP32 CAN ACK only
#define LED_BIT_UNISON          1   // IC3 GPA1
#define LED_BIT_DUO             2   // IC3 GPA2
#define LED_BIT_POLY            3   // IC3 GPA3
#define LED_BIT_LOWPASS         9   // IC3 GPB1
#define LED_BIT_BANDPASS       10   // IC3 GPB2
#define LED_BIT_HIGHPASS       11   // IC3 GPB3
#define LED_BIT_MOD_MUTE       12   // IC3 GPB4
#define LED_BIT_RING_ENABLED   15   // IC3 GPB7

// LFO retrigger LEDs managed separately via IO_SetRetrigLEDs()
// GPA4 = LFO1, GPA5 = LFO2 — not in the masks below
#define LED_BIT_LFO1_RETRIG     4   // IC3 GPA4 (info only — not in mask)
#define LED_BIT_LFO2_RETRIG     5   // IC3 GPA5 (info only — not in mask)

#define LED_IC3_LOCAL_MASK   0x6000  // bits 13, 14 (Track, Env)
#define LED_IC3_REMOTE_MASK  0x9E0E  // bits 1,2,3,9,10,11,12,15

// ================================================================
// ADC channel mapping — CH0 to CH15
//
// CALIBRATION REQUIRED for ENCODE_RANGED channels (CH5, CH10, CH15).
// Run DEBUG_ADC_CAL mode, step through each switch position, note
// the stable raw values, then update the StepRange_t tables in
// CAN_Comm.cpp (lfoWaveformRanges[] and ringModPairsRanges[]).
// ================================================================
static const ADC_Map_t adcMap[16] =
{
    // CH0  Unison Detune  (0–65535 → 0–50 cents)
    {
        { CAN_ID_REALTIME, TARGET_GLOBAL, PARAM_GLOBAL_UNI_DETUNE, ENCODE_LINEAR, 0 },
        { 0 }, -1
    },
    // CH1  ADSR Release
    {
        { CAN_ID_REALTIME, TARGET_ENVELOPE, PARAM_ENV_RELEASE,     ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH2  ADSR Sustain
    {
        { CAN_ID_REALTIME, TARGET_ENVELOPE, PARAM_ENV_SUSTAIN,     ENCODE_LINEAR, 0 },
        { 0 }, -1
    },
    // CH3  ADSR Decay
    {
        { CAN_ID_REALTIME, TARGET_ENVELOPE, PARAM_ENV_DECAY,       ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH4  ADSR Attack
    {
        { CAN_ID_REALTIME, TARGET_ENVELOPE, PARAM_ENV_ATTACK,      ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH5  LFO1 Waveform (4-position switch)
    //      Positions: Sine=0 / Triangle=1 / Chaos=2 / S&H=3
    //      *** CALIBRATE: run DEBUG_ADC_CAL and note raw values ***
    {
        { CAN_ID_CONTROL,  TARGET_LFO1,    PARAM_LFO_WAVEFORM,    ENCODE_RANGED,  4 },
        { 0 }, -1
    },
    // CH6  LFO1 Delay
    {
        { CAN_ID_REALTIME, TARGET_LFO1,    PARAM_LFO_DELAY,       ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH7  LFO1 Rate
    {
        { CAN_ID_REALTIME, TARGET_LFO1,    PARAM_LFO_RATE,        ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH8  LFO2 Rate
    {
        { CAN_ID_REALTIME, TARGET_LFO2,    PARAM_LFO_RATE,        ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH9  LFO2 Delay
    {
        { CAN_ID_REALTIME, TARGET_LFO2,    PARAM_LFO_DELAY,       ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH10 LFO2 Waveform (4-position switch, same ladder as CH5)
    //      *** CALIBRATE: same values as CH5 expected ***
    {
        { CAN_ID_CONTROL,  TARGET_LFO2,    PARAM_LFO_WAVEFORM,    ENCODE_RANGED,  4 },
        { 0 }, -1
    },
    // CH11 Key Track Amount      (primary,   BTN_ENV_TRACK OFF = Track mode)
    //       Filter Envelope Amt  (secondary, BTN_ENV_TRACK ON  = Env mode)
    //
    // When the Track LED is lit (button OFF) the pot controls key tracking.
    // When the Env LED is lit (button ON) the pot controls envelope amount.
    {
        { CAN_ID_REALTIME, TARGET_FILTER,  PARAM_FILTER_KEY_TRACK, ENCODE_LINEAR, 0 },
        { CAN_ID_REALTIME, TARGET_FILTER,  PARAM_FILTER_ENV_AMT,   ENCODE_LINEAR, 0 },
        (int8_t)BTN_ENV_TRACK
    },
    // CH12 Filter Resonance
    {
        { CAN_ID_REALTIME, TARGET_FILTER,  PARAM_FILTER_RESONANCE,ENCODE_LINEAR, 0 },
        { 0 }, -1
    },
    // CH13 Filter Frequency (cutoff)
    {
        { CAN_ID_REALTIME, TARGET_FILTER,  PARAM_FILTER_CUTOFF,   ENCODE_EXP,    0 },
        { 0 }, -1
    },
    // CH14 Ring Mod Level
    {
        { CAN_ID_REALTIME, TARGET_RINGMOD, PARAM_RINGMOD_LEVEL,   ENCODE_LINEAR, 0 },
        { 0 }, -1
    },
    // CH15 Ring Mod Pairs (4-position switch)
    //      Positions: OSC0&1=0 / OSC2&3=1 / OSC0&2=2 / OSC1&3=3
    //      *** CALIBRATE: run DEBUG_ADC_CAL and note raw values ***
    {
        { CAN_ID_CONTROL,  TARGET_RINGMOD, PARAM_RINGMOD_PAIRS,   ENCODE_RANGED,  4 },
        { 0 }, -1
    },
};

// ================================================================
// Button mapping — indexed by Button_t (IO.h order)
//
// BTN_ENV_TRACK: TOGGLE with paired LEDs.
//   OFF → Track mode → ledBit=LED_BIT_TRACK_SELECT lit (default)
//   ON  → Env mode   → ledBitSecondary=LED_BIT_ENV_SELECT lit
//   Note: ledBit is the OFF LED, ledBitSecondary is the ON LED.
//         rebuildLocalLEDs() in CAN_Comm.cpp handles both states.
//
// BTN_FILTER_MODE / BTN_POLY_MODE: CYCLER (3 states, all LEDs remote).
//
// BTN_LFO1_TRIG / BTN_LFO2_TRIG: MOMENTARY — ESP32 owns LEDs via CAN ACK.
// BTN_MOD_MUTE / BTN_RING_ENABLE: TOGGLE (no local LED) — sends 1/0 on
//   alternate presses. ESP32 owns the LED and confirms state via ACK.
// ================================================================
static const BTN_Map_t btnMap[BTN_COUNT] =
{
    // BTN_FILTER_MODE (0) — CYCLER: LP(0)→BP(1)→HP(2)→LP
    // All 3 filter LEDs are remote (ESP32 confirms via CAN ACK).
    {
        BTN_MODE_CYCLER,
        CAN_ID_CONTROL, TARGET_FILTER, PARAM_FILTER_MODE,
        0, 0,   // valueOn/Off unused for CYCLER (sends state index)
        -1, -1, // no local LEDs
        3       // 3 states
    },

    // BTN_MOD_MUTE (1) — TOGGLE (remote LED)
    // Sends value 1 (mute) on first press, 0 (unmute) on second.
    // LED_BIT_MOD_MUTE is remote — ESP32 confirms via CAN ACK.
    {
        BTN_MODE_TOGGLE,
        CAN_ID_CONTROL, TARGET_RINGMOD, PARAM_RINGMOD_MUTE,
        1, 0, -1, -1, 0
    },

    // BTN_RING_ENABLE (2) — TOGGLE (remote LED)
    // Sends value 1 (enable) on first press, 0 (disable) on second.
    // LED_BIT_RING_ENABLED is remote — ESP32 confirms via CAN ACK.
    {
        BTN_MODE_TOGGLE,
        CAN_ID_CONTROL, TARGET_RINGMOD, PARAM_RINGMOD_ENABLE,
        1, 0, -1, -1, 0
    },

    // BTN_LFO2_TRIG (3) — MOMENTARY
    // LFO2 retrigger LED (IC3 GPA5) is remote via IO_SetRetrigLEDs().
    {
        BTN_MODE_MOMENTARY,
        CAN_ID_CONTROL, TARGET_LFO2, PARAM_LFO_RETRIGGER,
        1, 0, -1, -1, 0
    },

    // BTN_ENV_TRACK (4) — LOCAL_TOGGLE with paired local LEDs
    // No CAN message sent — ATmega owns this entirely, identical to
    // Detune/Phase switching on the Oscillator board.
    // OFF (Track mode): LED_BIT_TRACK_SELECT lit (GPB5) — default
    // ON  (Env mode):   LED_BIT_ENV_SELECT lit  (GPB6)
    {
        BTN_MODE_LOCAL_TOGGLE,
        0, 0, 0,               // canId/target/param unused
        0, 0,                  // valueOn/Off unused
        LED_BIT_ENV_SELECT,    // lit when ON  (Env mode)
        LED_BIT_TRACK_SELECT,  // lit when OFF (Track mode, default)
        0
    },

    // BTN_POLY_MODE (5) — CYCLER: Poly(0)→Duo(1)→Unison(2)→Poly
    // All 3 mode LEDs (Poly/Duo/Unison) are remote (ESP32 confirms).
    {
        BTN_MODE_CYCLER,
        CAN_ID_CONTROL, TARGET_GLOBAL, PARAM_GLOBAL_PLAY_MODE,
        0, 0, -1, -1, 3
    },

    // BTN_LFO1_TRIG (6) — MOMENTARY
    // LFO1 retrigger LED (IC3 GPA4) is remote via IO_SetRetrigLEDs().
    {
        BTN_MODE_MOMENTARY,
        CAN_ID_CONTROL, TARGET_LFO1, PARAM_LFO_RETRIGGER,
        1, 0, -1, -1, 0
    },
};
