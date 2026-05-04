#pragma once

// ================================================================
// param_map.h
// The single "truth table" for all hardware → CAN mappings.
// Edit this file to change what any knob, button, or LED does.
// ================================================================

#include "synth_params.h"
#include "IO.h"
#include <stdint.h>

// ================================================================
// Encoding types
// ================================================================
typedef enum
{
    // 10-bit → 0-65535. ESP32 decodes linearly.
    ENCODE_LINEAR,

    // 10-bit → 0-65535. ESP32 decodes exponentially (freq/time).
    ENCODE_EXP,

    // Resistor-ladder rotary switch. Raw ADC value is matched
    // against a StepRange_t table (defined in CAN_Comm.cpp).
    // Sends the canValue of the matched window after a settle period.
    // Readings that fall between windows are discarded (transition).
    ENCODE_RANGED,

    // Log-pot Detune with zero-band and piecewise linear compensation.
    // Hardware centre = DETUNE_RAW_CENTRE (≈300 for A-taper pot).
    // Raw inside zero band → sends exact 32767 (= 0 cents on ESP32).
    // Raw outside zero band → piecewise linear both sides of centre.
    // LED behaviour: solid in band, flashing outside (rate ∝ distance).
    ENCODE_DETUNE,

} EncodeType_t;

// ================================================================
// Resistor ladder step range
// One entry per switch position.
// adcMin/adcMax: ADC window boundaries (derived from measured values
// by taking midpoints between adjacent positions).
// canValue: the integer value sent in the CAN frame for this position.
// ================================================================
typedef struct
{
    uint16_t adcMin;
    uint16_t adcMax;
    uint16_t canValue;
} StepRange_t;

// ================================================================
// Detune zero band config
//
// Pot is LINEAR. With ADC_SETTLE_US=20 the centre reads ~505-515.
// Zero band brackets this so the user has a comfortable dead zone
// at 12 o'clock that sends exactly 32767 (= 0 cents on ESP32).
// Adjust LOW/HIGH if your hardware centre sits outside this range.
//
// ENCODE_DETUNE uses simple linear encoding (0→0, 1023→65535)
// outside the band — no log compensation needed.
// ================================================================
#define DETUNE_ZERO_BAND_LOW   480   // lower edge of zero band
#define DETUNE_ZERO_BAND_HIGH  545   // upper edge of zero band

// Flash rate at band edge and at full deflection (milliseconds).
#define DETUNE_FLASH_SLOW_MS   800
#define DETUNE_FLASH_FAST_MS    60

// Hold duration (ms) to disable an active modifier.
// Pressing and holding a modifier button for this long sends the disable
// CAN message and returns the modifier to the off state.
#define MODIFIER_HOLD_MS          2000

// Pot catch tuning
// CATCH_WINDOW: if the pot is already within this many ADC counts of the
// catch target when catch is activated, skip catch entirely.
#define CATCH_WINDOW              50   // ~5% of range
// CATCH_MOVE_THRESHOLD: release catch once the pot has moved this far from
// the position it was at when catch was activated, bounding any discontinuity
// to this distance rather than the full gap between the two pot positions.
#define CATCH_MOVE_THRESHOLD      80   // ~8% of range

// ================================================================
// Switch settle time (ENCODE_RANGED channels)
// Reading must remain within the same window for this many ms
// before a CAN message is sent. Filters zero-drop transients
// between switch contacts and debounces resistor-ladder glitches.
// ================================================================
#define SWITCH_SETTLE_MS          40

// ================================================================
// A single CAN parameter target + its encoding
// ================================================================
typedef struct
{
    uint8_t      canId;
    uint8_t      target;
    uint8_t      param;
    EncodeType_t encoding;
    uint8_t      steps;     // unused for ENCODE_LINEAR/EXP/DETUNE
} ParamDef_t;

// ================================================================
// ADC channel entry
// shiftBtn: -1 = single function
//           Button_t = MODIFIER/TOGGLE button index
//           EXTRA_BTN_A6/A7 = local extra button
// ================================================================
typedef struct
{
    ParamDef_t primary;
    ParamDef_t secondary;
    int8_t     shiftBtn;
} ADC_Map_t;

// Extra local-only shift button sentinels (> BTN_COUNT, fits int8_t)
#define EXTRA_BTN_A6   100
#define EXTRA_BTN_A7   101

// ================================================================
// Button modes
// ================================================================
typedef enum
{
    // Latching 2-state. Sends valueOn/valueOff. Owns a local LED.
    BTN_MODE_TOGGLE,

    // Non-latching. Sends valueOn on press. No local LED.
    // LED owned by ESP32 via CAN ACK.
    BTN_MODE_MOMENTARY,

    // 3-state modifier cycle:
    //   0 → 1: sends valueOn  (enable)   lights ledBit
    //   1 → 2: no CAN msg    (shift knob) lights ledBitSecondary
    //   2 → 0: sends valueOff (disable)  both LEDs off
    // Green "enabled" confirmation LED is remote (ESP32 ACK).
    BTN_MODE_MODIFIER,

} BtnMode_t;

// ================================================================
// Button map entry
// ================================================================
typedef struct
{
    BtnMode_t mode;
    uint8_t   canId;
    uint8_t   target;
    uint8_t   param;
    uint16_t  valueOn;
    uint16_t  valueOff;
    int8_t    ledBit;           // local LED for TOGGLE ON / MODIFIER state 1
    int8_t    ledBitSecondary;  // local LED for MODIFIER state 2
} BTN_Map_t;

// ================================================================
// IC3 LED word layout (16-bit → IO_SetLEDs)
// bits 7..0  = IC3 GPA7..GPA0
// bits 15..8 = IC3 GPB7..GPB0
// ================================================================

// Local bits — ATmega-controlled
#define LED_BIT_OSC1_DETUNE        0   // IC3 GPA0 — solid/flash: detune mode
#define LED_BIT_OSC1_PHASE         1   // IC3 GPA1 — solid: phase mode
#define LED_BIT_OSC1_FOLD_AMOUNT   4   // IC3 GPA4 — red, modifier state 1
#define LED_BIT_OSC1_FOLD_BIAS     5   // IC3 GPA5 — red, modifier state 2
#define LED_BIT_OSC1_SH_RANGE      6   // IC3 GPA6 — red, modifier state 2
#define LED_BIT_OSC1_SH_RATE       7   // IC3 GPA7 — red, modifier state 1
#define LED_BIT_OSC2_SH_RATE      10   // IC3 GPB2 — red, modifier state 1
#define LED_BIT_OSC2_SH_RANGE     11   // IC3 GPB3 — red, modifier state 2
#define LED_BIT_OSC2_FOLD_AMOUNT  12   // IC3 GPB4 — red, modifier state 1
#define LED_BIT_OSC2_FOLD_BIAS    13   // IC3 GPB5 — red, modifier state 2
#define LED_BIT_OSC2_PHASE        14   // IC3 GPB6 — solid: phase mode
#define LED_BIT_OSC2_DETUNE       15   // IC3 GPB7 — solid/flash: detune mode

// Remote bits — set by ESP32 CAN ACK only
#define LED_BIT_OSC1_FOLD_ENABLE   2   // IC3 GPA2 — green, fold active
#define LED_BIT_OSC1_SH_ENABLE     3   // IC3 GPA3 — green, S&H active
#define LED_BIT_OSC2_SH_ENABLE     8   // IC3 GPB0 — green, S&H active
#define LED_BIT_OSC2_FOLD_ENABLE   9   // IC3 GPB1 — green, fold active

#define LED_IC3_LOCAL_MASK   0xFCF3   // bits 0,1,4,5,6,7,10,11,12,13,14,15
#define LED_IC3_REMOTE_MASK  0x030C   // bits 2,3,8,9

// IC4 LED byte (IO_SetIC4LEDs) — all remote, set by ESP32 ACK
#define IC4_LED_BIT_CLONE1  0   // IC4 GPA2
#define IC4_LED_BIT_SOLO1   1   // IC4 GPA4
#define IC4_LED_BIT_SYNC1   2   // IC4 GPA7
#define IC4_LED_BIT_SOLO2   3   // IC4 GPB2
#define IC4_LED_BIT_CLONE2  4   // IC4 GPB4
#define IC4_LED_BIT_SYNC2   5   // IC4 GPB7

// ================================================================
// ADC channel mapping — CH0 to CH15
// Confirmed by hardware calibration on this board.
// ================================================================
static const ADC_Map_t adcMap[16] =
{
    // CH0  OSC1 ChaosR  (0–65535 → 2.5–4.0, ESP32 linear decode)
    {
        { CAN_ID_CONTROL,  TARGET_OSC1, PARAM_OSC_CHAOS_R,      ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
    // CH1  OSC1 Pulse Width
    {
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_PULSE_WIDTH,  ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
    // CH2  OSC1 Fold Amount (modifier states 0/1) / Fold Bias (state 2)
    {
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_FOLD_AMOUNT,  ENCODE_LINEAR,  0 },
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_FOLD_BIAS,    ENCODE_LINEAR,  0 },
        (int8_t)BTN_OSC1_FOLD
    },
    // CH3  OSC1 S&H Rate (modifier states 0/1) / S&H Range (state 2)
    {
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_SH_RATE,      ENCODE_EXP,     0 },
        { CAN_ID_CONTROL,  TARGET_OSC1, PARAM_OSC_SH_RANGE,     ENCODE_LINEAR,  0 },
        (int8_t)BTN_OSC1_SH
    },
    // CH4  OSC1 Level
    {
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_LEVEL,        ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
    // CH5  OSC1 Waveform (6-position resistor ladder switch)
    //      Measured positions: 0 / 79 / 116 / 160 / 231 / 398
    //      Windows use midpoints. Settle 40ms. See waveformRanges[] in CAN_Comm.cpp.
    {
        { CAN_ID_CONTROL,  TARGET_OSC1, PARAM_OSC_WAVEFORM,     ENCODE_RANGED,  6 },
        { 0 }, -1
    },
    // CH6  OSC1 Detune (A6 OFF, default) / FM Depth (A6 ON)
    //      Log pot: centre = 300. Zero band 275-325.
    //      FM Depth only sent when FM is enabled on this board pair — ignored by ESP32 otherwise.
    {
        { CAN_ID_REALTIME, TARGET_OSC1, PARAM_OSC_DETUNE,       ENCODE_DETUNE,  0 },
        { CAN_ID_CONTROL,  TARGET_OSC1, PARAM_OSC_FM_DEPTH,     ENCODE_LINEAR,  0 },
        (int8_t)EXTRA_BTN_A6
    },
    // CH7  OSC1 Octave (3-position resistor ladder switch)
    //      Measured positions: 0 / 188 / 366
    //      See octaveRanges[] in CAN_Comm.cpp.
    {
        { CAN_ID_CONTROL,  TARGET_OSC1, PARAM_OSC_OCTAVE,       ENCODE_RANGED,  3 },
        { 0 }, -1
    },
    // CH8  OSC2 Waveform (identical ladder to CH5)
    //      Measured positions: 0 / 76 / 110 / 151 / 219 / 378
    {
        { CAN_ID_CONTROL,  TARGET_OSC2, PARAM_OSC_WAVEFORM,     ENCODE_RANGED,  6 },
        { 0 }, -1
    },
    // CH9  OSC2 Octave (identical ladder to CH7)
    //      Measured positions: 0 / 184 / 366
    {
        { CAN_ID_CONTROL,  TARGET_OSC2, PARAM_OSC_OCTAVE,       ENCODE_RANGED,  3 },
        { 0 }, -1
    },
    // CH10 OSC2 Detune (A7 OFF, default) / FM Ratio (A7 ON)
    //      Same log pot. Centre = 300.
    //      FM Ratio only sent when FM is enabled on this board pair — ignored by ESP32 otherwise.
    {
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_DETUNE,       ENCODE_DETUNE,  0 },
        { CAN_ID_CONTROL,  TARGET_OSC2, PARAM_OSC_FM_RATIO,     ENCODE_EXP,     0 },
        (int8_t)EXTRA_BTN_A7
    },
    // CH11 OSC2 Pulse Width
    {
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_PULSE_WIDTH,  ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
    // CH12 OSC2 ChaosR
    {
        { CAN_ID_CONTROL,  TARGET_OSC2, PARAM_OSC_CHAOS_R,      ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
    // CH13 OSC2 Fold Amount (modifier states 0/1) / Fold Bias (state 2)
    {
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_FOLD_AMOUNT,  ENCODE_LINEAR,  0 },
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_FOLD_BIAS,    ENCODE_LINEAR,  0 },
        (int8_t)BTN_OSC2_FOLD
    },
    // CH14 OSC2 S&H Rate (modifier states 0/1) / S&H Range (state 2)
    {
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_SH_RATE,      ENCODE_EXP,     0 },
        { CAN_ID_CONTROL,  TARGET_OSC2, PARAM_OSC_SH_RANGE,     ENCODE_LINEAR,  0 },
        (int8_t)BTN_OSC2_SH
    },
    // CH15 OSC2 Level
    {
        { CAN_ID_REALTIME, TARGET_OSC2, PARAM_OSC_LEVEL,        ENCODE_LINEAR,  0 },
        { 0 }, -1
    },
};

// ================================================================
// Button mapping — indexed by Button_t (IO.h order)
// ================================================================
static const BTN_Map_t btnMap[BTN_COUNT] =
{
    // BTN_OSC1_SH (0) — MODIFIER
    { BTN_MODE_MODIFIER, CAN_ID_CONTROL, TARGET_OSC1, PARAM_OSC_SH_ENABLE,
      1, 0, LED_BIT_OSC1_SH_RATE, LED_BIT_OSC1_SH_RANGE },

    // BTN_OSC1_FOLD (1) — MODIFIER
    { BTN_MODE_MODIFIER, CAN_ID_CONTROL, TARGET_OSC1, PARAM_OSC_FOLD_ENABLE,
      1, 0, LED_BIT_OSC1_FOLD_AMOUNT, LED_BIT_OSC1_FOLD_BIAS },

    // BTN_OSC1_CLONE (2) — MOMENTARY, LED remote (IC4 GPA2)
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC1, PARAM_OSC_CLONE,
      1, 0, -1, -1 },

    // BTN_OSC1_SOLO (3) — MOMENTARY, LED remote (IC4 GPA4)
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC1, PARAM_OSC_SOLO,
      1, 0, -1, -1 },

    // BTN_OSC1_SYNC (4) — MOMENTARY, repurposed as FM ENABLE
    // Toggles FM modulation: OSC1 (modulator) → OSC2 (carrier).
    // ESP32 handler cancels sync on OSC2 if active when FM is enabled.
    // LED remote (IC4 GPA7) now reflects FM active state.
    // NOTE: disabled on board ID 0 logic removed — FM enable valid on all boards.
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC1, PARAM_OSC_FM_ENABLE,
      1, 0, -1, -1 },

    // BTN_OSC2_SYNC (5) — MOMENTARY, LED remote (IC4 GPB7)
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC2, PARAM_OSC_SYNC_ENABLE,
      1, 0, -1, -1 },

    // BTN_OSC2_SOLO (6) — MOMENTARY, LED remote (IC4 GPB2)
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC2, PARAM_OSC_SOLO,
      1, 0, -1, -1 },

    // BTN_OSC2_CLONE (7) — MOMENTARY, LED remote (IC4 GPB4)
    { BTN_MODE_MOMENTARY, CAN_ID_CONTROL, TARGET_OSC2, PARAM_OSC_CLONE,
      1, 0, -1, -1 },

    // BTN_OSC2_FOLD (8) — MODIFIER
    { BTN_MODE_MODIFIER, CAN_ID_CONTROL, TARGET_OSC2, PARAM_OSC_FOLD_ENABLE,
      1, 0, LED_BIT_OSC2_FOLD_AMOUNT, LED_BIT_OSC2_FOLD_BIAS },

    // BTN_OSC2_SH (9) — MODIFIER
    { BTN_MODE_MODIFIER, CAN_ID_CONTROL, TARGET_OSC2, PARAM_OSC_SH_ENABLE,
      1, 0, LED_BIT_OSC2_SH_RATE, LED_BIT_OSC2_SH_RANGE },
};
