#pragma once
#include <math.h>   // powf, logf

// ==============================================================
// synth_params.h  —  Version 10.0
// Shared CAN parameter definitions
// Include in both ESP32-S3 (sound engine) and ATmega328p (control boards)
// CAN bus speed: 500kbps
// ==============================================================

// ==========================
// CAN Message IDs (11-bit)
// Lower ID = higher priority on bus arbitration
// ==========================
#define CAN_ID_SYSTEM       0x000  // System startup messages — highest priority
#define CAN_ID_PANIC        0x001  // All notes off
#define CAN_ID_ANNOUNCE     0x002  // Board presence announcement on power-up
#define CAN_ID_REQUEST      0x003  // ESP32 → ATmega: request full parameter dump
#define CAN_ID_PERFORMANCE  0x010  // Solo, clone, play mode
#define CAN_ID_REALTIME     0x020  // Level, detune, cutoff — time sensitive
#define CAN_ID_CONTROL      0x030  // Waveform, mode, switch changes
#define CAN_ID_ACK          0x040  // Acknowledgement ESP32-S3 → ATmega

// ==========================
// Data Byte 0: Target
// ==========================
#define TARGET_SYSTEM       0x00  // Startup handshake
#define TARGET_OSC1         0x01
#define TARGET_OSC2         0x02
#define TARGET_OSC3         0x03
#define TARGET_OSC4         0x04
#define TARGET_OSC5         0x05
#define TARGET_OSC6         0x06
#define TARGET_OSC7         0x07
#define TARGET_OSC8         0x08
#define TARGET_FILTER       0x10
#define TARGET_ENVELOPE     0x11
#define TARGET_GLOBAL       0x20
#define TARGET_LFO1         0x30
#define TARGET_LFO2         0x31
#define TARGET_RINGMOD      0x40
#define TARGET_ROUTE        0x50  // V11

// ==========================
// System Startup (TARGET_SYSTEM)
// ==========================
#define PARAM_SYSTEM_READY          0x01  // ESP32-S3 → all: ready to receive params
#define PARAM_SYSTEM_BOARD_COMPLETE 0x02  // Board → ESP32-S3: finished sending params
#define PARAM_SYSTEM_ALL_COMPLETE   0x03  // ESP32-S3 → all: fully initialised
#define PARAM_SYSTEM_REQUEST_DUMP   0x04  // ESP32-S3 → board: resend all params

#define BOARD_TYPE_OSC              0x01  // Dual oscillator board
#define BOARD_TYPE_COMMONS          0x02  // Commons board
#define BOARD_TYPE_ROUTING          0x03  // Routing board — V11

// ==========================
// Oscillator Parameters (TARGET_OSC1–OSC8)
// ==========================
#define PARAM_OSC_WAVEFORM      0x01  // uint16: 0=SAW 1=SINE 2=TRI 3=PULSE 4=CHAOS 5=NOISE
#define PARAM_OSC_OCTAVE        0x02  // uint16: 0=down(-1) 1=centre(0) 2=up(+1)
#define PARAM_OSC_DETUNE        0x03  // uint16: 0-65535 → -50.0 to +50.0 cents
#define PARAM_OSC_PHASE         0x04  // uint16: 0-65535 → 0.0 to 1.0
                                      // USER CONTROL: waveform start position at note-on
                                      // Distinct from internal phase accumulator
#define PARAM_OSC_LEVEL         0x05  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_OSC_PULSE_WIDTH   0x06  // uint16: 0-65535 → 0.01 to 0.99
#define PARAM_OSC_CHAOS_R       0x07  // uint16: 0-65535 → 2.5 to 4.0
#define PARAM_OSC_FOLD_ENABLE   0x08  // uint16: 0=off 1=on
#define PARAM_OSC_FOLD_AMOUNT   0x09  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_OSC_FOLD_BIAS     0x0A  // uint16: 0-65535 → -1.0 to +1.0
#define PARAM_OSC_SH_ENABLE     0x0B  // uint16: 0=off 1=on
#define PARAM_OSC_SH_RATE       0x0C  // uint16: 0-65535 → 0.5 to 40.0 Hz (exponential)
#define PARAM_OSC_SH_RANGE      0x0D  // uint16: 0-65535 → 1 to 48 semitones
#define PARAM_OSC_SYNC_ENABLE   0x0E  // uint16: 0=off 1=on (voices 2-6 only, syncs to voice 1)
#define PARAM_OSC_SOLO          0x0F  // uint16: 0=off 1=on
#define PARAM_OSC_CLONE         0x10  // uint16: source voice index (0-based)

// ==========================
// Filter Parameters (TARGET_FILTER)
// ==========================
#define PARAM_FILTER_CUTOFF     0x01  // uint16: 0-65535 → 20 to 18000 Hz (exponential)
#define PARAM_FILTER_RESONANCE  0x02  // uint16: 0-65535 → 0.0 to 0.97
#define PARAM_FILTER_ENV_AMT    0x03  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_FILTER_KEY_TRACK  0x04  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_FILTER_MODE       0x05  // uint16: 0=lowpass 1=bandpass 2=highpass

// ==========================
// Envelope Parameters (TARGET_ENVELOPE)
// ==========================
#define PARAM_ENV_ATTACK        0x01  // uint16: 0-65535 → 0.001 to 10.0 sec (exponential)
#define PARAM_ENV_DECAY         0x02  // uint16: 0-65535 → 0.001 to 10.0 sec (exponential)
#define PARAM_ENV_SUSTAIN       0x03  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_ENV_RELEASE       0x04  // uint16: 0-65535 → 0.001 to 10.0 sec (exponential)

// ==========================
// Global Parameters (TARGET_GLOBAL)
// ==========================
#define PARAM_GLOBAL_PLAY_MODE      0x01  // uint16: 0=poly 1=duo 2=unison
#define PARAM_GLOBAL_UNI_DETUNE     0x02  // uint16: 0-65535 → 0 to 50 cents
#define PARAM_GLOBAL_PANIC          0x03  // uint16: any value — all notes off
#define PARAM_GLOBAL_SYNC_MASTER    0x04  // uint16: OSC index (0-7) as sync master
#define PARAM_GLOBAL_SYNC_ALL       0x05  // uint16: 1=sync all OSC2-6 to OSC1, 0=unsync all
#define PARAM_GLOBAL_MASTER_LEVEL   0x06  // uint16: 0-65535 → 0.0 to 1.0

// ==========================
// LFO Parameters (TARGET_LFO1, TARGET_LFO2)
// Both LFOs share identical parameter IDs, differentiated by target.
//
// IMPORTANT: LFO depth is intentionally absent.
// Modulation depth is controlled per-route via PARAM_ROUTE_AMOUNT,
// giving independent depth per destination from the same LFO source.
// ==========================
#define PARAM_LFO_RATE          0x01  // uint16: 0-65535 → 0.01 to 20.0 Hz (exponential)
#define PARAM_LFO_DELAY         0x02  // uint16: 0-65535 → 0.0 to 5.0 sec (exponential)
#define PARAM_LFO_WAVEFORM      0x03  // uint16: 0=SINE 1=TRI 2=SAW 3=PULSE 4=CHAOS 5=S&H
#define PARAM_LFO_RETRIGGER     0x04  // uint16: 0=free running 1=retrigger on note-on

// ==========================
// Ring Modulator Parameters (TARGET_RINGMOD)
// ==========================
#define PARAM_RINGMOD_ENABLE    0x01  // uint16: 0=off 1=on
#define PARAM_RINGMOD_PAIRS     0x02  // uint16: 0=V0&V1 1=V2&V3 2=V0&V2 3=V1&V3
#define PARAM_RINGMOD_LEVEL     0x03  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_RINGMOD_MUTE      0x04  // uint16: 0=modulator audible 1=muted

// ==========================
// Routing Parameters (TARGET_ROUTE) — V11
// Defined here so CAN handlers can be stubbed in V10.
// ==========================
#define PARAM_ROUTE_SET         0x01
#define PARAM_ROUTE_CLEAR       0x02
#define PARAM_ROUTE_CLEAR_ALL   0x03
#define PARAM_ROUTE_AMOUNT      0x04  // Bipolar -1.0 to +1.0 per route
#define PARAM_ROUTE_ENABLE      0x05

// Route source IDs
#define ROUTE_SRC_LFO1          0x00
#define ROUTE_SRC_LFO2          0x01
#define ROUTE_SRC_ENV           0x02
#define ROUTE_SRC_VELOCITY      0x03  // Static per note 0.0-1.0
#define ROUTE_SRC_NOTE          0x04  // MIDI note normalised 0.0-1.0

// Route destination targets
#define ROUTE_DST_TARGET_OSC1   0x00
#define ROUTE_DST_TARGET_OSC2   0x01
#define ROUTE_DST_TARGET_OSC3   0x02
#define ROUTE_DST_TARGET_OSC4   0x03
#define ROUTE_DST_TARGET_OSC5   0x04
#define ROUTE_DST_TARGET_OSC6   0x05
#define ROUTE_DST_TARGET_LFO2   0x06  // LFO1 → LFO2 rate modulation
#define ROUTE_DST_TARGET_FILTER 0x07
#define ROUTE_DST_TARGET_GLOBAL 0x08

// Route destination parameters — OSC
#define ROUTE_DST_PITCH         0x00
#define ROUTE_DST_PULSE_WIDTH   0x01
#define ROUTE_DST_FOLD_AMOUNT   0x02
#define ROUTE_DST_FOLD_BIAS     0x03
#define ROUTE_DST_CHAOS_R       0x04
#define ROUTE_DST_SH_RATE       0x05
#define ROUTE_DST_LEVEL         0x06

// Route destination parameters — FILTER
#define ROUTE_DST_CUTOFF        0x00
#define ROUTE_DST_RESONANCE     0x01

// Route destination parameters — GLOBAL
#define ROUTE_DST_AMP           0x00
#define ROUTE_DST_UNI_DETUNE    0x01

// Route destination parameters — LFO2
#define ROUTE_DST_LFO2_RATE     0x00

// ==========================
// Frame Layout (5 bytes)
// ==========================
// Byte 0 : Target      (TARGET_xxx)
// Byte 1 : Parameter   (PARAM_xxx)
// Byte 2 : Value high  (MSB of uint16)
// Byte 3 : Value low   (LSB of uint16)
// Byte 4 : Sequence    (increments each message, for ACK matching)

// ==========================
// Value Encoding Helpers
// ==========================

inline float decodeLinear(uint16_t value, float minVal, float maxVal)
{
  return minVal + (value / 65535.0f) * (maxVal - minVal);
}

inline float decodeExp(uint16_t value, float minVal, float maxVal)
{
  return minVal * powf(maxVal / minVal, value / 65535.0f);
}

inline uint16_t encodeLinear(float value, float minVal, float maxVal)
{
  float norm = (value - minVal) / (maxVal - minVal);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return (uint16_t)(norm * 65535.0f);
}

inline uint16_t encodeExp(float value, float minVal, float maxVal)
{
  float norm = logf(value / minVal) / logf(maxVal / minVal);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return (uint16_t)(norm * 65535.0f);
}
