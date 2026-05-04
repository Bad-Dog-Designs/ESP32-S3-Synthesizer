#pragma once
#include <math.h>   // powf, logf

// ==============================================================
// synth_params.h  —  Version 10.1
// Shared CAN parameter definitions
// Include in: ESP32-S3 engine, ATmega oscillator boards, ATmega commons board
// CAN bus speed: 500kbps
// ==============================================================

// ==========================
// CAN Message IDs (11-bit)
// Lower ID = higher priority on bus arbitration
// ==========================
#define CAN_ID_SYSTEM       0x000  // System startup — highest priority
#define CAN_ID_PANIC        0x001  // All notes off
#define CAN_ID_ANNOUNCE     0x002  // Board presence announcement
#define CAN_ID_REQUEST      0x003  // ESP32 → ATmega: request full parameter dump
#define CAN_ID_PERFORMANCE  0x010  // Solo, clone, play mode
#define CAN_ID_REALTIME     0x020  // Level, detune, cutoff — time sensitive
#define CAN_ID_CONTROL      0x030  // Waveform, mode, switch changes
#define CAN_ID_ACK          0x040  // ESP32-S3 → ATmega acknowledgement
                                   //
                                   // ACK frame layout (5 bytes):
                                   //   Byte 0: Target echo
                                   //   Byte 1: Param echo
                                   //   Byte 2: IC3 remote LED high byte (bits 15-8)
                                   //   Byte 3: IC3 remote LED low byte  (bits 7-0)
                                   //   Byte 4: IC4 LED byte
                                   //
                                   // IC3 remote bits set by ESP32 (oscillator boards):
                                   //   bit 2 = OSC1 fold enabled
                                   //   bit 3 = OSC1 S&H enabled
                                   //   bit 8 = OSC2 S&H enabled
                                   //   bit 9 = OSC2 fold enabled
                                   //
                                   // IC4 remote bits set by ESP32 (oscillator boards):
                                   //   bit 0 = CLONE1  bit 1 = SOLO1  bit 2 = SYNC1
                                   //   bit 3 = SOLO2   bit 4 = CLONE2 bit 5 = SYNC2
                                   //
                                   // IC3 remote bits set by ESP32 (commons board):
                                   //   bit 1 = UNISON  bit 2 = DUO    bit 3 = POLY
                                   //   bit 9 = LOWPASS bit 10= BANDPASS bit 11= HIGHPASS
                                   //   bit 12= MOD_MUTE bit 15= RING_ENABLED

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
#define PARAM_SYSTEM_READY          0x01  // ESP32-S3 → all: ready to receive
#define PARAM_SYSTEM_BOARD_COMPLETE 0x02  // Board → ESP32-S3: finished sending params
                                          // data[2] = board ID
                                          // data[3] = board type (BOARD_TYPE_xxx)
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
                                      //         32767 = exact centre = 0 cents
#define PARAM_OSC_FM_DEPTH      0x04  // uint16: 0-65535 → 0.0 to 1.0
                                      //         OSC_A only: FM modulation depth.
                                      //         Ignored by ESP32 if FM not enabled for this board pair.
                                      //         Previously PARAM_OSC_PHASE — phase start not user-accessible.
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
#define PARAM_OSC_FM_RATIO      0x11  // uint16: 0-65535 → 0.5 to 8.0 (exponential)
                                      //         OSC_B only: FM carrier:modulator ratio.
                                      //         Ignored by ESP32 if FM not enabled for this board pair.
#define PARAM_OSC_FM_ENABLE     0x12  // uint16: 0=off 1=on (OSC_A only)
                                      //         Enables FM: OSC_A modulates OSC_B phase.
                                      //         Cancels sync on OSC_B if active.

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
#define PARAM_GLOBAL_SYNC_MASTER    0x04  // DEPRECATED — was OSC_A board sync button,
                                          // now repurposed as PARAM_OSC_FM_ENABLE.
                                          // ESP32 handler retained but no board sends this.
#define PARAM_GLOBAL_SYNC_ALL       0x05  // DEPRECATED — superseded by per-board-pair
                                          // sync via BTN_OSC2_SYNC on each board.
                                          // ESP32 handler retained but no board sends this.
#define PARAM_GLOBAL_MASTER_LEVEL   0x06  // uint16: 0-65535 → 0.0 to 1.0

// ==========================
// LFO Parameters (TARGET_LFO1, TARGET_LFO2)
// Both LFOs use identical parameter IDs, differentiated by target.
//
// LFO has 4 hardware waveforms matching the rotary switch:
//   0=SINE  1=TRI  2=CHAOS  3=S&H
//
// LFO depth is intentionally absent — modulation depth is set
// per-route via PARAM_ROUTE_AMOUNT in V11 routing.
//
// PARAM_LFO_RETRIGGER: sent as MOMENTARY (value=1 each press).
// ESP32 toggles retrigger state on each received message.
// ==========================
#define PARAM_LFO_RATE          0x01  // uint16: 0-65535 → 0.01 to 20.0 Hz (exponential)
#define PARAM_LFO_DELAY         0x02  // uint16: 0-65535 → 0.0 to 5.0 sec (exponential)
#define PARAM_LFO_WAVEFORM      0x03  // uint16: 0=SINE 1=TRI 2=CHAOS 3=S&H
#define PARAM_LFO_RETRIGGER     0x04  // uint16: 1 = toggle retrigger state (momentary)

// ==========================
// Ring Modulator Parameters (TARGET_RINGMOD)
// ==========================
#define PARAM_RINGMOD_ENABLE    0x01  // uint16: 0=off 1=on
#define PARAM_RINGMOD_PAIRS     0x02  // uint16: 0=V0&V1 1=V2&V3 2=V0&V2 3=V1&V3
#define PARAM_RINGMOD_LEVEL     0x03  // uint16: 0-65535 → 0.0 to 1.0
#define PARAM_RINGMOD_MUTE      0x04  // uint16: 0=modulator audible 1=muted

// ==========================
// Routing Parameters (TARGET_ROUTE) — V11
// ==========================
#define PARAM_ROUTE_SET         0x01
#define PARAM_ROUTE_CLEAR       0x02
#define PARAM_ROUTE_CLEAR_ALL   0x03
#define PARAM_ROUTE_AMOUNT      0x04  // Bipolar -1.0 to +1.0
#define PARAM_ROUTE_ENABLE      0x05

// Route source IDs
#define ROUTE_SRC_LFO1          0x00
#define ROUTE_SRC_LFO2          0x01
#define ROUTE_SRC_ENV           0x02
#define ROUTE_SRC_VELOCITY      0x03  // Static per note 0.0-1.0
#define ROUTE_SRC_NOTE          0x04  // MIDI note normalised 0.0-1.0
#define ROUTE_SRC_MODWHEEL      0x05  // MIDI CC1 mod wheel 0.0-1.0

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
