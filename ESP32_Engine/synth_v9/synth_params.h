#pragma once

// ==============================================================
// synth_params.h
// Shared CAN parameter definitions
// Include in both ESP32-S3 (sound engine) and ATmega328p (control boards)
// CAN bus speed: 500kbps
// ==============================================================

// ==========================
// CAN Message IDs (11-bit)
// Lower ID = higher priority on bus arbitration
// ==========================
#define CAN_ID_PANIC        0x001  // All notes off — highest priority
#define CAN_ID_PERFORMANCE  0x010  // Solo, clone, play mode
#define CAN_ID_REALTIME     0x020  // Level, detune, cutoff — time sensitive
#define CAN_ID_CONTROL      0x030  // Waveform, mode, switch changes
#define CAN_ID_ACK          0x040  // Acknowledgement ESP32-S3 → ATmega

// ==========================
// Data Byte 0: Target
// Identifies which section of the engine the message addresses
// ==========================
#define TARGET_OSC1         0x01
#define TARGET_OSC2         0x02
#define TARGET_OSC3         0x03
#define TARGET_OSC4         0x04
#define TARGET_FILTER       0x10
#define TARGET_ENVELOPE     0x11
#define TARGET_GLOBAL       0x20
#define TARGET_LFO          0x30   // Reserved for V10

// ==========================
// Data Byte 1: Parameter ID
// ==========================

// --- Oscillator Parameters (TARGET_OSC1 – TARGET_OSC4) ---
#define PARAM_OSC_WAVEFORM      0x01  // uint16: 0=SAW 1=SINE 2=TRI 3=PULSE 4=CHAOS 5=NOISE
#define PARAM_OSC_OCTAVE        0x02  // uint16: 0=down(-1) 1=centre(0) 2=up(+1)
#define PARAM_OSC_DETUNE        0x03  // uint16: 0–65535 → -50.0 to +50.0 cents
#define PARAM_OSC_PHASE         0x04  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_OSC_LEVEL         0x05  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_OSC_PULSE_WIDTH   0x06  // uint16: 0–65535 → 0.01 to 0.99
#define PARAM_OSC_CHAOS_R       0x07  // uint16: 0–65535 → 2.5 to 4.0
#define PARAM_OSC_FOLD_ENABLE   0x08  // uint16: 0=off 1=on
#define PARAM_OSC_FOLD_AMOUNT   0x09  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_OSC_FOLD_BIAS     0x0A  // uint16: 0–65535 → -1.0 to +1.0
#define PARAM_OSC_SH_ENABLE     0x0B  // uint16: 0=off 1=on
#define PARAM_OSC_SH_RATE       0x0C  // uint16: 0–65535 → 0.5 to 40.0 Hz (exponential)
#define PARAM_OSC_SH_RANGE      0x0D  // uint16: 0–65535 → 1 to 48 semitones
#define PARAM_OSC_SYNC_ENABLE   0x0E  // uint16: 0=off 1=on (voice 0 = sync master)
#define PARAM_OSC_SOLO          0x0F  // uint16: 0=off 1=on
#define PARAM_OSC_CLONE         0x10  // uint16: source voice index (0–3), clones to all others

// --- Filter Parameters (TARGET_FILTER) ---
#define PARAM_FILTER_CUTOFF     0x01  // uint16: 0–65535 → 20 to 18000 Hz (exponential)
#define PARAM_FILTER_RESONANCE  0x02  // uint16: 0–65535 → 0.0 to 0.97
#define PARAM_FILTER_ENV_AMT    0x03  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_FILTER_KEY_TRACK  0x04  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_FILTER_MODE       0x05  // uint16: 0=lowpass 1=bandpass 2=highpass

// --- Envelope Parameters (TARGET_ENVELOPE) ---
#define PARAM_ENV_ATTACK        0x01  // uint16: 0–65535 → 0.001 to 10.0 seconds (exponential)
#define PARAM_ENV_DECAY         0x02  // uint16: 0–65535 → 0.001 to 10.0 seconds (exponential)
#define PARAM_ENV_SUSTAIN       0x03  // uint16: 0–65535 → 0.0 to 1.0
#define PARAM_ENV_RELEASE       0x04  // uint16: 0–65535 → 0.001 to 10.0 seconds (exponential)

// --- Global Parameters (TARGET_GLOBAL) ---
#define PARAM_GLOBAL_PLAY_MODE  0x01  // uint16: 0=poly 1=unison
#define PARAM_GLOBAL_UNI_DETUNE 0x02  // uint16: 0–65535 → 0 to 50 cents
#define PARAM_GLOBAL_PANIC      0x03  // uint16: any value — all notes off immediately

// --- LFO Parameters (TARGET_LFO) — Reserved for V10 ---
#define PARAM_LFO_RATE          0x01
#define PARAM_LFO_DEPTH         0x02
#define PARAM_LFO_WAVEFORM      0x03
#define PARAM_LFO_DESTINATION   0x04

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
// Include <math.h> before using decodeExp / encodeExp
// ==========================

// Linear decode: uint16 → float range
inline float decodeLinear(uint16_t value, float minVal, float maxVal)
{
  return minVal + (value / 65535.0f) * (maxVal - minVal);
}

// Exponential decode: uint16 → float range (for frequency/time params)
inline float decodeExp(uint16_t value, float minVal, float maxVal)
{
  return minVal * powf(maxVal / minVal, value / 65535.0f);
}

// Linear encode: float → uint16
inline uint16_t encodeLinear(float value, float minVal, float maxVal)
{
  float norm = (value - minVal) / (maxVal - minVal);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return (uint16_t)(norm * 65535.0f);
}

// Exponential encode: float → uint16 (for frequency/time params)
inline uint16_t encodeExp(float value, float minVal, float maxVal)
{
  float norm = logf(value / minVal) / logf(maxVal / minVal);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return (uint16_t)(norm * 65535.0f);
}
