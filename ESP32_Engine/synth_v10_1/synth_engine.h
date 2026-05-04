#pragma once
// ==============================================================
// synth_engine.h  —  Version 10.1
// Voice, LFO, filter, oscillator and modulation routing types.
// Included by: synth_engine.cpp, synth_midi.cpp, synth_can.cpp, synth_v10_1.ino
// ==============================================================

#include <Arduino.h>
#include <math.h>
#include "synth_params.h"

// ==========================
// Configuration
// ==========================
#define SAMPLE_RATE  48000
#define TABLE_SIZE   1024
#define TABLE_BITS   10
#define MAX_VOICES   8    // Compile-time array size — actual active count is NUM_VOICES_ACTIVE

// ==========================
// Enumerations
// ==========================
enum OscType     { OSC_SAW, OSC_SINE, OSC_TRI, OSC_PULSE, OSC_CHAOS, OSC_NOISE };
enum FilterMode  { FILT_LOW, FILT_BAND, FILT_HIGH };
enum PlayMode    { MODE_POLY, MODE_DUO, MODE_UNISON };
enum EnvState    { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

// 4 waveforms matching the hardware rotary switch: SINE=0 TRI=1 CHAOS=2 S&H=3
enum LFOWaveform { LFO_SINE, LFO_TRI, LFO_CHAOS, LFO_SH };

// ==========================
// LFO
// Note: no 'depth' field — modulation depth is per-route via Route.amount.
// RETRIGGER is a toggle: each PARAM_LFO_RETRIGGER message flips the state.
// ==========================
struct LFO
{
  float        rate      = 1.0f;   // Hz
  float        delay     = 0.0f;   // seconds before LFO fades in after note-on
  LFOWaveform  waveform  = LFO_SINE;
  bool         retrigger = false;  // true = reset phase on every note-on

  // Internal state — not user accessible
  float        phase      = 0.0f;  // running phase accumulator 0.0-1.0
  float        shValue    = 0.0f;  // held value for S+H waveform
  float        chaosX     = 0.5f;  // logistic map state for CHAOS waveform
  float        chaosR     = 3.7f;  // fixed default — not a real-time control
  float        delayTimer = 0.0f;  // counts up from 0 at note-on
};

// ==========================
// Modulation Routing — V11 stubs
// Routing data received and stored via CAN. Modulation not applied until V11.
// ==========================
struct Route
{
  bool    enabled   = false;
  uint8_t source    = ROUTE_SRC_LFO1;
  uint8_t dstTarget = ROUTE_DST_TARGET_FILTER;
  uint8_t dstParam  = ROUTE_DST_CUTOFF;
  float   amount    = 0.0f;  // -1.0 to +1.0, negative = inverted modulation
};

// ==========================
// Ring Modulator
// ==========================
struct RingMod
{
  bool    enabled = false;
  uint8_t pair    = 0;      // 0=V0&V1  1=V2&V3  2=V0&V2  3=V1&V3
  float   level   = 1.0f;
  bool    modMute = false;  // true = modulator voice silent in final mix
};

// ==========================
// Voice
// ==========================
// PHASE NAMING:
//   phase       : uint32 running accumulator — INTERNAL, never user-accessible
//   phaseOffset : float 0.0-1.0, USER CONTROL — waveform start position at note-on
//
// CHAOS INTERNALS:
//   chaosR : user parameter (2.5-4.0)
//   chaosX : internal logistic map state, seeded from MIDI note — never user-accessible
// ==========================
struct Voice
{
  // --- Core state (volatile: read Core 1, written Core 0) ---
  volatile uint32_t phase         = 0;
  volatile uint32_t phaseInc      = 0;
  volatile bool     active        = false;
  volatile uint8_t  note          = 255;
  volatile EnvState envState      = ENV_IDLE;
  volatile float    envLevel      = 0.0f;

  // --- Waveform ---
  OscType           oscType       = OSC_SAW;
  float             pulseWidth    = 0.5f;

  // --- Pitch ---
  int8_t            octave        = 0;      // -1, 0, +1
  float             detuneCents   = 0.0f;   // -50 to +50 cents
  float             phaseOffset   = 0.0f;   // internal waveform start 0.0-1.0

  // --- FM ---
  // OSC_A (even voice) is the modulator, OSC_B (odd voice) is the carrier.
  // fmEnabled/fmDepth live on OSC_A. fmRatio lives on OSC_B.
  // FM modulates OSC_B's phase accumulator each sample:
  //   phaseInc_B += fmDepth * oscA_output * fmRatio_phaseInc_offset
  bool              fmEnabled     = false;  // OSC_A only: FM active for this board pair
  float             fmDepth       = 0.0f;   // OSC_A only: 0.0-1.0
  float             fmRatio       = 1.0f;   // OSC_B only: carrier:modulator ratio 0.5-8.0

  // --- Velocity ---
  // Normalised 0.0-1.0 at note-on. Scales voice output.
  // Also available as ROUTE_SRC_VELOCITY in V11.
  float             velocity      = 1.0f;

  // --- Level ---
  float             level         = 1.0f;

  // --- Fold modifier ---
  // Post-oscillator, pre-filter. foldBias shifts fold point asymmetrically.
  bool              foldEnabled   = false;
  float             foldAmount    = 0.5f;   // 0.0 = passthrough, 1.0 = max folds
  float             foldBias      = 0.0f;   // -1.0 to +1.0

  // --- S+H modifier ---
  // Independent clock at shRate Hz updates phaseInc with a random pitch
  // within shRange semitones of rootFreq.
  bool              shEnabled     = false;
  uint32_t          shPhase       = 0;      // internal S+H clock accumulator
  float             shRate        = 4.0f;   // Hz
  float             shRange       = 24.0f;  // semitones

  // --- Hard sync ---
  // Voice 0 is always sync master. syncEnabled must never be set on voice 0.
  bool              syncEnabled   = false;

  // --- Solo ---
  bool              soloEnabled   = false;

  // --- Chaos waveform internals ---
  float             prevPhaseNorm = 0.0f;  // internal — for wrap detection
  float             chaosX        = 0.5f;  // internal — evolves autonomously
  float             chaosR        = 3.7f;  // user parameter

  // --- SVF Filter integrators (internal) ---
  float             fltLow        = 0.0f;
  float             fltBand       = 0.0f;
  float             fltF          = 0.1f;
  float             fltDamp       = 1.4f;

  // --- Root frequency ---
  float             rootFreq      = 440.0f;
};

// ==========================
// Global State — extern declarations
// All defined in synth_engine.cpp
// ==========================
extern uint8_t           NUM_VOICES_ACTIVE;
extern Voice             voices[MAX_VOICES];
extern portMUX_TYPE      voiceMux;
extern volatile PlayMode playMode;
extern FilterMode        filterMode;
extern float             cutoffHz;
extern float             resonance;
extern float             envAmount;
extern float             keyTracking;
extern float             unisonDetuneCents;
extern float             masterLevel;
extern float             attackTime;
extern float             decayTime;
extern float             sustainLevel;
extern float             releaseTime;
extern float             attackCoeff;
extern float             decayCoeff;
extern float             releaseCoeff;
extern LFO               lfo1;
extern LFO               lfo2;
extern RingMod           ringMod;
extern Route             routes[4];
extern float             modWheelValue;   // MIDI CC1, normalised 0.0-1.0
extern float             tableA[TABLE_SIZE];
extern TaskHandle_t      audioTaskHandle;

// ==========================
// Function Declarations
// ==========================
void     setupI2S();
void     initWavetable();
void     updateEnvelopeRates();
void     updateFilterCoeffs(Voice &v);
float    midiToFreq(uint8_t note);
uint32_t freqToPhaseInc(float freq);
float    centsToRatio(float cents);
uint32_t shRandomPhaseInc(float rootFreq, float shRange);
float    chaosSeedFromNote(uint8_t note);
uint32_t voicePhaseInc(float baseFreq, int8_t octave, float detuneCents);
void     allNotesOff();
void     cloneVoice(uint8_t src);
int      allocateVoice();
int      allocateVoiceInGroup(int groupStart, int groupEnd);
void     audioTask(void *parameter);
