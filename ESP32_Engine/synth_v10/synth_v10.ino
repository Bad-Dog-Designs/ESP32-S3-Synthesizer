#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/twai.h>
#include <math.h>
#include <stdlib.h>
#include "synth_params.h"

// ==============================================================
// Synthesizer Engine  —  Version 10.0  —  ESP32-S3
// ==============================================================
//
// ARCHITECTURE REFERENCE
// ======================
//
// BASE WAVEFORMS (6) — selected via rotary switch per voice:
//   SAW, SINE, TRI, PULSE, CHAOS, NOISE
//
// MODIFIERS (2) — per-voice boolean, applied to any waveform:
//   FOLD  — post-oscillator, pre-filter waveshaper
//           Parameters: foldAmount (0.0-1.0), foldBias (-1.0 to +1.0)
//   S+H   — periodically updates oscillator pitch to a random value
//           Parameters: shRate (Hz), shRange (semitones)
//
// SIGNAL CHAIN PER VOICE:
//   S+H clock → phaseInc update (pitch modulation)
//   Oscillator → raw waveform sample
//   Fold modifier → waveshaping (if enabled)
//   SVF Filter → lowpass / bandpass / highpass
//   × envLevel × level × velocity → voice contribution
//
// PHASE NAMING CONVENTION — IMPORTANT:
//   phase       : uint32 running accumulator, advances every sample,
//                 wraps on overflow. INTERNAL — never user-accessible.
//   phaseOffset : float 0.0-1.0, USER CONTROL set by hardware pot.
//                 Determines where in the waveform cycle the voice
//                 starts when a note is triggered. Applied ONCE at
//                 note-on to initialise phase, then has no further role.
//
// CHAOS WAVEFORM INTERNALS:
//   chaosR : user parameter (2.5-4.0) — controls ordered vs chaotic output
//   chaosX : internal state variable — the running logistic map value.
//            Seeded from MIDI note at note-on, evolves autonomously.
//            Never user-accessible.
//
// PLAY MODES:
//   POLY   — up to NUM_VOICES_ACTIVE independent voices
//   DUO    — voices 0-2 form group A (note 1), voices 3-5 form group B (note 2)
//            Each group behaves as a 3-voice unison cluster.
//            Allows 2-note polyphony with 3-oscillator unison per note.
//   UNISON — all active voices play every note together
//
// HARD SYNC:
//   Voice 0 (OSC1) is always the sync master.
//   Voices 1-5 (OSC2-6) can individually enable syncEnabled.
//   On OSC1 panel: SYNC button syncs ALL voices 1-5 simultaneously.
//   On OSC2-6 panels: individual SYNC buttons per voice.
//
// LFO DEPTH:
//   Intentionally absent from the LFO struct.
//   Modulation depth is set per-route via Route.amount (-1.0 to +1.0).
//   This allows one LFO to drive multiple destinations at different depths.
//
// ==============================================================

// ==========================
// Configuration
// ==========================
#define SAMPLE_RATE     48000
#define TABLE_SIZE      1024
#define TABLE_BITS      10
#define MAX_VOICES      8       // Maximum voices — sets array size at compile time
                                // Actual active voices determined at runtime from
                                // detected oscillator boards during startup handshake

uint8_t NUM_VOICES_ACTIVE = 6; // Runtime voice count — updated by systemStartup()
                                // Default 6 (3 oscillator boards detected)

// --- I2S Pins (ESP32-S3) ---
#define I2S_BCK         4
#define I2S_WS          5
#define I2S_DOUT        6

// --- MIDI ---
#define MIDI_BAUD       31250
#define MIDI_RX_PIN     16

// --- TWAI (CAN) ---
#define TWAI_TX_PIN     GPIO_NUM_7
#define TWAI_RX_PIN     GPIO_NUM_8

// ==========================
// Wavetable
// ==========================
float tableA[TABLE_SIZE];

// ==========================
// Oscillator Type
// ==========================
// Six base waveforms on the rotary switch.
// FOLD and S+H are modifier booleans, not waveforms — see Voice struct.
enum OscType { OSC_SAW, OSC_SINE, OSC_TRI, OSC_PULSE, OSC_CHAOS, OSC_NOISE };

// ==========================
// Filter Mode
// ==========================
enum FilterMode { FILT_LOW, FILT_BAND, FILT_HIGH };
FilterMode filterMode = FILT_LOW;

// ==========================
// Play Mode
// ==========================
enum PlayMode { MODE_POLY, MODE_DUO, MODE_UNISON };
volatile PlayMode playMode = MODE_POLY;

// ==========================
// Global Parameters
// ==========================
float cutoffHz         = 800.0f;
float resonance        = 0.3f;
float envAmount        = 0.6f;
float keyTracking      = 0.5f;
float unisonDetuneCents = 25.0f;
float masterLevel      = 1.0f;

// ==========================
// Exponential ADSR
// ==========================
enum EnvState { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

float attackTime   = 0.01f;
float decayTime    = 0.2f;
float sustainLevel = 0.7f;
float releaseTime  = 0.3f;

float attackCoeff, decayCoeff, releaseCoeff;

// ==========================
// LFO
// ==========================
// Note: no 'depth' field — modulation depth is per-route via Route.amount.
enum LFOWaveform { LFO_SINE, LFO_TRI, LFO_SAW, LFO_PULSE, LFO_CHAOS, LFO_SH };

struct LFO
{
  float       rate       = 1.0f;      // Hz
  float       delay      = 0.0f;      // seconds before LFO fades in after note-on
  LFOWaveform waveform   = LFO_SINE;
  bool        retrigger  = false;     // true = reset phase on every note-on

  // Internal state — not user accessible
  float       phase      = 0.0f;     // running phase accumulator 0.0-1.0
  float       shValue    = 0.0f;     // held value for S+H waveform
  float       chaosX     = 0.5f;     // logistic map state for CHAOS waveform
  float       chaosR     = 3.7f;     // fixed default — not a real-time control
  float       delayTimer = 0.0f;     // counts up from 0 at note-on
};

LFO lfo1;
LFO lfo2;

// ==========================
// Modulation Routing — V11
// Stubs present so CAN handler can receive and store route definitions.
// Modulation is NOT applied until V11.
// ==========================
struct Route
{
  bool    enabled   = false;
  uint8_t source    = ROUTE_SRC_LFO1;
  uint8_t dstTarget = ROUTE_DST_TARGET_FILTER;
  uint8_t dstParam  = ROUTE_DST_CUTOFF;
  float   amount    = 0.0f;   // -1.0 to +1.0, negative = inverted modulation
};

Route routes[4];

// ==========================
// Ring Modulator
// ==========================
struct RingMod
{
  bool    enabled  = false;
  uint8_t pair     = 0;        // 0=V0&V1 1=V2&V3 2=V0&V2 3=V1&V3
  float   level    = 1.0f;
  bool    modMute  = false;    // true = modulator voice silent in final mix
};

RingMod ringMod;

// ==========================
// Voice Structure
// ==========================
struct Voice
{
  // --- Core state (volatile: read Core 1, written Core 0) ---
  volatile uint32_t phase         = 0;    // INTERNAL running accumulator — see note above
  volatile uint32_t phaseInc      = 0;
  volatile bool     active        = false;
  volatile uint8_t  note          = 255;
  volatile EnvState envState      = ENV_IDLE;
  volatile float    envLevel      = 0.0f;

  // --- Waveform ---
  OscType           oscType       = OSC_SAW;
  float             pulseWidth    = 0.5f;

  // --- Pitch ---
  // phaseOffset: USER CONTROL — waveform start position (0.0-1.0 = 0°-360°)
  //              Applied once at note-on to initialise phase accumulator.
  //              Most audible in unison mode — voices starting at different
  //              offsets interact to produce natural chorus-like thickening.
  int8_t            octave        = 0;       // -1, 0, +1
  float             detuneCents   = 0.0f;    // -50 to +50 cents
  float             phaseOffset   = 0.0f;    // 0.0-1.0 (0°-360°)

  // --- Velocity ---
  // Stored at note-on from MIDI velocity byte, normalised 0.0-1.0.
  // Scales voice output — velocity-sensitive amplitude response.
  // Also available as a routing source (ROUTE_SRC_VELOCITY) for V11.
  float             velocity      = 1.0f;

  // --- Level ---
  float             level         = 1.0f;

  // --- Fold modifier ---
  // Post-oscillator, pre-filter. Applies to any waveform.
  // foldAmount : 0.0 = passthrough, 1.0 = maximum folds
  // foldBias   : DC offset before folding (-1.0 to +1.0)
  //              shifts fold point, adds asymmetric harmonics,
  //              produces formant-like timbral character
  bool              foldEnabled   = false;
  float             foldAmount    = 0.5f;
  float             foldBias      = 0.0f;

  // --- S+H modifier ---
  // Independent clock running at shRate Hz updates phaseInc with
  // a random pitch within shRange semitones of rootFreq.
  // Works with any waveform including CHAOS and NOISE.
  bool              shEnabled     = false;
  uint32_t          shPhase       = 0;       // internal S+H clock accumulator
  float             shRate        = 4.0f;    // Hz
  float             shRange       = 24.0f;   // semitones

  // --- Hard sync ---
  // When true, phase resets to phaseOffset every time voice 0 wraps.
  // Voice 0 is always sync master — syncEnabled must never be set on voice 0.
  // OSC1 panel SYNC button: syncs ALL voices 1-5 (PARAM_GLOBAL_SYNC_ALL)
  // OSC2-6 panels: individual sync per voice (PARAM_OSC_SYNC_ENABLE)
  bool              syncEnabled   = false;

  // --- Solo ---
  bool              soloEnabled   = false;

  // --- Chaos waveform internals ---
  // chaosR : user parameter — character control
  // chaosX : INTERNAL logistic map state, seeded from MIDI note at note-on
  float             prevPhaseNorm = 0.0f;    // internal — for wrap detection
  float             chaosX        = 0.5f;    // internal — evolves autonomously
  float             chaosR        = 3.7f;    // user parameter

  // --- SVF Filter integrators (internal) ---
  float             fltLow        = 0.0f;
  float             fltBand       = 0.0f;
  float             fltF          = 0.1f;
  float             fltDamp       = 1.4f;

  // --- Root frequency ---
  float             rootFreq      = 440.0f;
};

Voice voices[MAX_VOICES];

// ==========================
// Thread Safety
// ==========================
portMUX_TYPE voiceMux = portMUX_INITIALIZER_UNLOCKED;

// ==========================
// MIDI
// ==========================
uint8_t midiStatus = 0;
uint8_t midiData[2];
uint8_t midiIndex = 0;

// ==========================
// CAN sequence counter
// ==========================
uint8_t canSeq = 0;

// ==========================
// Task Handle
// ==========================
TaskHandle_t audioTaskHandle;

// ==========================
// I2S Setup
// ==========================
void setupI2S()
{
  i2s_config_t config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags     = 0,
    .dma_buf_count        = 8,
    .dma_buf_len          = 64,
    .use_apll             = false,
    .tx_desc_auto_clear   = true,
    .fixed_mclk           = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_BCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// ==========================
// TWAI (CAN) Setup
// ==========================
void setupTWAI()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("TWAI install failed");
    return;
  }
  if (twai_start() != ESP_OK)
  {
    Serial.println("TWAI start failed");
    return;
  }
  Serial.println("TWAI started at 500kbps");
}

// ==========================
// CAN ACK
// ==========================
void sendCanAck(uint8_t target, uint8_t param, uint8_t seq)
{
  twai_message_t ack;
  ack.identifier       = CAN_ID_ACK;
  ack.extd             = 0;
  ack.rtr              = 0;
  ack.data_length_code = 3;
  ack.data[0]          = target;
  ack.data[1]          = param;
  ack.data[2]          = seq;
  twai_transmit(&ack, pdMS_TO_TICKS(5));
}

// ==========================
// System Startup Handshake
// ==========================
// Broadcasts SYSTEM_READY, waits for all boards to report in,
// determines active voice count from detected oscillator boards,
// then broadcasts ALL_COMPLETE so all boards know the system is ready.
// Times out after 2 seconds and uses defaults if boards don't respond.
void systemStartup()
{
  Serial.println("Broadcasting SYSTEM_READY...");

  twai_message_t msg;
  msg.identifier       = CAN_ID_SYSTEM;
  msg.extd             = 0;
  msg.rtr              = 0;
  msg.data_length_code = 2;
  msg.data[0]          = TARGET_SYSTEM;
  msg.data[1]          = PARAM_SYSTEM_READY;
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  uint32_t startTime      = millis();
  uint8_t  oscBoardsMask  = 0x00;  // bits 0-3, one per oscillator board
  bool     commonsReady   = false;

  while (millis() - startTime < 2000)
  {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      if (rx.data_length_code >= 3 &&
          rx.data[0] == TARGET_SYSTEM &&
          rx.data[1] == PARAM_SYSTEM_BOARD_COMPLETE)
      {
        uint8_t boardID   = rx.data[2];
        uint8_t boardType = (rx.data_length_code >= 4) ? rx.data[3] : BOARD_TYPE_OSC;

        if (boardType == BOARD_TYPE_OSC && boardID < 4)
        {
          oscBoardsMask |= (1 << boardID);
          Serial.printf("Oscillator board %d ready\n", boardID);
        }
        else if (boardType == BOARD_TYPE_COMMONS)
        {
          commonsReady = true;
          Serial.println("Commons board ready");
        }
      }
      else
      {
        // Process as a normal parameter update during startup window
        // (boards send their parameters after SYSTEM_READY)
      }
    }
  }

  // Determine active voice count from detected oscillator boards
  uint8_t oscCount     = __builtin_popcount(oscBoardsMask);
  NUM_VOICES_ACTIVE    = oscCount * 2;

  if (NUM_VOICES_ACTIVE == 0)
  {
    Serial.println("WARNING: No oscillator boards detected — using default 6 voices");
    NUM_VOICES_ACTIVE = 6;
  }
  else
  {
    Serial.printf("Detected %d oscillator boards — %d voices active\n",
                  oscCount, NUM_VOICES_ACTIVE);
  }

  if (!commonsReady)
    Serial.println("WARNING: Commons board not detected — using defaults");

  // Broadcast ALL_COMPLETE — boards use this to confirm system is live
  msg.data[1] = PARAM_SYSTEM_ALL_COMPLETE;
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  Serial.printf("System ready — %d voices active\n", NUM_VOICES_ACTIVE);
}

// ==========================
// Envelope Coefficients
// ==========================
float calcCoeff(float timeSeconds)
{
  if (timeSeconds <= 0.0f) return 1.0f;
  return 1.0f - expf(-1.0f / (timeSeconds * SAMPLE_RATE));
}

void updateEnvelopeRates()
{
  attackCoeff  = calcCoeff(attackTime);
  decayCoeff   = calcCoeff(decayTime);
  releaseCoeff = calcCoeff(releaseTime);
}

// ==========================
// Frequency Helpers
// ==========================
float midiToFreq(uint8_t note)
{
  return 440.0f * powf(2.0f, (note - 69) / 12.0f);
}

uint32_t freqToPhaseInc(float freq)
{
  return (uint32_t)((freq * 4294967296.0f) / SAMPLE_RATE);
}

float centsToRatio(float cents)
{
  return powf(2.0f, cents / 1200.0f);
}

// ==========================
// Filter Coefficient Update
// ==========================
void updateFilterCoeffs(Voice &v)
{
  float trackOffset     = (v.rootFreq - 261.63f) * keyTracking;
  float effectiveCutoff = cutoffHz + trackOffset;

  if (effectiveCutoff < 20.0f)    effectiveCutoff = 20.0f;
  if (effectiveCutoff > 18000.0f) effectiveCutoff = 18000.0f;

  v.fltF    = 2.0f * sinf(PI * effectiveCutoff / (float)SAMPLE_RATE);
  v.fltDamp = 2.0f * (1.0f - resonance);

  if (v.fltDamp < 0.005f) v.fltDamp = 0.005f;
  if (v.fltDamp > 2.0f)   v.fltDamp = 2.0f;
}

// ==========================
// S+H Random PhaseInc
// ==========================
uint32_t shRandomPhaseInc(float rootFreq, float shRange)
{
  float semitones = ((float)rand() / (float)RAND_MAX) * shRange;
  float freq      = rootFreq * powf(2.0f, semitones / 12.0f);
  return freqToPhaseInc(freq);
}

// ==========================
// Chaos Seed from MIDI Note
// ==========================
float chaosSeedFromNote(uint8_t note)
{
  return 0.1f + (note / 127.0f) * 0.8f;
}

// ==========================
// Voice PhaseInc with Octave + Detune
// ==========================
uint32_t voicePhaseInc(float baseFreq, int8_t octave, float detuneCents)
{
  float freq = baseFreq * powf(2.0f, (float)octave);
  freq      *= centsToRatio(detuneCents);
  return freqToPhaseInc(freq);
}

// ==========================
// All Notes Off
// ==========================
void allNotesOff()
{
  portENTER_CRITICAL(&voiceMux);
  for (int i = 0; i < MAX_VOICES; i++)
  {
    voices[i].envLevel = 0.0f;
    voices[i].envState = ENV_IDLE;
    voices[i].active   = false;
    voices[i].note     = 255;
  }
  portEXIT_CRITICAL(&voiceMux);
}

// ==========================
// Clone Voice
// ==========================
void cloneVoice(uint8_t src)
{
  if (src >= NUM_VOICES_ACTIVE) return;

  portENTER_CRITICAL(&voiceMux);

  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
  {
    if (i == src) continue;

    voices[i].oscType     = voices[src].oscType;
    voices[i].pulseWidth  = voices[src].pulseWidth;
    voices[i].chaosR      = voices[src].chaosR;
    voices[i].foldEnabled = voices[src].foldEnabled;
    voices[i].foldAmount  = voices[src].foldAmount;
    voices[i].foldBias    = voices[src].foldBias;
    voices[i].shEnabled   = voices[src].shEnabled;
    voices[i].shRate      = voices[src].shRate;
    voices[i].shRange     = voices[src].shRange;
    voices[i].octave      = voices[src].octave;
    voices[i].detuneCents = voices[src].detuneCents;
    voices[i].phaseOffset = voices[src].phaseOffset;
    voices[i].level       = voices[src].level;
    // syncEnabled intentionally NOT cloned — sync is per-voice hardware button
  }

  portEXIT_CRITICAL(&voiceMux);
}

// ==========================
// Voice Allocation
// ==========================
int allocateVoice()
{
  // Tier 1: idle voice
  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    if (!voices[i].active) return i;

  // Tier 2: releasing voice, skip voices just assigned in this chord burst
  int steal = -1;
  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
  {
    if (voices[i].envState == ENV_ATTACK) continue;
    if (steal == -1 || voices[i].envLevel < voices[steal].envLevel)
      steal = i;
  }
  if (steal != -1) return steal;

  // Tier 3: all in attack — steal quietest
  steal = 0;
  for (int i = 1; i < NUM_VOICES_ACTIVE; i++)
    if (voices[i].envLevel < voices[steal].envLevel) steal = i;

  return steal;
}

// ==========================
// Voice Group Allocation (DUO mode)
// ==========================
// Returns the least-active voice within the specified group range.
int allocateVoiceInGroup(int groupStart, int groupEnd)
{
  for (int i = groupStart; i < groupEnd; i++)
    if (!voices[i].active) return i;

  int steal = groupStart;
  for (int i = groupStart; i < groupEnd; i++)
  {
    if (voices[i].envState == ENV_ATTACK) continue;
    if (voices[i].envLevel < voices[steal].envLevel) steal = i;
  }
  return steal;
}

// ==========================
// Note On / Note Off
// ==========================
void noteOn(uint8_t note, uint8_t velocity)
{
  float freq     = midiToFreq(note);
  float velNorm  = velocity / 127.0f;

  // Retrigger LFOs if enabled
  if (lfo1.retrigger) { lfo1.phase = 0.0f; lfo1.delayTimer = 0.0f; }
  if (lfo2.retrigger) { lfo2.phase = 0.0f; lfo2.delayTimer = 0.0f; }

  portENTER_CRITICAL(&voiceMux);

  if (playMode == MODE_POLY)
  {
    int v = allocateVoice();

    voices[v].note          = note;
    voices[v].phase         = (uint32_t)(voices[v].phaseOffset * 4294967296.0f);
    voices[v].phaseInc      = voicePhaseInc(freq, voices[v].octave, voices[v].detuneCents);
    voices[v].active        = true;
    voices[v].envState      = ENV_ATTACK;
    voices[v].velocity      = velNorm;
    voices[v].shPhase       = 0;
    voices[v].prevPhaseNorm = 0.0f;
    voices[v].chaosX        = chaosSeedFromNote(note);
    voices[v].rootFreq      = freq;
    voices[v].fltLow        = 0.0f;
    voices[v].fltBand       = 0.0f;
    updateFilterCoeffs(voices[v]);
  }
  else if (playMode == MODE_DUO)
  {
    // Group A: voices 0-2, Group B: voices 3-5
    // First note goes to whichever group has a free voice.
    // Second note goes to the other group.
    // If both groups occupied, steal from the group with the quietest voice.
    int halfVoices = NUM_VOICES_ACTIVE / 2;

    // Find which group to use — prefer group with free voices
    bool groupAFree = false;
    bool groupBFree = false;
    for (int i = 0; i < halfVoices; i++)
      if (!voices[i].active) { groupAFree = true; break; }
    for (int i = halfVoices; i < NUM_VOICES_ACTIVE; i++)
      if (!voices[i].active) { groupBFree = true; break; }

    int groupStart = 0;
    if (!groupAFree && groupBFree)
      groupStart = halfVoices;
    else if (groupAFree)
      groupStart = 0;
    else
    {
      // Both occupied — find quietest group
      float levelA = 0, levelB = 0;
      for (int i = 0; i < halfVoices; i++)          levelA += voices[i].envLevel;
      for (int i = halfVoices; i < NUM_VOICES_ACTIVE; i++) levelB += voices[i].envLevel;
      groupStart = (levelA <= levelB) ? 0 : halfVoices;
    }

    int groupEnd  = groupStart + halfVoices;
    float spread  = unisonDetuneCents * 1.5f;

    for (int i = groupStart; i < groupEnd; i++)
    {
      int    gi     = i - groupStart;
      float  offset = -spread / 2.0f + (spread / (float)(halfVoices - 1)) * gi;
      float  dfreq  = freq * centsToRatio(offset);

      voices[i].note          = note;
      voices[i].phase         = (uint32_t)(voices[i].phaseOffset * 4294967296.0f);
      voices[i].phaseInc      = voicePhaseInc(dfreq, voices[i].octave, voices[i].detuneCents);
      voices[i].active        = true;
      voices[i].envState      = ENV_ATTACK;
      voices[i].velocity      = velNorm;
      voices[i].shPhase       = 0;
      voices[i].prevPhaseNorm = 0.0f;
      voices[i].chaosX        = chaosSeedFromNote(note) + gi * 0.05f;
      voices[i].rootFreq      = freq;
      voices[i].fltLow        = 0.0f;
      voices[i].fltBand       = 0.0f;
      updateFilterCoeffs(voices[i]);
    }
  }
  else // MODE_UNISON
  {
    float spread = unisonDetuneCents * 1.5f;

    for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    {
      float offset = -spread / 2.0f + (spread / (float)(NUM_VOICES_ACTIVE - 1)) * i;
      float dfreq  = freq * centsToRatio(offset);

      voices[i].note          = note;
      voices[i].phase         = (uint32_t)(voices[i].phaseOffset * 4294967296.0f);
      voices[i].phaseInc      = voicePhaseInc(dfreq, voices[i].octave, voices[i].detuneCents);
      voices[i].active        = true;
      voices[i].envState      = ENV_ATTACK;
      voices[i].velocity      = velNorm;
      voices[i].shPhase       = 0;
      voices[i].prevPhaseNorm = 0.0f;
      voices[i].chaosX        = chaosSeedFromNote(note) + i * 0.05f;
      voices[i].rootFreq      = freq;
      voices[i].fltLow        = 0.0f;
      voices[i].fltBand       = 0.0f;
      updateFilterCoeffs(voices[i]);
    }
  }

  portEXIT_CRITICAL(&voiceMux);
}

void noteOff(uint8_t note)
{
  portENTER_CRITICAL(&voiceMux);

  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    if (voices[i].note == note && voices[i].active)
      voices[i].envState = ENV_RELEASE;

  portEXIT_CRITICAL(&voiceMux);
}

// ==========================
// MIDI Processing
// ==========================
void processMidiByte(uint8_t byte)
{
  if (byte & 0x80)
  {
    midiStatus = byte;
    midiIndex  = 0;
    return;
  }

  midiData[midiIndex++] = byte;

  if (midiIndex >= 2)
  {
    uint8_t command = midiStatus & 0xF0;

    if (command == 0x90)
    {
      uint8_t note     = midiData[0];
      uint8_t velocity = midiData[1];

      if (velocity == 0) noteOff(note);
      else               noteOn(note, velocity);
    }
    else if (command == 0x80)
    {
      noteOff(midiData[0]);
    }
    else if (command == 0xB0 && midiData[0] == 123)
    {
      allNotesOff();
    }

    midiIndex = 0;
  }
}

// ==========================
// CAN Message Processing
// ==========================
void processCanMessages()
{
  twai_message_t msg;

  while (twai_receive(&msg, 0) == ESP_OK)
  {
    if (msg.data_length_code < 4) continue;

    uint8_t  target = msg.data[0];
    uint8_t  param  = msg.data[1];
    uint16_t value  = ((uint16_t)msg.data[2] << 8) | msg.data[3];
    uint8_t  seq    = (msg.data_length_code >= 5) ? msg.data[4] : 0;

    int v = -1;
    if (target >= TARGET_OSC1 && target <= TARGET_OSC8)
      v = target - TARGET_OSC1;

    // --- Oscillator parameters ---
    if (v >= 0 && v < NUM_VOICES_ACTIVE)
    {
      switch (param)
      {
        case PARAM_OSC_WAVEFORM:
          voices[v].oscType = (OscType)constrain(value, 0, 5);
          break;
        case PARAM_OSC_OCTAVE:
          voices[v].octave = (int8_t)(constrain(value, 0, 2) - 1);
          break;
        case PARAM_OSC_DETUNE:
          voices[v].detuneCents = decodeLinear(value, -50.0f, 50.0f);
          break;
        case PARAM_OSC_PHASE:
          voices[v].phaseOffset = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_OSC_LEVEL:
          voices[v].level = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_OSC_PULSE_WIDTH:
          voices[v].pulseWidth = decodeLinear(value, 0.01f, 0.99f);
          break;
        case PARAM_OSC_CHAOS_R:
          voices[v].chaosR = decodeLinear(value, 2.5f, 4.0f);
          break;
        case PARAM_OSC_FOLD_ENABLE:
          voices[v].foldEnabled = (value != 0);
          break;
        case PARAM_OSC_FOLD_AMOUNT:
          voices[v].foldAmount = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_OSC_FOLD_BIAS:
          voices[v].foldBias = decodeLinear(value, -1.0f, 1.0f);
          break;
        case PARAM_OSC_SH_ENABLE:
          voices[v].shEnabled = (value != 0);
          break;
        case PARAM_OSC_SH_RATE:
          voices[v].shRate = decodeExp(value, 0.5f, 40.0f);
          break;
        case PARAM_OSC_SH_RANGE:
          voices[v].shRange = decodeLinear(value, 1.0f, 48.0f);
          break;
        case PARAM_OSC_SYNC_ENABLE:
          if (v != 0)  // Voice 0 cannot sync to itself
            voices[v].syncEnabled = (value != 0);
          break;
        case PARAM_OSC_SOLO:
          voices[v].soloEnabled = (value != 0);
          break;
        case PARAM_OSC_CLONE:
          cloneVoice((uint8_t)constrain(value, 0, NUM_VOICES_ACTIVE - 1));
          break;
      }
    }

    // --- Filter parameters ---
    else if (target == TARGET_FILTER)
    {
      switch (param)
      {
        case PARAM_FILTER_CUTOFF:
          cutoffHz = decodeExp(value, 20.0f, 18000.0f);
          for (int i = 0; i < NUM_VOICES_ACTIVE; i++) updateFilterCoeffs(voices[i]);
          break;
        case PARAM_FILTER_RESONANCE:
          resonance = decodeLinear(value, 0.0f, 0.97f);
          for (int i = 0; i < NUM_VOICES_ACTIVE; i++) updateFilterCoeffs(voices[i]);
          break;
        case PARAM_FILTER_ENV_AMT:
          envAmount = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_FILTER_KEY_TRACK:
          keyTracking = decodeLinear(value, 0.0f, 1.0f);
          for (int i = 0; i < NUM_VOICES_ACTIVE; i++) updateFilterCoeffs(voices[i]);
          break;
        case PARAM_FILTER_MODE:
          filterMode = (FilterMode)constrain(value, 0, 2);
          break;
      }
    }

    // --- Envelope parameters ---
    else if (target == TARGET_ENVELOPE)
    {
      switch (param)
      {
        case PARAM_ENV_ATTACK:
          attackTime = decodeExp(value, 0.001f, 10.0f);
          updateEnvelopeRates();
          break;
        case PARAM_ENV_DECAY:
          decayTime = decodeExp(value, 0.001f, 10.0f);
          updateEnvelopeRates();
          break;
        case PARAM_ENV_SUSTAIN:
          sustainLevel = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_ENV_RELEASE:
          releaseTime = decodeExp(value, 0.001f, 10.0f);
          updateEnvelopeRates();
          break;
      }
    }

    // --- Global parameters ---
    else if (target == TARGET_GLOBAL)
    {
      switch (param)
      {
        case PARAM_GLOBAL_PLAY_MODE:
          playMode = (PlayMode)constrain(value, 0, 2);
          break;
        case PARAM_GLOBAL_UNI_DETUNE:
          unisonDetuneCents = decodeLinear(value, 0.0f, 50.0f);
          break;
        case PARAM_GLOBAL_PANIC:
          allNotesOff();
          break;
        case PARAM_GLOBAL_MASTER_LEVEL:
          masterLevel = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_GLOBAL_SYNC_ALL:
          // OSC1 SYNC button — sync or unsync all voices 1-5 simultaneously
          portENTER_CRITICAL(&voiceMux);
          for (int i = 1; i < NUM_VOICES_ACTIVE; i++)
            voices[i].syncEnabled = (value != 0);
          portEXIT_CRITICAL(&voiceMux);
          break;
        case PARAM_GLOBAL_SYNC_MASTER:
          // Reserved — voice 0 is always sync master in this implementation
          break;
      }
    }

    // --- LFO 1 parameters ---
    else if (target == TARGET_LFO1)
    {
      switch (param)
      {
        case PARAM_LFO_RATE:
          lfo1.rate = decodeExp(value, 0.01f, 20.0f);
          break;
        case PARAM_LFO_DELAY:
          lfo1.delay = decodeExp(value, 0.001f, 5.0f);
          break;
        case PARAM_LFO_WAVEFORM:
          lfo1.waveform = (LFOWaveform)constrain(value, 0, 5);
          break;
        case PARAM_LFO_RETRIGGER:
          lfo1.retrigger = (value != 0);
          break;
      }
    }

    // --- LFO 2 parameters ---
    else if (target == TARGET_LFO2)
    {
      switch (param)
      {
        case PARAM_LFO_RATE:
          lfo2.rate = decodeExp(value, 0.01f, 20.0f);
          break;
        case PARAM_LFO_DELAY:
          lfo2.delay = decodeExp(value, 0.001f, 5.0f);
          break;
        case PARAM_LFO_WAVEFORM:
          lfo2.waveform = (LFOWaveform)constrain(value, 0, 5);
          break;
        case PARAM_LFO_RETRIGGER:
          lfo2.retrigger = (value != 0);
          break;
      }
    }

    // --- Ring modulator parameters ---
    else if (target == TARGET_RINGMOD)
    {
      switch (param)
      {
        case PARAM_RINGMOD_ENABLE:
          ringMod.enabled = (value != 0);
          break;
        case PARAM_RINGMOD_PAIRS:
          ringMod.pair = (uint8_t)constrain(value, 0, 3);
          break;
        case PARAM_RINGMOD_LEVEL:
          ringMod.level = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_RINGMOD_MUTE:
          ringMod.modMute = (value != 0);
          break;
      }
    }

    // --- Routing parameters — V11 stubs ---
    else if (target == TARGET_ROUTE)
    {
      // Route data received and stored — modulation application in V11
      switch (param)
      {
        case PARAM_ROUTE_SET:
        {
          uint8_t routeIndex = (value >> 12) & 0x0F;
          if (routeIndex < 4)
          {
            routes[routeIndex].source    = (value >> 8) & 0x0F;
            routes[routeIndex].dstTarget = (value >> 4) & 0x0F;
            routes[routeIndex].dstParam  = value & 0x0F;
          }
          break;
        }
        case PARAM_ROUTE_AMOUNT:
        {
          uint8_t routeIndex = (uint8_t)(value >> 12);
          if (routeIndex < 4)
            routes[routeIndex].amount = decodeLinear(value & 0x0FFF, -1.0f, 1.0f);
          break;
        }
        case PARAM_ROUTE_ENABLE:
        {
          uint8_t routeIndex = (uint8_t)(value >> 8);
          if (routeIndex < 4)
            routes[routeIndex].enabled = (value & 0xFF) != 0;
          break;
        }
        case PARAM_ROUTE_CLEAR:
        {
          uint8_t routeIndex = (uint8_t)constrain(value, 0, 3);
          routes[routeIndex].enabled = false;
          break;
        }
        case PARAM_ROUTE_CLEAR_ALL:
          for (int i = 0; i < 4; i++) routes[i].enabled = false;
          break;
      }
    }

    sendCanAck(target, param, seq);
  }
}

// ==========================
// PolyBLEP
// ==========================
inline float polyBLEP(float t, float dt)
{
  if (t < dt)
  {
    t /= dt;
    return t + t - t*t - 1.0f;
  }
  else if (t > 1.0f - dt)
  {
    t = (t - 1.0f) / dt;
    return t*t + t + t + 1.0f;
  }
  return 0.0f;
}

// ==========================
// Wavefolder
// ==========================
// Post-oscillator modifier. Applies to any waveform when foldEnabled = true.
// amount : 0.0 = passthrough, 1.0 = maximum folds
// bias   : DC offset before folding — shifts fold point, adds asymmetric harmonics
inline float waveFold(float input, float amount, float bias)
{
  if (amount < 0.001f) return input;

  float drive = 1.0f + amount * 4.0f;
  float x     = (input + bias) * drive;

  while (x >  1.0f) x =  2.0f - x;
  while (x < -1.0f) x = -2.0f - x;

  return x;
}

// ==========================
// LFO Render
// ==========================
// Advances the LFO phase and returns the current output value (-1.0 to +1.0).
// delayScale fades the output in from 0 to 1 over the delay period.
// Called once per sample in the audio task.
inline float renderLFO(LFO &lfo)
{
  // Advance phase
  float phaseInc = lfo.rate / (float)SAMPLE_RATE;
  lfo.phase += phaseInc;
  if (lfo.phase >= 1.0f) lfo.phase -= 1.0f;

  // Compute raw output
  float out = 0.0f;
  switch (lfo.waveform)
  {
    case LFO_SINE:
    {
      uint32_t idx = (uint32_t)(lfo.phase * TABLE_SIZE) & (TABLE_SIZE - 1);
      out = tableA[idx];
      break;
    }
    case LFO_TRI:
      out = (lfo.phase < 0.5f) ? (lfo.phase * 4.0f - 1.0f)
                                : (3.0f - lfo.phase * 4.0f);
      break;
    case LFO_SAW:
      out = lfo.phase * 2.0f - 1.0f;
      break;
    case LFO_PULSE:
      out = (lfo.phase < 0.5f) ? 1.0f : -1.0f;
      break;
    case LFO_CHAOS:
    {
      // Advance chaos map once per LFO cycle
      if (lfo.phase < phaseInc)
      {
        float x = lfo.chaosR * lfo.chaosX * (1.0f - lfo.chaosX);
        if (x < 0.0001f) x = 0.0001f;
        if (x > 0.9999f) x = 0.9999f;
        lfo.chaosX = x;
      }
      out = lfo.chaosX * 2.0f - 1.0f;
      break;
    }
    case LFO_SH:
    {
      // Sample new random value on each cycle wrap
      if (lfo.phase < phaseInc)
        lfo.shValue = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
      out = lfo.shValue;
      break;
    }
  }

  // Apply delay scale — fades LFO in after note-on if delay > 0
  if (lfo.delay > 0.0f)
  {
    lfo.delayTimer += 1.0f / (float)SAMPLE_RATE;
    float scale = fminf(lfo.delayTimer / lfo.delay, 1.0f);
    out *= scale;
  }

  return out;
}

// ==========================
// Oscillator Render
// ==========================
inline float renderOsc(OscType type, uint32_t phase, uint32_t phaseInc, float &pulseWidth)
{
  const float PHASE_SCALE = 1.0f / 4294967296.0f;
  float phaseNorm = phase * PHASE_SCALE;
  float dt        = phaseInc * PHASE_SCALE;
  float osc       = 0.0f;

  switch (type)
  {
    case OSC_SINE:
    {
      uint32_t idx = phase >> (32 - TABLE_BITS);
      osc = tableA[idx];
      break;
    }
    case OSC_SAW:
    {
      osc  = 2.0f * phaseNorm - 1.0f;
      osc -= polyBLEP(phaseNorm, dt);
      break;
    }
    case OSC_TRI:
    {
      float saw1 = 2.0f * phaseNorm - 1.0f;
      saw1 -= polyBLEP(phaseNorm, dt);
      float pn2 = phaseNorm + 0.5f;
      if (pn2 >= 1.0f) pn2 -= 1.0f;
      float saw2 = 2.0f * pn2 - 1.0f;
      saw2 -= polyBLEP(pn2, dt);
      osc = 0.5f * (saw1 - saw2);
      break;
    }
    case OSC_PULSE:
    {
      float pw = pulseWidth;
      if (pw < 0.01f) pw = 0.01f;
      if (pw > 0.99f) pw = 0.99f;
      float pulse = (phaseNorm < pw) ? 1.0f : -1.0f;
      pulse += polyBLEP(phaseNorm, dt);
      float ps = phaseNorm - pw;
      if (ps < 0.0f) ps += 1.0f;
      pulse -= polyBLEP(ps, dt);
      osc = pulse;
      break;
    }
    case OSC_NOISE:
      osc = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
      break;
    default:
      osc = 0.0f;
      break;
  }

  return osc;
}

// ==========================
// State Variable Filter
// ==========================
inline float runFilter(Voice &v, float input)
{
  float fltFmod = v.fltF * (1.0f + v.envLevel * envAmount * 8.0f);
  if (fltFmod > 1.9f) fltFmod = 1.9f;

  float high = input - v.fltLow - v.fltDamp * v.fltBand;
  v.fltBand += fltFmod * high;
  v.fltLow  += fltFmod * v.fltBand;

  if (v.fltLow  >  1.0f) v.fltLow  =  1.0f;
  if (v.fltLow  < -1.0f) v.fltLow  = -1.0f;
  if (v.fltBand >  1.0f) v.fltBand =  1.0f;
  if (v.fltBand < -1.0f) v.fltBand = -1.0f;

  switch (filterMode)
  {
    case FILT_LOW:  return v.fltLow;
    case FILT_BAND: return v.fltBand;
    case FILT_HIGH: return high;
    default:        return v.fltLow;
  }
}

// ==========================
// Ring Modulator Helper
// ==========================
// Returns the carrier and modulator voice indices for the current pair setting.
// Carrier index goes in carrierV, modulator in modV.
void getRingModPair(int &carrierV, int &modV)
{
  switch (ringMod.pair)
  {
    case 0: carrierV = 0; modV = 1; break;
    case 1: carrierV = 2; modV = 3; break;
    case 2: carrierV = 0; modV = 2; break;
    case 3: carrierV = 1; modV = 3; break;
    default: carrierV = 0; modV = 1; break;
  }
}

// ==========================
// Audio Task (Core 1)
// ==========================
void audioTask(void *parameter)
{
  const int frames = 64;
  int16_t   buffer[frames * 2];

  while (true)
  {
    // Precompute S+H phase increments once per buffer
    uint32_t shPhaseIncs[MAX_VOICES];
    for (int v = 0; v < NUM_VOICES_ACTIVE; v++)
      shPhaseIncs[v] = voices[v].shEnabled ? freqToPhaseInc(voices[v].shRate) : 0;

    // Solo check — once per buffer
    bool anySolo = false;
    for (int v = 0; v < NUM_VOICES_ACTIVE; v++)
      if (voices[v].soloEnabled) { anySolo = true; break; }

    // Ring mod voice indices — once per buffer
    int rmCarrier = 0, rmMod = 1;
    if (ringMod.enabled) getRingModPair(rmCarrier, rmMod);

    // Pre-render raw oscillator outputs for ring mod
    // These are computed before the main voice loop so both carrier
    // and modulator are available when the carrier voice processes.
    float rawOsc[MAX_VOICES] = {0};

    for (int i = 0; i < frames; i++)
    {
      float sample = 0.0f;

      // --- Advance LFOs ---
      float lfo1Out = renderLFO(lfo1);
      float lfo2Out = renderLFO(lfo2);
      // V11: apply lfo1Out and lfo2Out to routing destinations here

      // --- Sync trigger detection ---
      // Check if voice 0 will wrap this sample — used for hard sync
      bool syncTrigger = false;
      if (voices[0].active)
      {
        uint32_t newPhase0 = voices[0].phase + voices[0].phaseInc;
        if (newPhase0 < voices[0].phase) syncTrigger = true;
      }

      // --- Per-voice processing ---
      for (int v = 0; v < NUM_VOICES_ACTIVE; v++)
      {
        // --- Envelope ---
        switch (voices[v].envState)
        {
          case ENV_IDLE:
            voices[v].envLevel = 0.0f;
            break;

          case ENV_ATTACK:
            voices[v].envLevel += (1.0f - voices[v].envLevel) * attackCoeff;
            if (voices[v].envLevel > 0.999f)
            {
              voices[v].envLevel = 1.0f;
              voices[v].envState = ENV_DECAY;
            }
            break;

          case ENV_DECAY:
            voices[v].envLevel += (sustainLevel - voices[v].envLevel) * decayCoeff;
            if (fabsf(voices[v].envLevel - sustainLevel) < 0.001f)
              voices[v].envState = ENV_SUSTAIN;
            break;

          case ENV_SUSTAIN:
            voices[v].envLevel = sustainLevel;
            break;

          case ENV_RELEASE:
            voices[v].envLevel += (0.0f - voices[v].envLevel) * releaseCoeff;
            if (voices[v].envLevel < 0.0005f)
            {
              voices[v].envLevel = 0.0f;
              voices[v].envState = ENV_IDLE;
              portENTER_CRITICAL(&voiceMux);
              voices[v].active = false;
              voices[v].note   = 255;
              portEXIT_CRITICAL(&voiceMux);
            }
            break;
        }

        if (!voices[v].active) continue;

        // --- Solo gate ---
        if (anySolo && !voices[v].soloEnabled) continue;

        // --- Hard sync ---
        // Voice 0 is always master. Voices 1-5 reset to phaseOffset when voice 0 wraps.
        if (v > 0 && voices[v].syncEnabled && syncTrigger)
          voices[v].phase = (uint32_t)(voices[v].phaseOffset * 4294967296.0f);

        // --- S+H pitch modulation ---
        if (voices[v].shEnabled)
        {
          uint32_t prevSh    = voices[v].shPhase;
          voices[v].shPhase += shPhaseIncs[v];
          if (voices[v].shPhase < prevSh)
            voices[v].phaseInc = shRandomPhaseInc(voices[v].rootFreq, voices[v].shRange);
        }

        // --- Oscillator ---
        float osc = 0.0f;

        if (voices[v].oscType == OSC_CHAOS)
        {
          const float PHASE_SCALE = 1.0f / 4294967296.0f;
          float phaseNorm = voices[v].phase * PHASE_SCALE;

          if (phaseNorm < voices[v].prevPhaseNorm)
          {
            float x = voices[v].chaosR * voices[v].chaosX * (1.0f - voices[v].chaosX);
            if (x < 0.0001f) x = 0.0001f;
            if (x > 0.9999f) x = 0.9999f;
            voices[v].chaosX = x;
          }
          voices[v].prevPhaseNorm = phaseNorm;
          osc = voices[v].chaosX * 2.0f - 1.0f;
          voices[v].phase += voices[v].phaseInc;
        }
        else
        {
          osc = renderOsc(voices[v].oscType,
                          voices[v].phase,
                          voices[v].phaseInc,
                          voices[v].pulseWidth);
          voices[v].phase += voices[v].phaseInc;
        }

        // Store raw osc for ring mod use
        rawOsc[v] = osc;

        // --- Fold modifier ---
        if (voices[v].foldEnabled)
          osc = waveFold(osc, voices[v].foldAmount, voices[v].foldBias);

        // --- Ring modulator ---
        // The carrier voice output is replaced by (carrier × modulator).
        // The modulator voice is processed normally but optionally muted.
        if (ringMod.enabled)
        {
          if (v == rmCarrier)
          {
            // Apply ring mod — multiply carrier by modulator raw output
            float modSig = rawOsc[rmMod];
            osc = osc * modSig * ringMod.level;
          }
          else if (v == rmMod && ringMod.modMute)
          {
            // Modulator muted from output — skip summing
            continue;
          }
        }

        // --- Filter ---
        float filtered = runFilter(voices[v], osc);

        // --- Sum: envelope × per-voice level × velocity ---
        sample += filtered * voices[v].envLevel * voices[v].level * voices[v].velocity;
      }

      // Normalise, apply master level, output
      sample *= (1.0f / (float)NUM_VOICES_ACTIVE) * 4.0f * masterLevel;

      int16_t out = (int16_t)(sample * 20000);
      buffer[2*i]     = out;
      buffer[2*i + 1] = out;
    }

    size_t bytes_written;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
  }
}

// ==========================
// Setup
// ==========================
void setup()
{
  Serial.begin(115200);
  Serial2.begin(MIDI_BAUD, SERIAL_8N1, MIDI_RX_PIN, -1);

  srand(esp_random());

  setupI2S();
  setupTWAI();
  systemStartup();

  // Build sine wavetable
  for (int i = 0; i < TABLE_SIZE; i++)
    tableA[i] = sinf(2.0f * PI * i / TABLE_SIZE);

  // -------------------------------------------------------
  // TEST VALUES
  // Remove or comment out individual sections as CAN hardware
  // comes online — the CAN handler will override these values.
  // -------------------------------------------------------

  // Voice 0 — SAW, fold on, sync master (never set syncEnabled)
  voices[0].oscType     = OSC_SAW;
  voices[0].foldEnabled = true;
  voices[0].foldAmount  = 0.5f;
  voices[0].foldBias    = 0.0f;
  voices[0].shEnabled   = false;
  voices[0].octave      = 0;
  voices[0].detuneCents = 0.0f;
  voices[0].phaseOffset = 0.0f;
  voices[0].level       = 1.0f;
  voices[0].pulseWidth  = 0.5f;
  voices[0].chaosR      = 3.7f;
  voices[0].shRate      = 4.0f;
  voices[0].shRange     = 24.0f;
  voices[0].syncEnabled = false;  // Voice 0 is sync master — never true
  voices[0].soloEnabled = false;

  // Voice 1 — SINE, S+H on, synced to voice 0
  voices[1].oscType     = OSC_SINE;
  voices[1].foldEnabled = false;
  voices[1].shEnabled   = true;
  voices[1].shRate      = 3.0f;
  voices[1].shRange     = 12.0f;
  voices[1].octave      = 0;
  voices[1].detuneCents = 5.0f;
  voices[1].phaseOffset = 0.25f;
  voices[1].level       = 0.8f;
  voices[1].pulseWidth  = 0.5f;
  voices[1].chaosR      = 3.7f;
  voices[1].syncEnabled = true;
  voices[1].soloEnabled = false;

  // Voice 2 — CHAOS, fold + S+H both active
  voices[2].oscType     = OSC_CHAOS;
  voices[2].chaosR      = 3.7f;
  voices[2].foldEnabled = true;
  voices[2].foldAmount  = 0.4f;
  voices[2].foldBias    = 0.1f;
  voices[2].shEnabled   = true;
  voices[2].shRate      = 2.0f;
  voices[2].shRange     = 24.0f;
  voices[2].octave      = 0;
  voices[2].detuneCents = -5.0f;
  voices[2].phaseOffset = 0.5f;
  voices[2].level       = 0.7f;
  voices[2].pulseWidth  = 0.5f;
  voices[2].syncEnabled = false;
  voices[2].soloEnabled = false;

  // Voice 3 — NOISE, fold on, octave down
  voices[3].oscType     = OSC_NOISE;
  voices[3].foldEnabled = true;
  voices[3].foldAmount  = 0.3f;
  voices[3].foldBias    = 0.0f;
  voices[3].shEnabled   = false;
  voices[3].octave      = -1;
  voices[3].detuneCents = 0.0f;
  voices[3].phaseOffset = 0.75f;
  voices[3].level       = 0.6f;
  voices[3].pulseWidth  = 0.5f;
  voices[3].chaosR      = 3.7f;
  voices[3].syncEnabled = true;
  voices[3].soloEnabled = false;

  // Voice 4 — PULSE
  voices[4].oscType     = OSC_PULSE;
  voices[4].pulseWidth  = 0.3f;
  voices[4].foldEnabled = false;
  voices[4].shEnabled   = false;
  voices[4].octave      = 1;
  voices[4].detuneCents = 7.0f;
  voices[4].phaseOffset = 0.1f;
  voices[4].level       = 0.7f;
  voices[4].chaosR      = 3.7f;
  voices[4].syncEnabled = false;
  voices[4].soloEnabled = false;

  // Voice 5 — TRI, fold on
  voices[5].oscType     = OSC_TRI;
  voices[5].foldEnabled = true;
  voices[5].foldAmount  = 0.6f;
  voices[5].foldBias    = -0.1f;
  voices[5].shEnabled   = false;
  voices[5].octave      = 0;
  voices[5].detuneCents = -7.0f;
  voices[5].phaseOffset = 0.6f;
  voices[5].level       = 0.7f;
  voices[5].pulseWidth  = 0.5f;
  voices[5].chaosR      = 3.7f;
  voices[5].syncEnabled = true;
  voices[5].soloEnabled = false;

  // -------------------------------------------------------
  // Global parameters
  // -------------------------------------------------------
  cutoffHz          = 1200.0f;
  resonance         = 0.4f;
  envAmount         = 0.6f;
  keyTracking       = 0.5f;
  filterMode        = FILT_LOW;
  unisonDetuneCents = 25.0f;
  playMode          = MODE_POLY;
  masterLevel       = 1.0f;

  attackTime   = 0.02f;
  decayTime    = 0.3f;
  sustainLevel = 0.7f;
  releaseTime  = 0.4f;
  updateEnvelopeRates();

  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
  {
    voices[i].rootFreq = 440.0f;
    updateFilterCoeffs(voices[i]);
  }

  // LFO defaults
  lfo1.rate      = 2.0f;
  lfo1.delay     = 0.5f;
  lfo1.waveform  = LFO_SINE;
  lfo1.retrigger = false;
  lfo1.chaosX    = 0.3f;

  lfo2.rate      = 0.5f;
  lfo2.delay     = 0.0f;
  lfo2.waveform  = LFO_TRI;
  lfo2.retrigger = true;
  lfo2.chaosX    = 0.6f;

  // Ring mod defaults — disabled
  ringMod.enabled = false;
  ringMod.pair    = 0;
  ringMod.level   = 1.0f;
  ringMod.modMute = false;

  xTaskCreatePinnedToCore(
    audioTask,
    "AudioTask",
    8192,          // Increased from 4096 for 6-voice headroom
    NULL,
    3,
    &audioTaskHandle,
    1
  );
}

// ==========================
// Core 0 Loop — MIDI + CAN
// ==========================
void loop()
{
  while (Serial2.available())
    processMidiByte(Serial2.read());

  processCanMessages();

  taskYIELD();
}
