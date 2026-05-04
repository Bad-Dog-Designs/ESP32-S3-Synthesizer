// ==============================================================
// synth_engine.cpp  —  Version 10.1
// Voice management, oscillator rendering, SVF filter, LFO, audio task.
// ==============================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <stdlib.h>
#include "synth_engine.h"

// ==========================
// I2S Pins
// ==========================
#define I2S_BCK   4
#define I2S_WS    5
#define I2S_DOUT  6

// ==========================
// Global Definitions
// ==========================
uint8_t           NUM_VOICES_ACTIVE = 6;
Voice             voices[MAX_VOICES];
portMUX_TYPE      voiceMux          = portMUX_INITIALIZER_UNLOCKED;
volatile PlayMode playMode          = MODE_POLY;
FilterMode        filterMode        = FILT_LOW;

float cutoffHz          = 800.0f;
float resonance         = 0.3f;
float envAmount         = 0.6f;
float keyTracking       = 0.5f;
float unisonDetuneCents = 25.0f;
float masterLevel       = 1.0f;

float attackTime    = 0.01f;
float decayTime     = 0.2f;
float sustainLevel  = 0.7f;
float releaseTime   = 0.3f;
float attackCoeff   = 0.0f;
float decayCoeff    = 0.0f;
float releaseCoeff  = 0.0f;

LFO     lfo1;
LFO     lfo2;
RingMod ringMod;
Route   routes[4];

float        modWheelValue = 0.0f;
float        tableA[TABLE_SIZE];
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
// Wavetable Initialisation
// ==========================
void initWavetable()
{
  for (int i = 0; i < TABLE_SIZE; i++)
    tableA[i] = sinf(2.0f * PI * i / TABLE_SIZE);
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
// Copies timbre parameters from src to all other voices.
// syncEnabled intentionally NOT cloned — sync is per-voice hardware button.
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

  // Tier 2: releasing voice — skip voices currently in attack
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
// Post-oscillator modifier. amount: 0.0 = passthrough, 1.0 = max folds.
// bias: DC offset before folding — shifts fold point, adds asymmetric harmonics.
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
// LFO Render
// ==========================
// Advances the LFO phase and returns the current output (-1.0 to +1.0).
// delayScale fades the output in from 0 to 1 over the delay period after note-on.
inline float renderLFO(LFO &lfo)
{
  float phaseInc = lfo.rate / (float)SAMPLE_RATE;
  lfo.phase += phaseInc;
  if (lfo.phase >= 1.0f) lfo.phase -= 1.0f;

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
    case LFO_CHAOS:
    {
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
      if (lfo.phase < phaseInc)
        lfo.shValue = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
      out = lfo.shValue;
      break;
    }
  }

  // Delay scale — fades LFO in after note-on
  if (lfo.delay > 0.0f)
  {
    lfo.delayTimer += 1.0f / (float)SAMPLE_RATE;
    float scale = fminf(lfo.delayTimer / lfo.delay, 1.0f);
    out *= scale;
  }

  return out;
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

    float rawOsc[MAX_VOICES] = {0};

    for (int i = 0; i < frames; i++)
    {
      float sample = 0.0f;

      // --- Advance LFOs ---
      float lfo1Out = renderLFO(lfo1);
      float lfo2Out = renderLFO(lfo2);
      // V11: apply lfo1Out and lfo2Out to routing destinations here
      (void)lfo1Out; (void)lfo2Out;

      
      // Sync trigger per board pair, not global voice 0
      bool syncTrigger[MAX_VOICES / 2] = {false};
      for (int b = 0; b < NUM_VOICES_ACTIVE / 2; b++)
      {
        int master = b * 2;
        if (voices[master].active)
        {
          uint32_t newPhase = voices[master].phase + voices[master].phaseInc;
          if (newPhase < voices[master].phase)
            syncTrigger[b] = true;
          }
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
        if (v % 2 == 1 && voices[v].syncEnabled && syncTrigger[v / 2])
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

        // Store raw osc for ring mod and FM use
        rawOsc[v] = osc;

        // --- FM modulation ---
        // OSC_A (even voice) modulates OSC_B (odd voice) phase on the same board.
        // Applied after OSC_A renders so its raw output is available.
        // OSC_B's phaseInc is offset by: fmDepth × oscA_output × fmRatio × basePhaseInc_B
        // This gives classic FM sidebands at fmRatio multiples of the carrier frequency.
        if (v % 2 == 0 && voices[v].fmEnabled)
        {
          int vB = v + 1;
          if (vB < NUM_VOICES_ACTIVE)
          {
            // FM offset scaled by fmDepth, modulator amplitude, and carrier phaseInc
            // fmRatio scales how far the modulator deflects the carrier phase
            float fmOffset = voices[v].fmDepth * osc
                           * voices[vB].fmRatio
                           * (float)voices[vB].phaseInc;
            voices[vB].phase += (uint32_t)(fmOffset);
          }
        }

        // --- Fold modifier ---
        if (voices[v].foldEnabled)
          osc = waveFold(osc, voices[v].foldAmount, voices[v].foldBias);

        // --- Ring modulator ---
        if (ringMod.enabled)
        {
          if (v == rmCarrier)
          {
            float modSig = rawOsc[rmMod];
            osc = osc * modSig * ringMod.level;
          }
          else if (v == rmMod && ringMod.modMute)
          {
            continue;
          }
        }

        // --- Filter ---
        float filtered = runFilter(voices[v], osc);

        // --- Sum: envelope × per-voice level × velocity ---
        sample += filtered * voices[v].envLevel * voices[v].level * voices[v].velocity;
      }

      // Normalise, apply master level, output
      //sample *= (1.0f / (float)NUM_VOICES_ACTIVE) * 4.0f * masterLevel;

      float gainBoost = (playMode == MODE_UNISON) ? 1.8f :
                  (playMode == MODE_DUO)    ? 2.0f : 4.0f;

        sample *= (1.0f / (float)NUM_VOICES_ACTIVE) * gainBoost * masterLevel;

      int16_t out = (int16_t)(sample * 20000);
      buffer[2*i]     = out;
      buffer[2*i + 1] = out;
    }

    size_t bytes_written;
    i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
  }
}
