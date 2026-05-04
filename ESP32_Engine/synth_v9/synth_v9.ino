#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/twai.h>
#include <math.h>
#include <stdlib.h>
#include "synth_params.h"

// ==========================
// Configuration
// Version 9.0 — ESP32-S3
// ==========================
#define SAMPLE_RATE     48000
#define TABLE_SIZE      1024
#define TABLE_BITS      10
#define NUM_VOICES      4

// --- I2S Pins (ESP32-S3) ---
// GPIO 25/26 do not exist on S3 — reassigned to safe GPIOs.
// Avoid: 0, 3, 19, 20, 45, 46 (boot/USB), 35-37 (PSRAM if fitted)
#define I2S_BCK         4
#define I2S_WS          5
#define I2S_DOUT        6

// --- MIDI ---
#define MIDI_BAUD       31250
#define MIDI_RX_PIN     16

// --- TWAI (CAN) Pins ---
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
// FOLD and S+H are per-voice modifier booleans, not waveforms.
enum OscType { OSC_SAW, OSC_SINE, OSC_TRI, OSC_PULSE, OSC_CHAOS, OSC_NOISE };

// ==========================
// Filter Mode
// ==========================
enum FilterMode { FILT_LOW, FILT_BAND, FILT_HIGH };
FilterMode filterMode = FILT_LOW;

// ==========================
// Global Filter Parameters
// ==========================
float cutoffHz    = 800.0f;
float resonance   = 0.3f;
float envAmount   = 0.6f;
float keyTracking = 0.5f;

// ==========================
// Play Mode
// ==========================
enum PlayMode { MODE_POLY, MODE_UNISON };
volatile PlayMode playMode = MODE_POLY;

float unisonDetuneCents = 25.0f;

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
// LFO — Stub for V10
// ==========================
// Structure and enums are defined here so the CAN parameter handler
// can receive and store LFO values. Actual modulation is not applied
// until V10 when the hardware panel is available for testing.
enum LFOWaveform { LFO_SINE, LFO_TRI, LFO_SAW, LFO_PULSE, LFO_SH };
enum LFODest
{
  LFO_DEST_PITCH,
  LFO_DEST_CUTOFF,
  LFO_DEST_AMP,
  LFO_DEST_PULSE_WIDTH,
  LFO_DEST_FOLD_AMOUNT,
  LFO_DEST_FOLD_BIAS,
  LFO_DEST_CHAOS_R
};

struct LFO
{
  float       rate        = 1.0f;
  float       depth       = 0.0f;
  LFOWaveform waveform    = LFO_SINE;
  LFODest     destination = LFO_DEST_PITCH;
  float       phase       = 0.0f;
  bool        enabled     = false;
  // V10: add modulation application
};

LFO lfo;

// ==========================
// Voice Structure
// ==========================
struct Voice
{
  // --- Core oscillator state (volatile: read by Core 1, written by Core 0) ---
  volatile uint32_t phase         = 0;
  volatile uint32_t phaseInc      = 0;
  volatile bool     active        = false;
  volatile uint8_t  note          = 255;
  volatile EnvState envState      = ENV_IDLE;
  volatile float    envLevel      = 0.0f;

  // --- Waveform ---
  OscType           oscType       = OSC_SAW;
  float             pulseWidth    = 0.5f;

  // --- Pitch controls ---
  // octave     : -1 = down one octave, 0 = normal, +1 = up one octave
  // detuneCents: fine pitch offset, -50 to +50 cents
  // phaseOffset: starting position in waveform cycle (0.0–1.0 = 0°–360°)
  //              applied at note-on so voices start at different points,
  //              creates natural thickening in unison especially with fold
  int8_t            octave        = 0;
  float             detuneCents   = 0.0f;
  float             phaseOffset   = 0.0f;

  // --- Level ---
  float             level         = 1.0f;   // 0.0 to 1.0

  // --- Fold modifier ---
  // Applies post-oscillator, pre-filter to any waveform.
  // foldAmount: 0.0 = no fold (passthrough), 1.0 = maximum folds
  // foldBias  : DC offset applied before folding (-1.0 to +1.0)
  //             shifts the fold operating point, producing asymmetric
  //             harmonic content and a formant-like timbral shift
  bool              foldEnabled   = false;
  float             foldAmount    = 0.5f;
  float             foldBias      = 0.0f;

  // --- S+H modifier ---
  // When enabled, a dedicated clock (shPhase at shRate Hz) periodically
  // updates phaseInc with a new random pitch within shRange semitones
  // above rootFreq. Works with any waveform including chaos and fold.
  bool              shEnabled     = false;
  uint32_t          shPhase       = 0;
  float             shRate        = 4.0f;   // Hz, per-voice
  float             shRange       = 24.0f;  // semitones, per-voice

  // --- Sync modifier ---
  // When true, this voice's phase is reset to phaseOffset every time
  // voice 0's phase accumulator wraps — hard sync to voice 0.
  // Voice 0 itself should never have syncEnabled = true.
  bool              syncEnabled   = false;

  // --- Solo ---
  // When any voice has soloEnabled, only solo voices contribute audio.
  // All other voices are silenced (envelope still runs, just not output).
  bool              soloEnabled   = false;

  // --- Chaos oscillator state ---
  float             prevPhaseNorm = 0.0f;
  float             chaosX        = 0.5f;
  float             chaosR        = 3.7f;

  // --- State Variable Filter integrators ---
  float             fltLow        = 0.0f;
  float             fltBand       = 0.0f;
  float             fltF          = 0.1f;
  float             fltDamp       = 1.4f;

  // --- Root frequency (keyboard tracking + S+H anchor) ---
  float             rootFreq      = 440.0f;
};

Voice voices[NUM_VOICES];

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
    Serial.println("TWAI driver install failed");
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
// Sends acknowledgement back to the ATmega328p control board.
// The ATmega uses this to light the appropriate confirmation LED.
void sendCanAck(uint8_t target, uint8_t param, uint8_t seq)
{
  twai_message_t ack;
  ack.identifier         = CAN_ID_ACK;
  ack.extd               = 0;
  ack.rtr                = 0;
  ack.data_length_code   = 3;
  ack.data[0]            = target;
  ack.data[1]            = param;
  ack.data[2]            = seq;   // Echo sequence so ATmega can match to request

  twai_transmit(&ack, pdMS_TO_TICKS(5));
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
// Chaos Seed
// ==========================
float chaosSeedFromNote(uint8_t note)
{
  return 0.1f + (note / 127.0f) * 0.8f;
}

// ==========================
// Compute phaseInc for a voice
// Applies octave and detune on top of base frequency
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
  for (int i = 0; i < NUM_VOICES; i++)
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
// Copies all sound-design parameters from srcVoice to all other voices.
// Does not copy pitch, phase, or envelope state — only timbre parameters.
void cloneVoice(uint8_t src)
{
  if (src >= NUM_VOICES) return;

  portENTER_CRITICAL(&voiceMux);

  for (int i = 0; i < NUM_VOICES; i++)
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
    voices[i].syncEnabled = voices[src].syncEnabled;
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
  for (int i = 0; i < NUM_VOICES; i++)
    if (!voices[i].active) return i;

  // Tier 2: releasing voice, skip voices just assigned in this chord burst
  int steal = -1;
  for (int i = 0; i < NUM_VOICES; i++)
  {
    if (voices[i].envState == ENV_ATTACK) continue;
    if (steal == -1 || voices[i].envLevel < voices[steal].envLevel)
      steal = i;
  }

  if (steal != -1) return steal;

  // Tier 3: all voices in attack — steal the quietest regardless
  steal = 0;
  for (int i = 1; i < NUM_VOICES; i++)
    if (voices[i].envLevel < voices[steal].envLevel) steal = i;

  return steal;
}

// ==========================
// Note On / Note Off
// ==========================
void noteOn(uint8_t note, uint8_t velocity)
{
  float freq = midiToFreq(note);

  portENTER_CRITICAL(&voiceMux);

  if (playMode == MODE_POLY)
  {
    int v = allocateVoice();

    voices[v].note          = note;
    voices[v].phase         = (uint32_t)(voices[v].phaseOffset * 4294967296.0f);
    voices[v].phaseInc      = voicePhaseInc(freq, voices[v].octave, voices[v].detuneCents);
    voices[v].active        = true;
    voices[v].envState      = ENV_ATTACK;
    voices[v].shPhase       = 0;
    voices[v].prevPhaseNorm = 0.0f;
    voices[v].chaosX        = chaosSeedFromNote(note);
    voices[v].rootFreq      = freq;
    voices[v].fltLow        = 0.0f;
    voices[v].fltBand       = 0.0f;
    updateFilterCoeffs(voices[v]);
  }
  else // MODE_UNISON
  {
    // Automatic spread + per-voice octave/detune layered on top.
    // This allows independent voice character within the unison cluster.
    float spread = unisonDetuneCents * 1.5f;

    for (int i = 0; i < NUM_VOICES; i++)
    {
      float unisonOffset = -spread / 2.0f + (spread / 3.0f) * i;
      float spreadFreq   = freq * centsToRatio(unisonOffset);

      voices[i].note          = note;
      voices[i].phase         = (uint32_t)(voices[i].phaseOffset * 4294967296.0f);
      voices[i].phaseInc      = voicePhaseInc(spreadFreq, voices[i].octave, voices[i].detuneCents);
      voices[i].active        = true;
      voices[i].envState      = ENV_ATTACK;
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

  for (int i = 0; i < NUM_VOICES; i++)
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
      // MIDI All Notes Off (CC 123)
      allNotesOff();
    }

    midiIndex = 0;
  }
}

// ==========================
// CAN Message Processing
// ==========================
// Receives parameter update messages from ATmega328p control boards,
// applies them to the engine, and sends ACK back.
// Called from loop() — non-blocking (0 tick timeout).
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

    // Determine which voice (0-based index from TARGET_OSC1–4)
    int v = -1;
    if (target >= TARGET_OSC1 && target <= TARGET_OSC4)
      v = target - TARGET_OSC1;

    // --- Oscillator Parameters ---
    if (v >= 0 && v < NUM_VOICES)
    {
      switch (param)
      {
        case PARAM_OSC_WAVEFORM:
          voices[v].oscType = (OscType)constrain(value, 0, 5);
          break;

        case PARAM_OSC_OCTAVE:
          // 0=down, 1=centre, 2=up
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
          // Exponential mapping: 0.5 Hz to 40 Hz
          voices[v].shRate = decodeExp(value, 0.5f, 40.0f);
          break;

        case PARAM_OSC_SH_RANGE:
          voices[v].shRange = decodeLinear(value, 1.0f, 48.0f);
          break;

        case PARAM_OSC_SYNC_ENABLE:
          // Voice 0 cannot sync to itself — it is always the master
          if (v != 0)
            voices[v].syncEnabled = (value != 0);
          break;

        case PARAM_OSC_SOLO:
          voices[v].soloEnabled = (value != 0);
          break;

        case PARAM_OSC_CLONE:
          cloneVoice((uint8_t)constrain(value, 0, NUM_VOICES - 1));
          break;
      }
    }

    // --- Filter Parameters ---
    else if (target == TARGET_FILTER)
    {
      switch (param)
      {
        case PARAM_FILTER_CUTOFF:
          cutoffHz = decodeExp(value, 20.0f, 18000.0f);
          for (int i = 0; i < NUM_VOICES; i++)
            updateFilterCoeffs(voices[i]);
          break;

        case PARAM_FILTER_RESONANCE:
          resonance = decodeLinear(value, 0.0f, 0.97f);
          for (int i = 0; i < NUM_VOICES; i++)
            updateFilterCoeffs(voices[i]);
          break;

        case PARAM_FILTER_ENV_AMT:
          envAmount = decodeLinear(value, 0.0f, 1.0f);
          break;

        case PARAM_FILTER_KEY_TRACK:
          keyTracking = decodeLinear(value, 0.0f, 1.0f);
          for (int i = 0; i < NUM_VOICES; i++)
            updateFilterCoeffs(voices[i]);
          break;

        case PARAM_FILTER_MODE:
          filterMode = (FilterMode)constrain(value, 0, 2);
          break;
      }
    }

    // --- Envelope Parameters ---
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

    // --- Global Parameters ---
    else if (target == TARGET_GLOBAL)
    {
      switch (param)
      {
        case PARAM_GLOBAL_PLAY_MODE:
          playMode = (value == 0) ? MODE_POLY : MODE_UNISON;
          break;

        case PARAM_GLOBAL_UNI_DETUNE:
          unisonDetuneCents = decodeLinear(value, 0.0f, 50.0f);
          break;

        case PARAM_GLOBAL_PANIC:
          allNotesOff();
          break;
      }
    }

    // --- LFO Parameters (stored, not yet applied — V10) ---
    else if (target == TARGET_LFO)
    {
      switch (param)
      {
        case PARAM_LFO_RATE:
          lfo.rate = decodeExp(value, 0.01f, 20.0f);
          break;

        case PARAM_LFO_DEPTH:
          lfo.depth = decodeLinear(value, 0.0f, 1.0f);
          break;

        case PARAM_LFO_WAVEFORM:
          lfo.waveform = (LFOWaveform)constrain(value, 0, 4);
          break;

        case PARAM_LFO_DESTINATION:
          lfo.destination = (LFODest)constrain(value, 0, 6);
          break;
      }
    }

    // Send ACK for every successfully processed message
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
// Applied post-oscillator to any waveform when foldEnabled = true.
// amount : 0.0 = passthrough, 1.0 = heavy folding
// bias   : DC offset before folding (-1.0 to +1.0)
//          shifts fold operating point, adds asymmetric harmonics,
//          produces a formant-like timbral character
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
    {
      // White noise — rand() output mapped to -1 to +1.
      // No phase accumulator needed; pitch has no meaning for noise.
      // Run through the filter it produces classic analogue noise textures.
      osc = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
      break;
    }

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
// Audio Task (Core 1)
// ==========================
void audioTask(void *parameter)
{
  const int frames = 64;
  int16_t   buffer[frames * 2];

  while (true)
  {
    // --- Precompute per-voice S+H increments once per buffer ---
    uint32_t shPhaseIncs[NUM_VOICES];
    for (int v = 0; v < NUM_VOICES; v++)
      shPhaseIncs[v] = voices[v].shEnabled ? freqToPhaseInc(voices[v].shRate) : 0;

    // --- Check if any voice is soloed ---
    // Computed once per buffer — soloEnabled changes only from CAN messages
    // which arrive at control-rate, not audio-rate.
    bool anySolo = false;
    for (int v = 0; v < NUM_VOICES; v++)
      if (voices[v].soloEnabled) { anySolo = true; break; }

    for (int i = 0; i < frames; i++)
    {
      float sample = 0.0f;

      // --- Pre-detect voice 0 phase wrap for hard sync ---
      // Check before the voice loop so all slave voices can respond
      // within the same sample that voice 0 wraps.
      bool syncTrigger = false;
      if (voices[0].active)
      {
        uint32_t newPhase0 = voices[0].phase + voices[0].phaseInc;
        if (newPhase0 < voices[0].phase)  // uint32 overflow = wrap
          syncTrigger = true;
      }

      for (int v = 0; v < NUM_VOICES; v++)
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
        // If any voice is soloed, skip non-soloed voices.
        // Envelope still runs above to allow clean release.
        if (anySolo && !voices[v].soloEnabled) continue;

        // --- Hard Sync ---
        // Reset slave voice phase to its phaseOffset when voice 0 wraps.
        // Creates the classic hard sync harmonic reshaping effect.
        if (v > 0 && voices[v].syncEnabled && syncTrigger)
          voices[v].phase = (uint32_t)(voices[v].phaseOffset * 4294967296.0f);

        // --- S+H Modulation ---
        // Advances independent S+H clock and updates phaseInc on wrap.
        // Works with any oscType — the waveform jumps to a new random pitch.
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

        // --- Fold Modifier ---
        // Post-oscillator, pre-filter.
        // Applies to any waveform including chaos and noise.
        if (voices[v].foldEnabled)
          osc = waveFold(osc, voices[v].foldAmount, voices[v].foldBias);

        // --- Filter ---
        float filtered = runFilter(voices[v], osc);

        // --- Sum with envelope and per-voice level ---
        sample += filtered * voices[v].envLevel * voices[v].level;
      }

      sample *= 0.25f;

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

  // Comment out setupTWAI() until CAN hardware is available
  // setupTWAI();

  for (int i = 0; i < TABLE_SIZE; i++)
    tableA[i] = sinf(2.0f * PI * i / TABLE_SIZE);

  // -------------------------------------------------------
  // TEST VALUES — replace with CAN-driven values in production
  // Adjust these to audition any combination of features
  // -------------------------------------------------------

  // Voice 0 — Saw with fold modifier
  voices[0].oscType     = OSC_SAW;
  voices[0].foldEnabled = true;
  voices[0].foldAmount  = 0.6f;    // 0.0 = no fold, 1.0 = maximum
  voices[0].foldBias    = 0.0f;    // -1.0 to +1.0
  voices[0].shEnabled   = false;
  voices[0].octave      = 0;
  voices[0].detuneCents = 0.0f;
  voices[0].phaseOffset = 0.0f;
  voices[0].level       = 1.0f;
  voices[0].pulseWidth  = 0.5f;
  voices[0].chaosR      = 3.7f;
  voices[0].shRate      = 4.0f;
  voices[0].shRange     = 24.0f;
  voices[0].syncEnabled = false;
  voices[0].soloEnabled = false;

  // Voice 1 — Sine with S+H modifier
  voices[1].oscType     = OSC_SINE;
  voices[1].foldEnabled = false;
  voices[1].shEnabled   = true;
  voices[1].shRate      = 3.0f;    // pitch steps per second
  voices[1].shRange     = 12.0f;   // semitones above root
  voices[1].octave      = 0;
  voices[1].detuneCents = 5.0f;
  voices[1].phaseOffset = 0.25f;   // 90 degrees offset
  voices[1].level       = 0.8f;
  voices[1].pulseWidth  = 0.5f;
  voices[1].chaosR      = 3.7f;
  voices[1].syncEnabled = false;
  voices[1].soloEnabled = false;

  // Voice 2 — Chaos with both fold AND S+H active simultaneously
  voices[2].oscType     = OSC_CHAOS;
  voices[2].chaosR      = 3.7f;
  voices[2].foldEnabled = true;    // fold applied to chaos output
  voices[2].foldAmount  = 0.4f;
  voices[2].foldBias    = 0.1f;
  voices[2].shEnabled   = true;    // S+H also active — randomly steps chaos pitch
  voices[2].shRate      = 2.0f;
  voices[2].shRange     = 24.0f;
  voices[2].octave      = 0;
  voices[2].detuneCents = -5.0f;
  voices[2].phaseOffset = 0.5f;   // 180 degrees offset
  voices[2].level       = 0.7f;
  voices[2].pulseWidth  = 0.5f;
  voices[2].syncEnabled = false;
  voices[2].soloEnabled = false;

  // Voice 3 — Noise with fold, synced to voice 0
  voices[3].oscType     = OSC_NOISE;
  voices[3].foldEnabled = true;
  voices[3].foldAmount  = 0.3f;
  voices[3].foldBias    = 0.0f;
  voices[3].shEnabled   = false;
  voices[3].octave      = -1;      // one octave down
  voices[3].detuneCents = 0.0f;
  voices[3].phaseOffset = 0.75f;  // 270 degrees offset
  voices[3].level       = 0.6f;
  voices[3].pulseWidth  = 0.5f;
  voices[3].chaosR      = 3.7f;
  voices[3].syncEnabled = true;    // hard sync to voice 0
  voices[3].soloEnabled = false;

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

  // ADSR
  attackTime   = 0.02f;
  decayTime    = 0.3f;
  sustainLevel = 0.7f;
  releaseTime  = 0.4f;

  updateEnvelopeRates();

  for (int i = 0; i < NUM_VOICES; i++)
  {
    voices[i].rootFreq = 440.0f;
    updateFilterCoeffs(voices[i]);
  }

  xTaskCreatePinnedToCore(
    audioTask,
    "AudioTask",
    4096,
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
  // MIDI
  while (Serial2.available())
    processMidiByte(Serial2.read());

  // CAN — process all pending messages, send ACKs
  //processCanMessages();

  taskYIELD();
}
