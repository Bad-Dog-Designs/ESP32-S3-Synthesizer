Project overview:
A custom polyphonic synthesizer built around an ESP32-S3 audio engine communicating with multiple ATmega328p control boards via CAN bus at 500kbps. All boards share synth_params.h as the single source of truth for CAN message definitions.
Hardware boards:

ESP32-S3 — audio engine, MIDI input, I2S DAC output, CAN master
Oscillator boards (ATmega328p, up to 4) — dual oscillator control per board, board ID set by hardware jumpers (both open = board 0)
Commons board (ATmega328p) — filter, envelope, LFO, ring mod, play mode controls
Routing board (ATmega328p) — modulation matrix UI, 12 routes, LED matrix display
Patch board (ATmega328p, pending) — SD card patch storage UI, keypad, VFD/starburst display

ESP32-S3 pin assignments:

I2S: GPIO 4(BCK), 5(WS), 6(DOUT)
CAN TX/RX: GPIO 7/8
MIDI RX: GPIO 16
SD card: GPIO 9(CD), 10(CS), 11(MOSI), 12(SCK), 13(MISO)

Current file set (V11.0):

synth_v11.ino — setup(), loop(), task spawn only
synth_engine.h — all type/struct definitions, extern globals, function prototypes
synth_engine.cpp — globals, voice management, oscillator, filter, LFO, FM, audio task
synth_midi.h/.cpp — MIDI byte processing, noteOn/Off, voice allocation
synth_can.h/.cpp — TWAI setup, systemStartup, processCanMessages, sendCanAck
synth_params.h — shared CAN parameter definitions, all boards

Architecture:

Audio runs on Core 1 via audioTask(), CAN+MIDI runs on Core 0 via loop()
portMUX_TYPE voiceMux protects voice array cross-core access
NUM_VOICES_ACTIVE set dynamically from detected oscillator boards at startup (2 per board)
TWAI rx_queue_len=64 to handle full parameter dumps

Voice model:

Up to 8 voices (MAX_VOICES), typically 6 active (3 oscillator boards)
Even voices (0,2,4,6) are FM/sync masters per board pair
Odd voices (1,3,5,7) are FM/sync slaves — can sync to or be FM-modulated by their partner
FM: OSC_A (even) modulates OSC_B (odd) phase. fmEnabled, fmDepth on even voice. fmRatio on odd voice
Hard sync: per-board-pair, not global. syncEnabled only valid on odd voices
Solo: toggle per voice, restricts noteOn to soloed voices only
Clone: copies all timbre params including fmEnabled, fmDepth, fmRatio to all other voices. cloneActive global drives all clone LEDs on all boards simultaneously. Second press clears LEDs, cloned params remain

Signal chain per voice:
S+H → phaseInc | FM phase offset | Oscillator → Wavefold → SVF Filter → × envLevel × level × velocity
Waveforms: SAW, SINE, TRI, PULSE, CHAOS (logistic map), NOISE
Modifiers:

FOLD: post-oscillator waveshaper. foldEnabled, foldAmount (0-1), foldBias (-1 to +1)
S+H: independent pitch randomiser clock. shEnabled, shRate (0.5-40Hz), shRange (1-48 semitones)
FM: fmEnabled, fmDepth (0-1), fmRatio (0.5-8.0)

Play modes: POLY, DUO (half voices per note), UNISON (all voices per note)
Gain scaling by mode: UNISON=1.8f, DUO=2.0f, POLY=4.0f (tuned by ear)
SVF Filter: Low/Band/High pass. cutoffHz, resonance, envAmount (envelope→cutoff), keyTracking. Coefficients updated on each filter parameter change and at noteOn.
Envelope: ADSR. Times stored as float seconds. Coefficients (attackCoeff etc.) computed by updateEnvelopeRates() — must be called at startup and on any time change.
LFOs: 2× LFOs. 4 waveforms (SINE/TRI/CHAOS/S&H). Rate, delay, retrigger. No depth — depth is per-route. RETRIGGER is toggle: each CAN message flips state.
Ring modulator: 4 voice pair options. level, modMute.
Modulation routing (V11 — implementation pending):
