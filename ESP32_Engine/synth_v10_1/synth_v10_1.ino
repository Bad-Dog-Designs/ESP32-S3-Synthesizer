// ==============================================================
// synth_v10_1.ino  —  Version 10.1  —  ESP32-S3
// ==============================================================
//
// Entry point: setup() and loop() only.
// All engine, MIDI and CAN logic lives in the respective modules:
//
//   synth_engine.h/.cpp  — Voice, oscillator, filter, LFO, audio task
//   synth_midi.h/.cpp    — MIDI byte processing, noteOn/Off
//   synth_can.h/.cpp     — TWAI setup, CAN message handler, ACK
//   synth_params.h       — Shared CAN parameter definitions (unchanged)
//
// ARCHITECTURE REFERENCE
// ======================
// BASE WAVEFORMS (6): SAW, SINE, TRI, PULSE, CHAOS, NOISE
// MODIFIERS (2):  FOLD (post-osc waveshaper), S+H (pitch randomiser)
// SIGNAL CHAIN:   S+H → phaseInc | Osc → Fold → SVF Filter → ×env×level×vel
// PLAY MODES:     POLY | DUO (2×3-voice unison) | UNISON
// HARD SYNC:      Voice 0 always master. Voices 1-5 can sync individually.
// LFO:            2× LFOs, 4 waveforms (SINE/TRI/CHAOS/S&H), delay + retrigger.
//                 No depth field — depth is per-route via Route.amount in V11.
// ==============================================================

#include "synth_engine.h"
#include "synth_midi.h"
#include "synth_can.h"

// --- MIDI ---
#define MIDI_BAUD    31250
#define MIDI_RX_PIN  16

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
  initWavetable();

  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
  {
    voices[i].rootFreq = 440.0f;
  }
  
  updateEnvelopeRates();
  // Spawn audio task on Core 1
  xTaskCreatePinnedToCore(
    audioTask,
    "AudioTask",
    8192,
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
 // One-shot parameter dump request after startup
  static bool dumpRequested = false;
  if (!dumpRequested)
  {
    dumpRequested = true;
    delay(100);
    for (uint8_t boardId = 0; boardId < 4; boardId++)
    {
      CAN_RequestDump(boardId);
      delay(50);
    }
  }

  while (Serial2.available())
    processMidiByte(Serial2.read());

  processCanMessages();
  taskYIELD();
}
