#pragma once
// ==============================================================
// synth_midi.h  —  Version 11.0
// MIDI byte processing, note-on/off, all-notes-off.
// Included by: synth_midi.cpp, synth_v10_1.ino
// ==============================================================

#include "synth_engine.h"

// ==========================
// Function Declarations
// ==========================
void noteOn(uint8_t note, uint8_t velocity);
void noteOff(uint8_t note);
void processMidiByte(uint8_t byte);
