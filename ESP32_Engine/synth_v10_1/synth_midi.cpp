// ==============================================================
// synth_midi.cpp  —  Version 10.1
// MIDI byte processing, noteOn/Off voice allocation.
// ==============================================================

#include <Arduino.h>
#include "synth_midi.h"

// ==========================
// MIDI State
// ==========================
static uint8_t midiStatus = 0;
static uint8_t midiData[2];
static uint8_t midiIndex  = 0;

// ==========================
// Note On
// ==========================
void noteOn(uint8_t note, uint8_t velocity)
{
  float freq    = midiToFreq(note);
  float velNorm = velocity / 127.0f;

  // Retrigger LFOs if enabled
  if (lfo1.retrigger) { lfo1.phase = 0.0f; lfo1.delayTimer = 0.0f; }
  if (lfo2.retrigger) { lfo2.phase = 0.0f; lfo2.delayTimer = 0.0f; }

  // If any voice is soloed, restrict allocation to soloed voices only
  bool anySolo = false;
  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    if (voices[i].soloEnabled) { anySolo = true; break; }

  if (anySolo)
  {
    portENTER_CRITICAL(&voiceMux);
    for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    {
      if (!voices[i].soloEnabled) continue;
      voices[i].note          = note;
      voices[i].phase         = (uint32_t)(voices[i].phaseOffset * 4294967296.0f);
      voices[i].phaseInc      = voicePhaseInc(midiToFreq(note), voices[i].octave, voices[i].detuneCents);
      voices[i].active        = true;
      voices[i].envState      = ENV_ATTACK;
      voices[i].velocity      = velNorm;
      voices[i].shPhase       = 0;
      voices[i].prevPhaseNorm = 0.0f;
      voices[i].chaosX        = chaosSeedFromNote(note);
      voices[i].rootFreq      = midiToFreq(note);
      voices[i].fltLow        = 0.0f;
      voices[i].fltBand       = 0.0f;
      updateFilterCoeffs(voices[i]);
    }
    portEXIT_CRITICAL(&voiceMux);
    return;
  }

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
    // Group A: voices 0 to halfVoices-1
    // Group B: voices halfVoices to NUM_VOICES_ACTIVE-1
    // First note goes to whichever group has a free voice.
    // Second note goes to the other group.
    // If both groups occupied, steal from the quietest group.
    int halfVoices = NUM_VOICES_ACTIVE / 2;

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
      float levelA = 0, levelB = 0;
      for (int i = 0; i < halfVoices; i++)               levelA += voices[i].envLevel;
      for (int i = halfVoices; i < NUM_VOICES_ACTIVE; i++) levelB += voices[i].envLevel;
      groupStart = (levelA <= levelB) ? 0 : halfVoices;
    }

    int   groupEnd = groupStart + halfVoices;
    float spread   = unisonDetuneCents * 1.5f;

    for (int i = groupStart; i < groupEnd; i++)
    {
      int   gi     = i - groupStart;
      float offset = -spread / 2.0f + (spread / (float)(halfVoices - 1)) * gi;
      float dfreq  = freq * centsToRatio(offset);

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
  else  // MODE_UNISON
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

// ==========================
// Note Off
// ==========================
void noteOff(uint8_t note)
{
  portENTER_CRITICAL(&voiceMux);

  for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
    if (voices[i].note == note && voices[i].active)
      voices[i].envState = ENV_RELEASE;

  portEXIT_CRITICAL(&voiceMux);
}

// ==========================
// MIDI Byte Processing
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
    else if (command == 0xB0)
    {
      if (midiData[0] == 123)
      {
        // CC123 — all notes off
        allNotesOff();
      }
      else if (midiData[0] == 1)
      {
        // CC1 — mod wheel. Store normalised 0.0-1.0 for ROUTE_SRC_MODWHEEL (V11).
        modWheelValue = midiData[1] / 127.0f;
      }
    }

    midiIndex = 0;
  }
}
