// ==============================================================
// synth_can.cpp  —  Version 10.1
// TWAI (CAN) setup, system startup handshake, parameter dispatch, ACK.
// ==============================================================

#include <Arduino.h>
#include <driver/twai.h>
#include "synth_can.h"

// ==========================
// TWAI Pin Definitions
// ==========================
#define TWAI_TX_PIN  GPIO_NUM_7
#define TWAI_RX_PIN  GPIO_NUM_8

// ==========================
// CAN Sequence Counter
// ==========================
static uint8_t canSeq = 0;

// ==========================
// TWAI Setup
// ==========================
void setupTWAI()
{
  twai_general_config_t g_config = {
    .mode             = TWAI_MODE_NORMAL,
    .tx_io            = TWAI_TX_PIN,
    .rx_io            = TWAI_RX_PIN,
    .clkout_io        = TWAI_IO_UNUSED,
    .bus_off_io       = TWAI_IO_UNUSED,
    .tx_queue_len     = 10,
    .rx_queue_len     = 64,   // enough for a full parameter dump from all boards
    .alerts_enabled   = TWAI_ALERT_NONE,
    .clkout_divider   = 0
  };

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

void CAN_RequestDump(uint8_t boardId)
{
  twai_message_t req;
  req.identifier       = CAN_ID_REQUEST;
  req.extd             = 0;
  req.rtr              = 0;
  req.data_length_code = 1;
  req.data[0]          = boardId;
  twai_transmit(&req, pdMS_TO_TICKS(10));
}


// ==========================
// System Startup Handshake
// ==========================
// Broadcasts SYSTEM_READY, waits up to 2 seconds for boards to report in,
// sets NUM_VOICES_ACTIVE from detected oscillator boards,
// then broadcasts ALL_COMPLETE.
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

  uint32_t startTime     = millis();
  uint8_t  oscBoardsMask = 0x00;
  bool     commonsReady  = false;
  uint32_t lastBroadcast = 0;

  while (millis() - startTime < 2000)
  {
    if (millis() - lastBroadcast >= 200)
    {
      twai_transmit(&msg, pdMS_TO_TICKS(10));
      lastBroadcast = millis();
    }

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
    }
  }  // <-- this was missing

  uint8_t oscCount  = __builtin_popcount(oscBoardsMask);
  NUM_VOICES_ACTIVE = oscCount * 2;

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

  msg.data[1] = PARAM_SYSTEM_ALL_COMPLETE;
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  Serial.printf("System ready — %d voices active\n", NUM_VOICES_ACTIVE);
}
// ==========================
// CAN ACK LED State Helper
// ==========================
// Computes IC3 (16-bit) and IC4 (8-bit) LED state bytes for an oscillator board.
// boardIdx: 0 = voices 0+1,  1 = voices 2+3,  2 = voices 4+5.
//
// IC3 bits driven by ESP32 (oscillator boards):
//   bit 2 = OSC_A fold enabled   bit 3 = OSC_A S&H enabled
//   bit 9 = OSC_B fold enabled   bit 8 = OSC_B S&H enabled
//
// IC4 bits driven by ESP32 (oscillator boards):
//   bit 0 = CLONE_A  bit 1 = SOLO_A  bit 2 = SYNC_A
//   bit 3 = SOLO_B   bit 4 = CLONE_B bit 5 = SYNC_B
//
// CLONE bits: CLONE is a momentary action — the caller passes isCloneMsg=true
// when the ACK is responding to a PARAM_OSC_CLONE message so the LED can confirm.
static void computeOscBoardLeds(int boardIdx, bool cloneA, bool cloneB,
                                uint16_t &ic3, uint8_t &ic4)
{
  ic3 = 0;
  ic4 = 0;

  int vA = boardIdx * 2;
  int vB = boardIdx * 2 + 1;

  if (vA < MAX_VOICES)
  {
    if (voices[vA].foldEnabled) ic3 |= (1 << 2);
    if (voices[vA].shEnabled)   ic3 |= (1 << 3);
    if (cloneA)                 ic4 |= (1 << 0);
    if (voices[vA].soloEnabled) ic4 |= (1 << 1);
    if (voices[vA].fmEnabled)   ic4 |= (1 << 2);  // repurposed: was SYNC_A, now FM enabled
  }
  if (vB < MAX_VOICES)
  {
    if (voices[vB].shEnabled)   ic3 |= (1 << 8);
    if (voices[vB].foldEnabled) ic3 |= (1 << 9);
    if (voices[vB].soloEnabled) ic4 |= (1 << 3);
    if (cloneB)                 ic4 |= (1 << 4);
    if (voices[vB].syncEnabled) ic4 |= (1 << 5);
  }
}

// ==========================
// CAN ACK
// ==========================
// Sends a 5-byte ACK to CAN_ID_ACK.
//
// Frame layout:
//   Byte 0: Target echo
//   Byte 1: Param echo
//   Byte 2: IC3 high byte (bits 15-8)
//   Byte 3: IC3 low byte  (bits 7-0)
//   Byte 4: IC4 LED byte
//
// For oscillator-board targets, LED bytes carry current voice state so the
// ATmega can update its IC3/IC4 IO expanders without needing local state.
// For non-oscillator targets, LED bytes are 0x00.

// Compute IC3 remote LED state for the commons board from current engine state.
// Bit layout matches synth_params.h CAN_ID_ACK comment for commons board:
//   bit 1=UNISON  bit 2=DUO     bit 3=POLY
//   bit 9=LOWPASS bit 10=BANDPASS bit 11=HIGHPASS
//   bit 12=MOD_MUTE bit 15=RING_ENABLED
static uint16_t computeCommonsIC3()
{
  uint16_t ic3 = 0;

  switch (playMode)
  {
    case MODE_POLY:   ic3 |= (1 << 3); break;
    case MODE_DUO:    ic3 |= (1 << 2); break;
    case MODE_UNISON: ic3 |= (1 << 1); break;
  }

  switch (filterMode)
  {
    case FILT_LOW:  ic3 |= (1 << 9);  break;
    case FILT_BAND: ic3 |= (1 << 10); break;
    case FILT_HIGH: ic3 |= (1 << 11); break;
  }

  if (ringMod.enabled) ic3 |= (1 << 15);
  if (ringMod.modMute) ic3 |= (1 << 12);

  return ic3;
}


void sendCanAck(uint8_t target, uint8_t param, uint8_t /*seq*/)
{
  uint16_t ic3 = 0;
  uint8_t  ic4 = 0;

  if (target >= TARGET_OSC1 && target <= TARGET_OSC8)
  {
    int  v        = target - TARGET_OSC1;
    int  boardIdx = v / 2;
    int  vA       = boardIdx * 2;
    bool cloneA   = (param == PARAM_OSC_CLONE && v == vA);
    bool cloneB   = (param == PARAM_OSC_CLONE && v != vA);
    computeOscBoardLeds(boardIdx, cloneA, cloneB, ic3, ic4);
  }
  else
  {
    // For all non-OSC targets (filter, envelope, global, LFO, ringmod)
    // the ACK goes to the commons board — send current commons IC3 state
    // so it can maintain its remote LEDs correctly.
    ic3 = computeCommonsIC3();
     // Byte 4 for commons: bit 0 = LFO1 retrigger, bit 1 = LFO2 retrigger
    ic4 = (lfo1.retrigger ? 0x01 : 0x00) |
          (lfo2.retrigger ? 0x02 : 0x00);
  }

  twai_message_t ack;
  ack.identifier       = CAN_ID_ACK;
  ack.extd             = 0;
  ack.rtr              = 0;
  ack.data_length_code = 5;
  ack.data[0]          = target;
  ack.data[1]          = param;
  ack.data[2]          = (uint8_t)(ic3 >> 8);
  ack.data[3]          = (uint8_t)(ic3 & 0xFF);
  ack.data[4]          = ic4;
  twai_transmit(&ack, pdMS_TO_TICKS(5));
}

// ==========================
// CAN Message Processing
// ==========================
void processCanMessages()
{
  twai_message_t msg;

  while (twai_receive(&msg, 0) == ESP_OK)
  {
    // TEMPORARY DEBUG — remove after diagnosis
    Serial.printf("CAN RX  id=0x%03X  dlc=%d  data=", msg.identifier, msg.data_length_code);
    for (int i = 0; i < msg.data_length_code; i++)
    Serial.printf("%02X ", msg.data[i]);
    Serial.println();

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
           if (v % 2 != 0)  // Only odd voices can sync — even voices are always masters
          voices[v].syncEnabled = !voices[v].syncEnabled;
          break;
        case PARAM_OSC_SOLO:
          voices[v].soloEnabled = !voices[v].soloEnabled;
          break;
        case PARAM_OSC_CLONE:
          cloneVoice((uint8_t)constrain(value, 0, NUM_VOICES_ACTIVE - 1));
          break;
        case PARAM_OSC_FM_DEPTH:
          // OSC_A (even voice) only — depth of FM modulation 0.0-1.0
          if (v % 2 == 0)
            voices[v].fmDepth = decodeLinear(value, 0.0f, 1.0f);
          break;
        case PARAM_OSC_FM_RATIO:
          // OSC_B (odd voice) only — carrier:modulator ratio 0.5-8.0
          if (v % 2 != 0)
            voices[v].fmRatio = decodeExp(value, 0.5f, 8.0f);
          break;
        case PARAM_OSC_FM_ENABLE:
          // OSC_A (even voice) only — toggle FM for this board pair
          // Cancels sync on OSC_B if active, since FM and sync are mutually exclusive
          if (v % 2 == 0)
          {
            voices[v].fmEnabled = !voices[v].fmEnabled;
            if (voices[v].fmEnabled)
            {
              int vB = v + 1;
              if (vB < NUM_VOICES_ACTIVE)
                voices[vB].syncEnabled = false;
            }
          }
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
          // OSC1 SYNC button (individual per-voice sync from OSC1 panel)
          portENTER_CRITICAL(&voiceMux);
          for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
              if (i % 2 != 0)  // only sync slave voices
                  voices[i].syncEnabled = (value != 0);
          portEXIT_CRITICAL(&voiceMux);
          break;
        case PARAM_GLOBAL_SYNC_MASTER:
          // Sent by OSC1 board SYNC button with value = Board_GetOscOffset() (0 for board 0).
          // Value 0 means sync-all — same behaviour as PARAM_GLOBAL_SYNC_ALL with value=1.
           if (value == 0)
            {
              portENTER_CRITICAL(&voiceMux);
              for (int i = 0; i < NUM_VOICES_ACTIVE; i++)
                  if (i % 2 != 0)  // only sync slave voices
                      voices[i].syncEnabled = true;
              portEXIT_CRITICAL(&voiceMux);
            }
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
          lfo1.waveform = (LFOWaveform)constrain(value, 0, 3);
          break;
        case PARAM_LFO_RETRIGGER:
          // Sent as MOMENTARY (value=1 each press) — toggle state on each message
          lfo1.retrigger = !lfo1.retrigger;
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
          lfo2.waveform = (LFOWaveform)constrain(value, 0, 3);
          break;
        case PARAM_LFO_RETRIGGER:
          // Sent as MOMENTARY (value=1 each press) — toggle state on each message
          lfo2.retrigger = !lfo2.retrigger;
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
