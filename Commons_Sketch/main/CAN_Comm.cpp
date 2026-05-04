#include "CAN_Comm.h"
#include "param_map.h"
#include "Board.h"
#include "ADC.h"
#include "synth_params.h"

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// ================================================================
// CAN_Comm.cpp — Commons PCB
//
// Differences from Oscillator board CAN_Comm.cpp:
//
//  + BTN_MODE_CYCLER: N-state button (Filter Mode, Poly Mode)
//  + TOGGLE with paired LEDs (Env/Track button)
//  + No ENCODE_DETUNE / LED flash logic (no detune control)
//  + No extra shift buttons (A6/A7 are LED outputs on Commons)
//  + Pot catch still present for CH11 (Env/Track dual function)
//  + ENCODE_RANGED for CH5 (LFO1 waveform), CH10 (LFO2 waveform),
//    CH15 (Ring Mod pairs) — 4-position switches
//  + ACK frame byte 4 carries LFO retrigger LED state (bits 0-1)
//    rather than IC4 LED byte (IC4 has no outputs on Commons)
// ================================================================

MCP_CAN g_CAN(CAN_CS_PIN);

// ================================================================
// Resistor ladder range tables — 4-position switches
//
// Calibrated measurements:
//   LFO1:     0 / 253 / 510 / 768
//   LFO2:     0 / 254 / 511 / 769
//   Ring Mod: 0 / 253 / 511 / 769
//
// All three ladders are effectively identical — midpoints averaged:
//   Between pos 0 and 1: (253+254+253)/3 ≈ 253 → midpoint 126
//   Between pos 1 and 2: (510+511+511)/3 ≈ 511 → midpoint 382
//   Between pos 2 and 3: (768+769+769)/3 ≈ 769 → midpoint 640
//
// LFO Waveform: Sine=0 / Triangle=1 / Chaos=2 / S&H=3
// Ring Mod Pairs: OSC0&1=0 / OSC2&3=1 / OSC0&2=2 / OSC1&3=3
// ================================================================

static const StepRange_t lfoWaveformRanges[4] =
{
    {   0, 126, 0 },    // Sine
    { 127, 382, 1 },    // Triangle
    { 383, 640, 2 },    // Chaos
    { 641, 1023, 3 },   // S&H
};

static const StepRange_t ringModPairsRanges[4] =
{
    {   0, 126, 0 },    // OSC0&1
    { 127, 382, 1 },    // OSC2&3
    { 383, 640, 2 },    // OSC0&2
    { 641, 1023, 3 },   // OSC1&3
};

typedef struct { const StepRange_t *table; uint8_t count; } RangeRef_t;

static RangeRef_t getRangeTable(uint8_t ch)
{
    switch (ch)
    {
        case  5: case 10: return { lfoWaveformRanges,  4 };
        case 15:          return { ringModPairsRanges, 4 };
        default:          return { NULL, 0 };
    }
}

// ================================================================
// State
// ================================================================

static bool    btnToggleState[BTN_COUNT] = { false };
static uint8_t cyclerState[BTN_COUNT]   = { 0 };

// Pot catch for CH11 (Env/Track dual function)
static uint16_t lastSentPrimary[16];
static uint16_t lastSentSecondary[16];
static bool     catchMode[16];
static uint16_t catchTarget[16];
static uint16_t catchPrevRaw[16];
static uint16_t catchOriginRaw[16];  // pot position at moment catch was activated — never updated

// Switch settle state (CH5, CH10, CH15)
static int8_t   switchLastWindow[16];
static uint32_t switchWindowEntryMs[16];
static int8_t   switchLastSentWindow[16];

// IC3 LED state
static uint16_t ic3LocalLEDs  = 0;
static uint16_t ic3RemoteLEDs = 0;

// LFO retrigger LED state (remote — ESP32 via ACK byte 4)
static bool retrigLFO1 = false;
static bool retrigLFO2 = false;

static uint8_t seqNum = 0;

// ================================================================
// Internal helpers
// ================================================================

static uint16_t encodeValue(uint16_t raw, const ParamDef_t *def)
{
    switch (def->encoding)
    {
        case ENCODE_LINEAR:
        case ENCODE_EXP:
            return (uint16_t)((uint32_t)raw * 65535UL / 1023UL);

        case ENCODE_DETUNE:
            // Not used on Commons — treat as linear
            return (uint16_t)((uint32_t)raw * 65535UL / 1023UL);

        default:
            return 0;   // ENCODE_RANGED handled separately
    }
}

static void sendFrame(uint8_t canId, uint8_t target, uint8_t param, uint16_t value)
{
    // Commons board: no OSC target offsetting needed.
    // All targets are FILTER, ENVELOPE, GLOBAL, LFO1/2, RINGMOD.
    uint8_t data[5];
    data[0] = target;
    data[1] = param;
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)(value & 0xFF);
    data[4] = seqNum++;
    g_CAN.sendMsgBuf(canId, 0, 5, data);
}

// Rebuild ic3LocalLEDs from button states.
static void rebuildLocalLEDs(void)
{
    ic3LocalLEDs = 0;

    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];

        if (e->mode == BTN_MODE_TOGGLE || e->mode == BTN_MODE_LOCAL_TOGGLE)
        {
            // Paired LED: ledBit lit when ON, ledBitSecondary lit when OFF.
            if (btnToggleState[b])
            {
                if (e->ledBit >= 0)
                    ic3LocalLEDs |= (1u << e->ledBit);
            }
            else
            {
                if (e->ledBitSecondary >= 0)
                    ic3LocalLEDs |= (1u << e->ledBitSecondary);
            }
        }
        // CYCLER, MOMENTARY, LOCAL_TOGGLE (via above): no further action
    }
}

// Activate pot catch for CH11 on Env/Track button toggle
static void activateCatchForButton(Button_t btn, bool goingToSecondary)
{
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        if (adcMap[ch].shiftBtn != (int8_t)btn) continue;

        uint16_t target = goingToSecondary
                          ? lastSentSecondary[ch]
                          : lastSentPrimary[ch];

        if (target == 0xFFFF) continue;

        uint16_t currentRaw = ADC_GetChannelValue(ch);

        // If pot is already within CATCH_WINDOW of the target value, skip
        // the catch entirely — start sending immediately with no perceptible
        // jump. The two pot positions happen to be close enough.
        int16_t dist = (int16_t)currentRaw - (int16_t)target;
        if (dist < 0) dist = -dist;
        if ((uint16_t)dist <= CATCH_WINDOW) continue;

        catchMode[ch]      = true;
        catchTarget[ch]    = target;
        catchPrevRaw[ch]   = currentRaw;
        catchOriginRaw[ch] = currentRaw;  // snapshot — never updated during catch
    }
}

// ================================================================
// Public API
// ================================================================

bool CAN_Init(void)
{
    if (g_CAN.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) != CAN_OK)
        return false;

    g_CAN.setMode(MCP_NORMAL);
    pinMode(CAN_INT_PIN, INPUT);

    for (uint8_t i = 0; i < 16; i++)
    {
        lastSentPrimary[i]     = 0xFFFF;
        lastSentSecondary[i]   = 0xFFFF;
        catchMode[i]           = false;
        catchTarget[i]         = 0;
        catchPrevRaw[i]        = 0;
        catchOriginRaw[i]      = 0;
        switchLastWindow[i]    = -1;
        switchWindowEntryMs[i] = 0;
        switchLastSentWindow[i]= -1;
    }

    ic3RemoteLEDs = 0;
    retrigLFO1    = false;
    retrigLFO2    = false;
    seqNum        = 0;

    // Filter mode cycler: initialise to state 0 (Lowpass) — the power-up default.
    // First button press advances to 1 (Bandpass), giving LP → BP → HP → LP...
    // CAN_SendAllParameters will broadcast value=0, and the ESP32/tester ACK
    // will confirm LP — so the pre-lit LED stays on rather than being overwritten.
    cyclerState[BTN_FILTER_MODE] = 0;

    // Poly mode cycler: initialise to state 0 (Poly) — the power-up default.
    // First button press advances to 1 (Duo), giving Poly → Duo → Unison → Poly...
    cyclerState[BTN_POLY_MODE] = 0;

    // Pre-light LP + Poly so the panel is not dark before the first ESP32 ACK.
    // The stored cycler states (both 0) now match what CAN_SendAllParameters
    // broadcasts, so incoming ACKs will confirm these bits and leave them set.
    ic3RemoteLEDs = (1u << LED_BIT_LOWPASS) | (1u << LED_BIT_POLY);
    ic3RemoteLEDs &= LED_IC3_REMOTE_MASK;

    // Initial LED state: Track LED lit (Env/Track defaults to Track mode)
    rebuildLocalLEDs();

    // Announce presence — catches the case where we boot before the ESP32
    uint8_t announceData[4];
    announceData[0] = TARGET_SYSTEM;
    announceData[1] = PARAM_SYSTEM_BOARD_COMPLETE;
    announceData[2] = Board_GetID();
    announceData[3] = BOARD_TYPE_COMMONS;   // BOARD_TYPE_OSC on oscillator boards
    g_CAN.sendMsgBuf(CAN_ID_SYSTEM, 0, 4, announceData);

    return true;
}

// ----------------------------------------------------------------
void CAN_Task(void)
{
    if (digitalRead(CAN_INT_PIN) == HIGH) return;

    long unsigned int rxId;
    unsigned char     rxLen;
    unsigned char     rxData[8];

    if (g_CAN.readMsgBuf(&rxId, &rxLen, rxData) != CAN_OK) return;

     // ---- CAN_ID_SYSTEM: ESP32 startup handshake ----
    if (rxId == CAN_ID_SYSTEM && rxLen >= 2 &&
        rxData[0] == TARGET_SYSTEM &&
        rxData[1] == PARAM_SYSTEM_READY)
    {
        // ESP32 is (re)broadcasting SYSTEM_READY — respond immediately
        uint8_t announceData[4];
        announceData[0] = TARGET_SYSTEM;
        announceData[1] = PARAM_SYSTEM_BOARD_COMPLETE;
        announceData[2] = Board_GetID();
        announceData[3] = BOARD_TYPE_COMMONS;   // BOARD_TYPE_OSC on oscillator boards
        g_CAN.sendMsgBuf(CAN_ID_SYSTEM, 0, 4, announceData);

        // Also resend all parameters since ESP32 may have just reset
        CAN_SendAllParameters();
        return;
    }

    // ---- CAN_ID_REQUEST: ESP32 requests full parameter dump ----
    if (rxId == CAN_ID_REQUEST && rxLen >= 1)
    {
        if (rxData[0] == Board_GetID())
            CAN_SendAllParameters();
        return;
    }

    // ---- CAN_ID_ACK: ESP32 sends back LED state ----
    // ACK frame byte layout for Commons PCB:
    //   Byte 0: Target (echo)
    //   Byte 1: Param  (echo)
    //   Byte 2: IC3 remote LED high byte (bits 8-15)
    //   Byte 3: IC3 remote LED low byte  (bits 0-7)
    //   Byte 4: LFO retrigger LEDs (bit0=LFO1, bit1=LFO2)
    //
    // Guard: both boards receive every ACK on the bus. Only process frames
    // whose echoed target (byte 0) belongs to the Commons board. OSC targets
    // (0x01-0x08) carry IC4 LED bytes in byte 4 and must be silently dropped,
    // otherwise the oscillator board's ACKs would corrupt ic3RemoteLEDs and
    // the retrigger LED state here.
    if (rxId == CAN_ID_ACK && rxLen >= 5)
    {
        uint8_t echoTarget = rxData[0];

        // Accept only Commons targets: FILTER, ENVELOPE, GLOBAL,
        // LFO1, LFO2, RINGMOD. Drop OSC targets and anything unknown.
        bool isCommonsACK = (echoTarget == TARGET_FILTER   ||
                             echoTarget == TARGET_ENVELOPE ||
                             echoTarget == TARGET_GLOBAL   ||
                             echoTarget == TARGET_LFO1     ||
                             echoTarget == TARGET_LFO2     ||
                             echoTarget == TARGET_RINGMOD);

        if (!isCommonsACK) return;

        uint16_t espIC3 = ((uint16_t)rxData[2] << 8) | rxData[3];
        ic3RemoteLEDs   = espIC3 & LED_IC3_REMOTE_MASK;

        retrigLFO1 = (rxData[4] & 0x01) != 0;
        retrigLFO2 = (rxData[4] & 0x02) != 0;
        IO_SetRetrigLEDs(retrigLFO1, retrigLFO2);
    }
}

// ----------------------------------------------------------------
void CAN_ProcessRangedChannels(void)
{
    // CH5 (LFO1 waveform), CH10 (LFO2 waveform), CH15 (Ring Mod pairs)
    static const uint8_t rangedChannels[] = { 5, 10, 15 };
    for (uint8_t i = 0; i < 3; i++)
        CAN_ProcessADC(rangedChannels[i], ADC_GetChannelValue(rangedChannels[i]));
}

// ----------------------------------------------------------------
void CAN_ProcessADC(uint8_t channel, uint16_t raw)
{
    if (channel >= 16) return;

    const ADC_Map_t *entry = &adcMap[channel];

    // Determine primary or secondary (CH11 only shifts on BTN_ENV_TRACK)
    bool useSecondary = false;
    if (entry->shiftBtn >= 0 && entry->shiftBtn < BTN_COUNT)
    {
        uint8_t sb = (uint8_t)entry->shiftBtn;
        useSecondary = btnToggleState[sb];
    }

    const ParamDef_t *def = useSecondary ? &entry->secondary : &entry->primary;
    if (def->canId == 0) return;

    // ---- ENCODE_RANGED: settle filter ----
    if (def->encoding == ENCODE_RANGED)
    {
        RangeRef_t ref = getRangeTable(channel);
        if (ref.table == NULL) return;

        int8_t window = -1;
        for (uint8_t w = 0; w < ref.count; w++)
        {
            if (raw >= ref.table[w].adcMin && raw <= ref.table[w].adcMax)
            {
                window = (int8_t)w;
                break;
            }
        }

        if (window < 0) return;   // in transition — discard

        uint32_t now = millis();
        if (window != switchLastWindow[channel])
        {
            switchLastWindow[channel]    = window;
            switchWindowEntryMs[channel] = now;
            return;
        }

        if (now - switchWindowEntryMs[channel] < SWITCH_SETTLE_MS) return;
        if (window == switchLastSentWindow[channel]) return;

        switchLastSentWindow[channel] = window;
        sendFrame(def->canId, def->target, def->param, ref.table[window].canValue);
        return;
    }

    // ---- Pot catch (CH11 Env/Track) ----
    if (catchMode[channel])
    {
        // Release condition 1 — hard cross: pot has crossed the target value.
        // No discontinuity; the physical and virtual positions meet exactly.
        bool crossed =
            (catchPrevRaw[channel] <= catchTarget[channel] && raw >= catchTarget[channel]) ||
            (catchPrevRaw[channel] >= catchTarget[channel] && raw <= catchTarget[channel]);

        // Release condition 2 — movement threshold: pot has moved far enough
        // from its position at button-press time that the user is clearly
        // intending to adjust. Release and start tracking from current position.
        // The discontinuity is bounded by CATCH_MOVE_THRESHOLD (~8% of range).
        int16_t moved = (int16_t)raw - (int16_t)catchOriginRaw[channel];
        if (moved < 0) moved = -moved;
        bool movedEnough = ((uint16_t)moved >= CATCH_MOVE_THRESHOLD);

        catchPrevRaw[channel] = raw;
        if (!crossed && !movedEnough) return;
        catchMode[channel] = false;
    }

    catchPrevRaw[channel] = raw;

    uint16_t value = encodeValue(raw, def);
    sendFrame(def->canId, def->target, def->param, value);

    if (useSecondary) lastSentSecondary[channel] = raw;
    else              lastSentPrimary[channel]    = raw;
}

// ----------------------------------------------------------------
void CAN_ProcessButton(Button_t btn)
{
    if (btn >= BTN_COUNT) return;

    const BTN_Map_t *entry = &btnMap[btn];

    switch (entry->mode)
    {
        case BTN_MODE_TOGGLE:
        {
            btnToggleState[btn] = !btnToggleState[btn];
            bool isOn = btnToggleState[btn];

            sendFrame(entry->canId, entry->target, entry->param,
                      isOn ? entry->valueOn : entry->valueOff);

            // Activate pot catch for any dual-function channels
            activateCatchForButton(btn, isOn);
            rebuildLocalLEDs();

            // Resend shifted channels on their new mapping
            for (uint8_t ch = 0; ch < 16; ch++)
                if (adcMap[ch].shiftBtn == (int8_t)btn)
                    CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
            break;
        }

        case BTN_MODE_LOCAL_TOGGLE:
        {
            // Same as TOGGLE but no CAN message sent — ATmega owns this
            // function entirely. Used for Env/Track (identical to Detune/
            // Phase switching on the Oscillator board). The button only
            // shifts the pot mapping and updates local indicator LEDs.
            btnToggleState[btn] = !btnToggleState[btn];
            bool isOn = btnToggleState[btn];

            activateCatchForButton(btn, isOn);
            rebuildLocalLEDs();

            for (uint8_t ch = 0; ch < 16; ch++)
                if (adcMap[ch].shiftBtn == (int8_t)btn)
                    CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
            break;
        }

        case BTN_MODE_MOMENTARY:
        {
            sendFrame(entry->canId, entry->target, entry->param, entry->valueOn);
            break;
        }

        case BTN_MODE_CYCLER:
        {
            // Advance state and wrap
            uint8_t next = (cyclerState[btn] + 1) % entry->cyclerStates;
            cyclerState[btn] = next;

            // Send the state index as the parameter value.
            // ESP32 interprets: 0=LP/Poly, 1=BP/Duo, 2=HP/Unison etc.
            sendFrame(entry->canId, entry->target, entry->param, (uint16_t)next);

            // No local LEDs — ESP32 confirms state and drives them via ACK
            break;
        }

        case BTN_MODE_MODIFIER:
            // Not used on Commons PCB — no-op
            break;
    }
}

// ----------------------------------------------------------------
uint16_t CAN_GetLEDs(void)
{
    return (ic3LocalLEDs & LED_IC3_LOCAL_MASK) | (ic3RemoteLEDs & LED_IC3_REMOTE_MASK);
}

bool CAN_GetToggleState(Button_t btn)
{
    if (btn >= BTN_COUNT) return false;
    return btnToggleState[btn];
}

uint8_t CAN_GetCyclerState(Button_t btn)
{
    if (btn >= BTN_COUNT) return 0;
    return cyclerState[btn];
}

// ----------------------------------------------------------------
void CAN_SendAllParameters(void)
{
    // ---- ADC channels ----
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        const ADC_Map_t  *entry = &adcMap[ch];
        const ParamDef_t *def   = &entry->primary;

        // ENCODE_RANGED channels (LFO waveform switches, Ring Mod pairs):
        // bypass settle filter and send directly at dump time.
        if (def->encoding == ENCODE_RANGED)
        {
            RangeRef_t ref = getRangeTable(ch);
            if (ref.table != NULL)
            {
                uint16_t raw = ADC_GetChannelValue(ch);
                for (uint8_t w = 0; w < ref.count; w++)
                {
                    if (raw >= ref.table[w].adcMin && raw <= ref.table[w].adcMax)
                    {
                        sendFrame(def->canId, def->target, def->param,
                                  ref.table[w].canValue);
                        switchLastWindow[ch]     = (int8_t)w;
                        switchLastSentWindow[ch] = (int8_t)w;
                        break;
                    }
                }
            }
            delayMicroseconds(200);
            continue;
        }

        CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
        delayMicroseconds(200);
    }

    // ---- Toggle button states (LOCAL_TOGGLE excluded — no CAN representation) ----
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];
        if (e->mode == BTN_MODE_TOGGLE)
        {
            sendFrame(e->canId, e->target, e->param,
                      btnToggleState[b] ? e->valueOn : e->valueOff);
            delayMicroseconds(200);
        }
    }

    // ---- Cycler states ----
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];
        if (e->mode == BTN_MODE_CYCLER)
        {
            sendFrame(e->canId, e->target, e->param, (uint16_t)cyclerState[b]);
            delayMicroseconds(200);
        }
    }
}
