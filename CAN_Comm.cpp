#include "CAN_Comm.h"
#include "param_map.h"
#include "Board.h"
#include "ADC.h"
#include "synth_params.h"

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// ================================================================
// CAN_Comm.cpp
//
// Modifications implemented here:
//
//  Mod 1 — Board ID target offsetting.
//           Board_GetOscOffset() is added to every OSC target before
//           transmission so board 0 sends TARGET_OSC1/2, board 1
//           sends TARGET_OSC3/4, etc. — transparent to param_map.h.
//
//  Mod 2 — BTN_OSC1_SYNC disabled on board 0.
//           btnEnabled[] array checked in CAN_ProcessButton().
//
//  Mod 3a — Detune zero band + LED flash.
//            ENCODE_DETUNE: piecewise linear mapping compensates for
//            log pot (centre=300). In-band → 32767 (0 cents).
//            CAN_LEDTask() drives solid/flash on the detune LED.
//
//  Mod 3b — Pot pickup / parameter catch.
//            When A6/A7 is toggled, catch mode activates for the
//            affected channel. CAN sends are suppressed until the
//            pot physically sweeps through the last sent position,
//            preventing parameter jumps on function switch.
//
//  Mod 4  — Resistor ladder ENCODE_RANGED.
//            Raw ADC matched to StepRange_t windows (midpoint-based).
//            40ms settle filter discards zero-drop transients.
// ================================================================

// ----------------------------------------------------------------
// CAN instance — exported so Board.cpp can send the announcement
// ----------------------------------------------------------------
MCP_CAN g_CAN(CAN_CS_PIN);

// ================================================================
// Resistor ladder range tables
// Values measured from hardware, windows computed from midpoints.
// ================================================================

// ---- Octave switch (3 positions) ----
// Measured: 0 / 340 / 683  (both OSCs, uniform ladder)
// Midpoints: 170, 511
// canValue: 0=down(-1oct)  1=centre(0)  2=up(+1oct)
static const StepRange_t octaveRanges[3] =
{
    {   0, 170, 0 },   // -1 octave
    { 171, 511, 1 },   //  0
    { 512, 1023, 2 },  // +1 octave
};

// ---- Waveform switch (6 positions) ----
// Measured: 0 / 169 / 340 / 511 / 682 / 853  (both OSCs, uniform ladder)
// Midpoints: 84, 254, 425, 596, 767
// Physical order low→high: Sine / Triangle / Noise / Chaos / Pulse / Saw
// canValue per synth_params.h: SAW=0 SINE=1 TRI=2 PULSE=3 CHAOS=4 NOISE=5
static const StepRange_t waveformRanges[6] =
{
    {   0,  84, 1 },   // Sine
    {  85, 254, 2 },   // Triangle
    { 255, 425, 5 },   // Noise
    { 426, 596, 4 },   // Chaos
    { 597, 767, 3 },   // Pulse
    { 768, 1023, 0 },  // Saw
};

// ---- Range table lookup by channel index ----
typedef struct { const StepRange_t *table; uint8_t count; } RangeRef_t;

static RangeRef_t getRangeTable(uint8_t ch)
{
    switch (ch)
    {
        case 5: case 8: return { waveformRanges, 6 };
        case 7: case 9: return { octaveRanges,   3 };
        default:        return { NULL, 0 };
    }
}

// ================================================================
// State
// ================================================================

// ---- Button state ----
static bool    btnToggleState[BTN_COUNT]   = { false };
static uint8_t modifierState[BTN_COUNT]    = { 0 };
static bool    modifierConfirmed[BTN_COUNT]= { false }; // true once ESP32 ACK lights green LED
static bool    btnEnabled[BTN_COUNT];                   // Mod 2: false = silenced

// ---- Modifier hold state ----
// Hold timer per modifier button. Start is set on press while ON; 0 = not tracking.
// holdFired prevents the release handler acting after a hold has already fired.
// justEnabled suppresses the release handler for the initial enable press (0→1).
static uint32_t modifierHoldStart[BTN_COUNT]   = { 0 };
static bool     modifierHoldFired[BTN_COUNT]   = { false };
static bool     modifierJustEnabled[BTN_COUNT] = { false };

// ---- Extra local shift buttons (A6=0, A7=1) ----
static bool extraBtnState[2] = { false, false };

// ---- LED state ----
static uint16_t ic3LocalLEDs  = 0;
static uint16_t ic3RemoteLEDs = 0;
static uint8_t  ic4RemoteLEDs = 0;

// ---- Detune LED flash state (Mod 3a) ----
// Index 0 = OSC1 (CH6), index 1 = OSC2 (CH10)
// flashPeriodMs == 0 means solid ON; > 0 means flash at that period.
static uint32_t detuneFlashPeriodMs[2]    = { 0, 0 };
static uint32_t detuneFlashLastToggleMs[2]= { 0, 0 };
static bool     detuneFlashLEDOn[2]       = { true, true };

// ---- Pot catch state (Mod 3b) ----
// Tracks last raw value sent for each mode (primary / secondary) per channel.
// Catch mode suppresses sends until pot crosses the catch target.
static uint16_t lastSentPrimary[16];    // 0xFFFF = never sent
static uint16_t lastSentSecondary[16];  // 0xFFFF = never sent
static bool     catchMode[16];
static uint16_t catchTarget[16];
static uint16_t catchPrevRaw[16];
static uint16_t catchOriginRaw[16];  // pot position at moment catch activated — never updated

// ---- Switch settle state (Mod 4) ----
// Per channel: which range window we're currently in, and when we entered it.
// -1 means no window matched (in transition).
static int8_t   switchLastWindow[16];
static uint32_t switchWindowEntryMs[16];
static int8_t   switchLastSentWindow[16];

// ---- CAN sequence counter ----
static uint8_t seqNum = 0;

// ================================================================
// Internal helpers
// ================================================================

// Simple linear detune encoding with zero-band clamp.
// Pot is linear — no log compensation needed.
// In-band → 32767 exactly (= 0 cents on ESP32).
// Outside band → straight 0–65535 linear mapping.
// The slight asymmetry from hardware centre not being exactly 512
// is imperceptible in practice and corrected by the zero band.
static uint16_t encodeDetune(uint16_t raw)
{
    if (raw >= DETUNE_ZERO_BAND_LOW && raw <= DETUNE_ZERO_BAND_HIGH)
        return 32767;

    return (uint16_t)((uint32_t)raw * 65535UL / 1023UL);
}

// Flash period in ms proportional to distance from zero band edge.
// Returns 0 if inside the band (solid on).
static uint32_t calcDetuneFlashPeriod(uint16_t raw)
{
    if (raw >= DETUNE_ZERO_BAND_LOW && raw <= DETUNE_ZERO_BAND_HIGH)
        return 0;   // solid

    uint32_t dist, maxDist;

    if (raw < DETUNE_ZERO_BAND_LOW)
    {
        dist    = DETUNE_ZERO_BAND_LOW - raw;
        maxDist = DETUNE_ZERO_BAND_LOW;                  // 275
    }
    else
    {
        dist    = raw - DETUNE_ZERO_BAND_HIGH;
        maxDist = 1023UL - DETUNE_ZERO_BAND_HIGH;        // 698
    }

    // dist/maxDist → 0.0-1.0 mapped to SLOW..FAST
    uint32_t range = DETUNE_FLASH_SLOW_MS - DETUNE_FLASH_FAST_MS;
    return DETUNE_FLASH_SLOW_MS - (dist * range) / maxDist;
}

// Standard encode for non-detune channels.
static uint16_t encodeValue(uint16_t raw, const ParamDef_t *def)
{
    switch (def->encoding)
    {
        case ENCODE_LINEAR:
        case ENCODE_EXP:
            return (uint16_t)((uint32_t)raw * 65535UL / 1023UL);

        case ENCODE_DETUNE:
            return encodeDetune(raw);

        default:
            return 0;   // ENCODE_RANGED handled separately in CAN_ProcessADC
    }
}

// Build and transmit a 5-byte CAN frame.
// Applies board OSC target offset (Mod 1) transparently.
static void sendFrame(uint8_t canId, uint8_t target, uint8_t param, uint16_t value)
{
    // Mod 1: offset OSC targets by board address so the ESP32 receives
    // TARGET_OSC3/4 from board 1, TARGET_OSC5/6 from board 2, etc.
    // Non-OSC targets (FILTER, ENVELOPE, GLOBAL) pass through unchanged.
    uint8_t adjustedTarget = target;
    if (target >= TARGET_OSC1 && target <= TARGET_OSC4)
        adjustedTarget = target + Board_GetOscOffset();

    uint8_t data[5];
    data[0] = adjustedTarget;
    data[1] = param;
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)(value & 0xFF);
    data[4] = seqNum++;

    g_CAN.sendMsgBuf(canId, 0, 5, data);
}

// Rebuild ic3LocalLEDs from button states.
// Detune/Phase LEDs are excluded — CAN_LEDTask() owns those bits.
static void rebuildLocalLEDs(void)
{
    ic3LocalLEDs = 0;

    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];

        if (e->mode == BTN_MODE_MODIFIER)
        {
            uint8_t s = modifierState[b];
            // state 1 LED lights immediately — it shows which parameter the knob is aimed at,
            // not whether the ESP32 has confirmed the modifier. Confirmation only gates the
            // green remote LED (driven by ESP32 ACK) and the knob sends (CAN_ProcessADC).
            if      (s == 1 && e->ledBit >= 0)          ic3LocalLEDs |= (1u << e->ledBit);
            else if (s == 2 && e->ledBitSecondary >= 0)  ic3LocalLEDs |= (1u << e->ledBitSecondary);
        }
        else if (e->mode == BTN_MODE_TOGGLE && btnToggleState[b] && e->ledBit >= 0)
        {
            ic3LocalLEDs |= (1u << e->ledBit);
        }
    }

    // Phase LEDs: lit when in phase mode (simple, no flash)
    if (extraBtnState[0]) ic3LocalLEDs |= (1u << LED_BIT_OSC1_PHASE);
    if (extraBtnState[1]) ic3LocalLEDs |= (1u << LED_BIT_OSC2_PHASE);

    // Note: Detune LEDs NOT set here — CAN_LEDTask() drives them
    //       to support both solid and flashing states.
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

    // Initialise all per-channel state
    for (uint8_t i = 0; i < 16; i++)
    {
        lastSentPrimary[i]    = 0xFFFF;
        lastSentSecondary[i]  = 0xFFFF;
        catchMode[i]          = false;
        catchTarget[i]        = 0;
        catchPrevRaw[i]       = 0;
        catchOriginRaw[i]     = 0;
        switchLastWindow[i]   = -1;
        switchWindowEntryMs[i]= 0;
        switchLastSentWindow[i]= -1;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        detuneFlashPeriodMs[i]    = 0;
        detuneFlashLastToggleMs[i]= 0;
        detuneFlashLEDOn[i]       = true;
    }

    ic3RemoteLEDs = 0;
    ic4RemoteLEDs = 0;
    seqNum        = 0;

    // Mod 2 (revised): on board 0, BTN_OSC1_SYNC is NOT disabled.
    // Instead it sends PARAM_GLOBAL_SYNC_MASTER with value=0 (OSC1 index),
    // telling the ESP32 to make OSC1 the sync master for all other oscillators.
    // This is handled in CAN_ProcessButton() by checking Board_GetID()==0
    // and btn==BTN_OSC1_SYNC — no btnEnabled flag needed.
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        btnEnabled[b]          = true;
        modifierConfirmed[b]   = false;
        modifierHoldStart[b]   = 0;
        modifierHoldFired[b]   = false;
        modifierJustEnabled[b] = false;
    }

    // Initial LED state: both OSCs start in Detune mode (solid LEDs)
    rebuildLocalLEDs();
    detuneFlashLEDOn[0] = true;
    detuneFlashLEDOn[1] = true;
    ic3LocalLEDs |= (1u << LED_BIT_OSC1_DETUNE);
    ic3LocalLEDs |= (1u << LED_BIT_OSC2_DETUNE);

     // Announce presence — catches the case where we boot before the ESP32
    uint8_t announceData[4];
    announceData[0] = TARGET_SYSTEM;
    announceData[1] = PARAM_SYSTEM_BOARD_COMPLETE;
    announceData[2] = Board_GetID();
    announceData[3] = BOARD_TYPE_OSC;   // BOARD_TYPE_OSC on oscillator boards
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

        uint8_t announceData[4];
        announceData[0] = TARGET_SYSTEM;
        announceData[1] = PARAM_SYSTEM_BOARD_COMPLETE;
        announceData[2] = Board_GetID();
        announceData[3] = BOARD_TYPE_OSC;
        g_CAN.sendMsgBuf(CAN_ID_SYSTEM, 0, 4, announceData);

        // Resend all parameters since ESP32 may have just reset
        CAN_SendAllParameters();
        return;
    }



    // ACK frame from ESP32:
    //   Byte 0: Target (echo)
    //   Byte 1: Param  (echo)
    //   Byte 2: IC3 remote LED high byte (bits 8-15, LED_IC3_REMOTE_MASK)
    //   Byte 3: IC3 remote LED low byte  (bits 0-7,  LED_IC3_REMOTE_MASK)
    //   Byte 4: IC4 LED byte (bits 0-5 = IC4_LED_BIT_xxx)
    if (rxId == CAN_ID_ACK && rxLen >= 5)
    {
        uint8_t echoTarget = rxData[0];
        uint8_t echoParam  = rxData[1];

        // Only process OSC ACKs for our own two oscillators
        uint8_t myOscA = TARGET_OSC1 + Board_GetOscOffset();
        uint8_t myOscB = TARGET_OSC1 + Board_GetOscOffset() + 1;
        bool isOscAck = (echoTarget == myOscA || echoTarget == myOscB);

        // GLOBAL SYNC_MASTER / SYNC_ALL / PANIC originate from the oscillator board
        bool isOscGlobal = (echoTarget == TARGET_GLOBAL &&
                    (echoParam == PARAM_GLOBAL_SYNC_MASTER ||
                     echoParam == PARAM_GLOBAL_SYNC_ALL    ||
                     echoParam == PARAM_GLOBAL_PANIC));

        if (!isOscAck && !isOscGlobal) return;

        if (isOscAck)
        {
            // OSC ACKs carry our own IC3 remote LED state in bytes 2-3 — safe to apply.
            uint16_t espIC3     = ((uint16_t)rxData[2] << 8) | rxData[3];
            uint16_t prevRemote = ic3RemoteLEDs;
            ic3RemoteLEDs       = espIC3 & LED_IC3_REMOTE_MASK;
            ic4RemoteLEDs       = rxData[4] & 0x3F;

            // Detect rising edges on modifier enable bits.
            // A 0→1 transition means the ESP32 has confirmed the modifier is active.
            static const struct { uint8_t btn; uint8_t bit; } confirmMap[4] =
            {
                { BTN_OSC1_FOLD, LED_BIT_OSC1_FOLD_ENABLE },
                { BTN_OSC1_SH,   LED_BIT_OSC1_SH_ENABLE   },
                { BTN_OSC2_SH,   LED_BIT_OSC2_SH_ENABLE   },
                { BTN_OSC2_FOLD, LED_BIT_OSC2_FOLD_ENABLE  },
            };
            for (uint8_t i = 0; i < 4; i++)
            {
                uint16_t mask   = (1u << confirmMap[i].bit);
                bool     wasSet = (prevRemote    & mask) != 0;
                bool     isSet  = (ic3RemoteLEDs & mask) != 0;

                if (!wasSet && isSet)
                {
                    uint8_t b = confirmMap[i].btn;
                    if (modifierState[b] >= 1 && !modifierConfirmed[b])
                    {
                        modifierConfirmed[b] = true;
                        rebuildLocalLEDs();
                    }
                }
            }
        }
        else  // isOscGlobal
        {
            // GLOBAL ACKs carry commons IC3 state in bytes 2-3 which overlaps
            // the OSC remote LED mask — do NOT update IC3 from these.
            // Only update IC4 for sync/solo/clone LED confirmation.
            ic4RemoteLEDs = rxData[4] & 0x3F;
        }
    }
    // ---- Parameter dump request from ESP32 ----
    // Frame layout:
    //   Byte 0: Board ID this request is addressed to (0-3)
    //   Bytes 1-4: unused (0x00)
    // Only respond if the request is for our board ID.
    if (rxId == CAN_ID_REQUEST && rxLen >= 1)
    {
        if (rxData[0] == Board_GetID())
            CAN_SendAllParameters();
    }
}

// ----------------------------------------------------------------
// CAN_LEDTask — drives detune LED solid/flash behaviour.
// Call every loop(). Non-blocking (uses millis()).
// ----------------------------------------------------------------
void CAN_LEDTask(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t ledBit = (i == 0) ? LED_BIT_OSC1_DETUNE : LED_BIT_OSC2_DETUNE;

        if (extraBtnState[i])
        {
            // In Phase mode — Detune LED off (Phase LED handled by rebuildLocalLEDs)
            ic3LocalLEDs &= ~(1u << ledBit);
            continue;
        }

        // In Detune mode
        uint32_t period = detuneFlashPeriodMs[i];

        if (period == 0)
        {
            // Solid ON
            ic3LocalLEDs |= (1u << ledBit);
        }
        else
        {
            // Flashing — toggle at half-period intervals
            uint32_t now = millis();
            if (now - detuneFlashLastToggleMs[i] >= (period / 2))
            {
                detuneFlashLastToggleMs[i] = now;
                detuneFlashLEDOn[i]        = !detuneFlashLEDOn[i];
            }

            if (detuneFlashLEDOn[i])
                ic3LocalLEDs |=  (1u << ledBit);
            else
                ic3LocalLEDs &= ~(1u << ledBit);
        }
    }
}

// ----------------------------------------------------------------
void CAN_ProcessADC(uint8_t channel, uint16_t raw)
{
    if (channel >= 16) return;

    const ADC_Map_t *entry = &adcMap[channel];

    // Determine primary or secondary mapping
    bool useSecondary = false;

    if (entry->shiftBtn == (int8_t)EXTRA_BTN_A6)
        useSecondary = extraBtnState[0];
    else if (entry->shiftBtn == (int8_t)EXTRA_BTN_A7)
        useSecondary = extraBtnState[1];
    else if (entry->shiftBtn >= 0 && entry->shiftBtn < BTN_COUNT)
    {
        uint8_t sb = (uint8_t)entry->shiftBtn;
        if (btnMap[sb].mode == BTN_MODE_MODIFIER)
        {
            uint8_t ms = modifierState[sb];
            // Knob is silent until modifier is enabled and ESP32 has confirmed it
            if (ms == 0)                        return;
            if (!modifierConfirmed[sb])         return;
            useSecondary = (ms == 2);
        }
        else
        {
            useSecondary = btnToggleState[sb];
        }
    }

    const ParamDef_t *def = useSecondary ? &entry->secondary : &entry->primary;
    if (def->canId == 0) return;

    // ---- Mod 4: ENCODE_RANGED — settle filter ----
    if (def->encoding == ENCODE_RANGED)
    {
        RangeRef_t ref = getRangeTable(channel);
        if (ref.table == NULL) return;

        // Find which window raw falls in
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

        // New window entered — reset settle timer
        if (window != switchLastWindow[channel])
        {
            switchLastWindow[channel]    = window;
            switchWindowEntryMs[channel] = now;
            return;
        }

        // Not yet settled
        if (now - switchWindowEntryMs[channel] < SWITCH_SETTLE_MS) return;

        // Settled — only send if window changed from last sent
        if (window == switchLastSentWindow[channel]) return;

        switchLastSentWindow[channel] = window;
        sendFrame(def->canId, def->target, def->param, ref.table[window].canValue);
        return;
    }

    // ---- Mod 3a: ENCODE_DETUNE — update flash state ----
    if (def->encoding == ENCODE_DETUNE)
    {
        uint8_t idx = (channel == 6) ? 0 : 1;
        detuneFlashPeriodMs[idx] = calcDetuneFlashPeriod(raw);
    }

    // ---- Mod 3b: catch mode — suppress send until pot catches up ----
    if (catchMode[channel])
    {
        bool crossed =
            (catchPrevRaw[channel] <= catchTarget[channel] && raw >= catchTarget[channel]) ||
            (catchPrevRaw[channel] >= catchTarget[channel] && raw <= catchTarget[channel]);

        int16_t moved = (int16_t)raw - (int16_t)catchOriginRaw[channel];
        if (moved < 0) moved = -moved;
        bool movedEnough = ((uint16_t)moved >= CATCH_MOVE_THRESHOLD);

        catchPrevRaw[channel] = raw;

        if (!crossed && !movedEnough) return;   // still catching — don't send
        catchMode[channel] = false;             // caught — resume normal operation
    }

    catchPrevRaw[channel] = raw;

    // ---- Standard encode and send ----
    uint16_t value = encodeValue(raw, def);
    sendFrame(def->canId, def->target, def->param, value);

    // Track last sent raw per mode for catch mode on next function switch
    if (useSecondary) lastSentSecondary[channel] = raw;
    else              lastSentPrimary[channel]    = raw;
}

// Activate pot catch for all ADC channels that shift on the given button.
// goingToSecondary: true  = switching primary→secondary (1→2)
//                   false = switching secondary→primary (2→0 or toggle OFF)
// The pot must sweep through the last-sent position in the newly-active
// mode before any sends resume, preventing parameter jumps.
static void activateCatchForButton(Button_t btn, bool goingToSecondary)
{
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        if (adcMap[ch].shiftBtn != (int8_t)btn) continue;

        uint16_t target = goingToSecondary
                          ? lastSentSecondary[ch]
                          : lastSentPrimary[ch];

        if (target == 0xFFFF) continue;   // never sent in this mode — no catch needed

        uint16_t currentRaw = ADC_GetChannelValue(ch);
        int16_t  dist       = (int16_t)currentRaw - (int16_t)target;
        if (dist < 0) dist = -dist;
        if ((uint16_t)dist <= CATCH_WINDOW) continue;  // already close — skip catch

        catchMode[ch]      = true;
        catchTarget[ch]    = target;
        catchPrevRaw[ch]   = currentRaw;
        catchOriginRaw[ch] = currentRaw;  // snapshot — never updated during catch
    }
}

// ----------------------------------------------------------------
void CAN_ProcessButton(Button_t btn)
{
    if (btn >= BTN_COUNT) return;

    // Mod 2: silently ignore disabled buttons
    if (!btnEnabled[btn]) return;

    const BTN_Map_t *entry = &btnMap[btn];

    switch (entry->mode)
    {
        case BTN_MODE_TOGGLE:
        {
            btnToggleState[btn] = !btnToggleState[btn];
            bool isOn = btnToggleState[btn];
            sendFrame(entry->canId, entry->target, entry->param,
                      isOn ? entry->valueOn : entry->valueOff);
            // Activate catch for any channels that shift on this button
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

        case BTN_MODE_MODIFIER:
        {
            if (modifierState[btn] == 0)
            {
                // OFF → ON (press): send enable immediately.
                // Local LED stays off until the ESP32 ACK confirms.
                // Hold timer is NOT started here — hold from state 0 is a no-op.
                modifierState[btn]      = 1;
                modifierConfirmed[btn]  = false;
                modifierJustEnabled[btn]= true;   // suppress release handler for this press
                sendFrame(entry->canId, entry->target, entry->param, entry->valueOn);
                rebuildLocalLEDs();
                for (uint8_t ch = 0; ch < 16; ch++)
                    if (adcMap[ch].shiftBtn == (int8_t)btn)
                        CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
            }
            else
            {
                // ON (state 1 or 2): start hold timer on press.
                // The toggle action (1↔2) is deferred to CAN_ProcessButtonRelease.
                // CAN_ModifierHoldTask() will fire the disable if held >= MODIFIER_HOLD_MS.
                modifierHoldStart[btn] = millis();
                modifierHoldFired[btn] = false;
            }
            break;
        }
    }
}

// ----------------------------------------------------------------
// CAN_ProcessButtonRelease
// Called on button-up for modifier buttons. Handles the short-press
// toggle between primary (state 1) and secondary (state 2) while the
// modifier is ON. Does nothing if the hold action already fired.
// ----------------------------------------------------------------
void CAN_ProcessButtonRelease(Button_t btn)
{
    if (btn >= BTN_COUNT) return;
    if (btnMap[btn].mode != BTN_MODE_MODIFIER) return;
    if (modifierState[btn] == 0) return;        // wasn't ON — no action
    if (modifierHoldFired[btn]) return;         // hold already handled this press
    if (modifierJustEnabled[btn])               // release of the initial enable press
    {
        modifierJustEnabled[btn] = false;       // clear flag — next press is a real toggle
        return;
    }

    // Short press while ON: toggle primary ↔ secondary
    uint8_t newState = (modifierState[btn] == 1) ? 2 : 1;
    modifierState[btn] = newState;

    if (newState == 2)
        activateCatchForButton(btn, true);      // primary → secondary
    else
        activateCatchForButton(btn, false);     // secondary → primary

    rebuildLocalLEDs();
    for (uint8_t ch = 0; ch < 16; ch++)
        if (adcMap[ch].shiftBtn == (int8_t)btn)
            CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
}

// ----------------------------------------------------------------
// CAN_ModifierHoldTask
// Call every loop(). Non-blocking. Fires the modifier disable action
// when a modifier button has been held for >= MODIFIER_HOLD_MS while
// the modifier is ON. Sets modifierHoldFired to prevent the release
// handler from also toggling.
// ----------------------------------------------------------------
void CAN_ModifierHoldTask(void)
{
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        if (btnMap[b].mode != BTN_MODE_MODIFIER)    continue;
        if (modifierState[b] == 0)                  continue;
        if (modifierHoldFired[b])                   continue;
        if (modifierHoldStart[b] == 0)              continue;
        if (!IO_IsButtonHeld((Button_t)b))          continue;
        if (millis() - modifierHoldStart[b] < MODIFIER_HOLD_MS) continue;

        // Hold threshold reached — disable modifier
        modifierHoldFired[b]    = true;
        modifierState[b]        = 0;
        modifierConfirmed[b]    = false;
        modifierJustEnabled[b]  = false;
        sendFrame(btnMap[b].canId, btnMap[b].target, btnMap[b].param, btnMap[b].valueOff);
        activateCatchForButton((Button_t)b, false);
        rebuildLocalLEDs();
        for (uint8_t ch = 0; ch < 16; ch++)
            if (adcMap[ch].shiftBtn == (int8_t)b)
                CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
    }
}

// ----------------------------------------------------------------
void CAN_ProcessExtraButton(uint8_t which)
{
    if (which > 1) return;

    extraBtnState[which] = !extraBtnState[which];
    bool goingToSecondary = extraBtnState[which];

    rebuildLocalLEDs();

    // Mod 3b: activate catch mode for the affected channel.
    // catchTarget = last sent raw for the newly-active mode.
    // The pot must physically sweep through that position before
    // any sends resume, preventing a parameter jump.
    uint8_t ch = (which == 0) ? 6 : 10;

    uint16_t target = goingToSecondary
                      ? lastSentSecondary[ch]
                      : lastSentPrimary[ch];

    if (target != 0xFFFF)
    {
        uint16_t currentRaw = ADC_GetChannelValue(ch);
        int16_t  dist       = (int16_t)currentRaw - (int16_t)target;
        if (dist < 0) dist = -dist;
        if ((uint16_t)dist > CATCH_WINDOW)
        {
            catchMode[ch]      = true;
            catchTarget[ch]    = target;
            catchPrevRaw[ch]   = currentRaw;
            catchOriginRaw[ch] = currentRaw;
        }
        // else: already close enough — skip catch, send immediately
    }
    // If 0xFFFF (never sent in this mode), no catch needed —
    // first send in new mode will establish the baseline naturally.
}

// ----------------------------------------------------------------
// CAN_ProcessRangedChannels
// Must be called every loop() unconditionally so the 40ms settle
// timer can elapse. ADC change detection is one-shot and would
// prevent the timer completing if used as a gate.
// ----------------------------------------------------------------
void CAN_ProcessRangedChannels(void)
{
    static const uint8_t rangedChannels[] = { 5, 7, 8, 9 };
    for (uint8_t i = 0; i < 4; i++)
        CAN_ProcessADC(rangedChannels[i], ADC_GetChannelValue(rangedChannels[i]));
}

// ----------------------------------------------------------------
// CAN_SendAllParameters
// Sends the current value of every control over CAN.
// Called on receipt of CAN_ID_REQUEST from the ESP32.
// A small inter-frame gap is inserted so the bus isn't flooded.
//
// Frame ordering: ADC channels first (0-15), then modifier/toggle
// button states, then extra button (A6/A7) states.
// ----------------------------------------------------------------
void CAN_SendAllParameters(void)
{
    // ---- ADC channels ----
    for (uint8_t ch = 0; ch < 16; ch++)
    {
        const ADC_Map_t  *entry = &adcMap[ch];
        const ParamDef_t *def   = &entry->primary;

        // ENCODE_RANGED channels (waveform + octave switches): bypass the
        // settle filter and send directly. The switch is physically stable
        // at dump time — settle logic is only needed for live transitions.
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
                        // Seed settle state so live transitions work immediately
                        switchLastWindow[ch]     = (int8_t)w;
                        switchLastSentWindow[ch] = (int8_t)w;
                        break;
                    }
                }
            }
            delayMicroseconds(200);
            continue;  // skip CAN_ProcessADC for this channel
        }

        CAN_ProcessADC(ch, ADC_GetChannelValue(ch));
        delayMicroseconds(200);
    }

    // ---- Modifier button states ----
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];
        if (e->mode != BTN_MODE_MODIFIER) continue;

        uint8_t s = modifierState[b];
        uint16_t val = (s > 0) ? e->valueOn : e->valueOff;
        sendFrame(e->canId, e->target, e->param, val);
        delayMicroseconds(200);
    }

    // ---- Toggle button states ----
    for (uint8_t b = 0; b < BTN_COUNT; b++)
    {
        const BTN_Map_t *e = &btnMap[b];
        if (e->mode != BTN_MODE_TOGGLE) continue;

        uint16_t val = btnToggleState[b] ? e->valueOn : e->valueOff;
        sendFrame(e->canId, e->target, e->param, val);
        delayMicroseconds(200);
    }
}

// ----------------------------------------------------------------
uint16_t CAN_GetLEDs(void)
{
    return (ic3LocalLEDs & LED_IC3_LOCAL_MASK) | (ic3RemoteLEDs & LED_IC3_REMOTE_MASK);
}

uint8_t CAN_GetIC4LEDs(void)    { return ic4RemoteLEDs; }

bool CAN_GetToggleState(Button_t btn)
{
    if (btn >= BTN_COUNT) return false;
    return btnToggleState[btn];
}

uint8_t CAN_GetModifierState(Button_t btn)
{
    if (btn >= BTN_COUNT) return 0;
    return modifierState[btn];
}

bool CAN_GetExtraState(uint8_t which)
{
    if (which > 1) return false;
    return extraBtnState[which];
}
