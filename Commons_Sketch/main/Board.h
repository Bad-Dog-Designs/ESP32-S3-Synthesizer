#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

// ================================================================
// Board.h — Commons PCB version
//
// The Commons PCB has a fixed board ID set in firmware rather than
// read from hardware pins. ID 4 places it after the four possible
// oscillator boards (IDs 0–3) in the ESP32 startup sequence.
//
// The Commons board does not use OSC target offsetting — it sends
// to TARGET_FILTER, TARGET_ENVELOPE, TARGET_GLOBAL, TARGET_LFO1/2,
// and TARGET_RINGMOD, none of which are offset by board ID.
//
// Board_GetOscOffset() always returns 0 for the Commons board —
// the sendFrame() OSC-offset logic in CAN_Comm.cpp only applies
// to TARGET_OSC1–OSC8, so non-OSC targets pass through unchanged
// regardless.
// ================================================================

#define BOARD_ID_COMMONS   4   // Fixed — must not conflict with OSC board IDs 0-3

// D8/D9 not used for ID on Commons PCB (only one Commons board exists)
// Pins are left as inputs (default) and not read.

void    Board_Init(void);
uint8_t Board_GetID(void);
uint8_t Board_GetOscOffset(void);  // always 0 on Commons
void    Board_SendAnnouncement(void);

#endif
