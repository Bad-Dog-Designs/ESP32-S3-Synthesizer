#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

// ================================================================
// Board.h
// Reads the 2-bit hardware address from digital pins D8/D9.
// The pins are tied to 0V or +5V via zero-ohm links at assembly time.
//
// D9  D8  Board ID  OSCs
//  0   0      0     1 & 2
//  0   1      1     3 & 4
//  1   0      2     5 & 6
//  1   1      3     7 & 8
//
// Board_GetOscOffset() returns 0, 2, 4, or 6.
// This is added to TARGET_OSC1/OSC2 in every outgoing CAN frame
// so the ESP32 receives the correct oscillator targets without
// the ATmega needing board-specific mapping tables.
//
// Board_SendAnnouncement() transmits a CAN_ID_ANNOUNCE frame so
// the ESP32 knows which boards are present and can stagger the
// initial parameter dump to prevent all boards transmitting at once.
// ================================================================

#define BOARD_SELECT_0_PIN  8   // D8 — LSB of board address
#define BOARD_SELECT_1_PIN  9   // D9 — MSB of board address

void    Board_Init(void);

// Returns the board ID (0-3) read from D8/D9
uint8_t Board_GetID(void);

// Returns the oscillator CAN target offset (0, 2, 4, or 6)
uint8_t Board_GetOscOffset(void);

// Transmit a CAN_ID_ANNOUNCE frame.
// Frame layout (5 bytes):
//   Byte 0 : Board ID   (0-3)
//   Byte 1 : OSC offset (0,2,4,6)
//   Byte 2 : 0x00 (reserved)
//   Byte 3 : 0x00 (reserved)
//   Byte 4 : 0x00 (sequence)
// Call once in setup() after CAN_Init().
void    Board_SendAnnouncement(void);

#endif
