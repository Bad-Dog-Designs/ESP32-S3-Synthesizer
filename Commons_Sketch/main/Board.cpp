#include "Board.h"
#include "synth_params.h"
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// ================================================================
// Board.cpp — Commons PCB
// Fixed board ID — no hardware pins read.
// ================================================================

extern MCP_CAN g_CAN;

void Board_Init(void)
{
    // Nothing to read — Commons board ID is fixed in firmware.
    // D8/D9 are left as default inputs (no pullup needed).
}

uint8_t Board_GetID(void)        { return BOARD_ID_COMMONS; }
uint8_t Board_GetOscOffset(void) { return 0; }  // Commons never offsets OSC targets

void Board_SendAnnouncement(void)
{
    // Byte layout matches oscillator board announcement:
    //   Byte 0: Board ID (4)
    //   Byte 1: OSC offset (0 — not applicable for Commons)
    //   Byte 2: 0x01 = Commons board type flag (distinguishes from OSC boards)
    //   Byte 3: 0x00 reserved
    //   Byte 4: 0x00 sequence
    uint8_t data[5] = { BOARD_ID_COMMONS, 0x00, 0x01, 0x00, 0x00 };
    g_CAN.sendMsgBuf(CAN_ID_ANNOUNCE, 0, 5, data);
}
