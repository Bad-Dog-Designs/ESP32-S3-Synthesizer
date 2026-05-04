#include "Board.h"
#include "synth_params.h"
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// ================================================================
// Board.cpp
// ================================================================

// CAN instance is owned by CAN_Comm.cpp.
// Board_SendAnnouncement() uses the same MCP_CAN object via an
// extern reference declared here.
extern MCP_CAN g_CAN;

static uint8_t boardID     = 0;
static uint8_t oscOffset   = 0;

void Board_Init(void)
{
    // Use INPUT_PULLUP — external zero-ohm link to 0V pulls LOW,
    // absent link leaves pin HIGH via internal pullup.
    // Either way the external connection always wins.
    pinMode(BOARD_SELECT_0_PIN, INPUT_PULLUP);
    pinMode(BOARD_SELECT_1_PIN, INPUT_PULLUP);

    // Read once and cache — these are hardware-wired and never change.
    uint8_t bit0 = digitalRead(BOARD_SELECT_0_PIN) ? 0 : 1;  // active LOW
    uint8_t bit1 = digitalRead(BOARD_SELECT_1_PIN) ? 0 : 1;

    boardID   = (bit1 << 1) | bit0;
    oscOffset = boardID * 2;
}

uint8_t Board_GetID(void)        { return boardID;   }
uint8_t Board_GetOscOffset(void) { return oscOffset; }

void Board_SendAnnouncement(void)
{
    uint8_t data[5] = { boardID, oscOffset, 0x00, 0x00, 0x00 };
    g_CAN.sendMsgBuf(CAN_ID_ANNOUNCE, 0, 5, data);
}
