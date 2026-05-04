#pragma once
// ==============================================================
// synth_can.h  —  Version 11.0
// TWAI (CAN) setup, system startup handshake, message processing, ACK.
// Included by: synth_can.cpp, synth_v10_1.ino
// ==============================================================

#include "synth_engine.h"

// ==========================
// Function Declarations
// ==========================
void setupTWAI();
void systemStartup();
void processCanMessages();
void sendCanAck(uint8_t target, uint8_t param, uint8_t seq);
void CAN_RequestDump(uint8_t boardId);