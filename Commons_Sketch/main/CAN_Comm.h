#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "IO.h"

// ================================================================
// CAN_Comm.h — Commons PCB
// ================================================================

#define CAN_CS_PIN   10
#define CAN_INT_PIN   2
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ   // change to MCP_16MHZ if needed

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

bool     CAN_Init(void);

// Call every loop() — handles incoming frames (ACKs, requests)
void     CAN_Task(void);

// Call every loop() — polls ENCODE_RANGED channels (CH5, CH10, CH15)
void     CAN_ProcessRangedChannels(void);

// No LED flash task needed on Commons (no detune LED)

void     CAN_ProcessADC(uint8_t channel, uint16_t raw);
void     CAN_ProcessButton(Button_t btn);

// Returns combined IC3 LED word. Pass to IO_SetLEDs().
uint16_t CAN_GetLEDs(void);

// State accessors for debug output
bool     CAN_GetToggleState(Button_t btn);
uint8_t  CAN_GetCyclerState(Button_t btn);

// Broadcast all current parameter values (response to CAN_ID_REQUEST)
void     CAN_SendAllParameters(void);

#endif
