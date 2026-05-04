#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "IO.h"

// ================================================================
// CAN_Comm.h
// ================================================================

#define CAN_CS_PIN   10
#define CAN_INT_PIN   2
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ   // change to MCP_16MHZ if needed

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------

bool     CAN_Init(void);

// Call every loop() — handles incoming CAN frames (remote LEDs, ACKs, requests)
void     CAN_Task(void);

// Call every loop() — drives detune LED solid/flash behaviour
void     CAN_LEDTask(void);

// Call every loop() — runs settle timer for ENCODE_RANGED channels.
// Must be called unconditionally every loop (not gated on ADC change).
void     CAN_ProcessRangedChannels(void);

// Transmit current values of all parameters to the CAN bus.
// Called in response to a CAN_ID_REQUEST frame from the ESP32.
void     CAN_SendAllParameters(void);

void     CAN_ProcessADC(uint8_t channel, uint16_t raw);
void     CAN_ProcessButton(Button_t btn);
void     CAN_ProcessButtonRelease(Button_t btn);  // short-press release for modifier toggle
void     CAN_ModifierHoldTask(void);              // call every loop() — fires 2s hold disable
void     CAN_ProcessExtraButton(uint8_t which);

// Returns combined IC3 LED word (local | remote). Pass to IO_SetLEDs().
uint16_t CAN_GetLEDs(void);

// Returns IC4 LED byte (Solo/Clone/Sync). Pass to IO_SetIC4LEDs().
uint8_t  CAN_GetIC4LEDs(void);

// State accessors
bool     CAN_GetToggleState(Button_t btn);
uint8_t  CAN_GetModifierState(Button_t btn);
bool     CAN_GetExtraState(uint8_t which);

#endif
