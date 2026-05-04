#ifndef IO_H
#define IO_H

#include <stdint.h>
#include <stdbool.h>

void IO_Init(void);
void IO_Task(void);

// Button interface
uint16_t IO_GetButtons(void);
uint16_t IO_GetButtonsPressed(void);
uint16_t IO_GetButtonsReleased(void);

// ----------------------------------------------------------------
// LED control
//
// IO_SetLEDs(uint16_t)
//   Drives IC3 (0x20) — all-output MCP23017.
//   bits  7..0 → IC3 GPA7..GPA0
//   bits 15..8 → IC3 GPB7..GPB0
//   Bit position = LED_BIT_xxx value in param_map.h.
//
// IO_SetRetrigLEDs(bool lfo1, bool lfo2)
//   Drives LFO retrigger indicator LEDs on IC3 GPA4 (LFO1) and
//   GPA5 (LFO2). These were originally intended for ATmega A6/A7
//   but those pins are analog-input only on ATmega328p and cannot
//   drive outputs. Routed to spare IC3 GPA pins instead.
//   Both LEDs are remote — lit by CAN ACK from ESP32.
// ----------------------------------------------------------------
void IO_SetLEDs(uint16_t ledMask);
void IO_SetRetrigLEDs(bool lfo1, bool lfo2);

// ----------------------------------------------------------------
// Button enum
//
// All 7 buttons are on IC4 (0x21) GPB0–GPB6 (all inputs).
// IC4 GPA is entirely unused on the Commons PCB.
// ----------------------------------------------------------------
typedef enum
{
    BTN_FILTER_MODE  = 0,   // IC4 GPB0 — cycles LP→BP→HP
    BTN_MOD_MUTE     = 1,   // IC4 GPB1 — toggle ring mod mute
    BTN_RING_ENABLE  = 2,   // IC4 GPB2 — toggle ring mod enable
    BTN_LFO2_TRIG    = 3,   // IC4 GPB3 — LFO2 retrigger (momentary)
    BTN_ENV_TRACK    = 4,   // IC4 GPB4 — toggle Env/Track pot function
    BTN_POLY_MODE    = 5,   // IC4 GPB5 — cycles Poly→Duo→Unison
    BTN_LFO1_TRIG    = 6,   // IC4 GPB6 — LFO1 retrigger (momentary)

    BTN_COUNT
} Button_t;

bool IO_IsButtonPressed(Button_t btn);
bool IO_IsButtonReleased(Button_t btn);

#endif
