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
//   bits 7..0  → IC3 GPA7..GPA0
//   bits 15..8 → IC3 GPB7..GPB0
//   Bit position = LED_BIT_xxx value in param_map.h.
//   Simple: writes low byte to GPIOA, high byte to GPIOB.
//
// IO_SetIC4LEDs(uint8_t)
//   Drives the output pins on IC4 (0x21) — mixed GPIO MCP23017.
//   Bit layout (IC4_LED_BIT_xxx in param_map.h):
//     Bit 0 → IC4 GPA2 → Led Clone 1
//     Bit 1 → IC4 GPA4 → Led Solo 1
//     Bit 2 → IC4 GPA7 → Led Sync 1
//     Bit 3 → IC4 GPB2 → Led Solo 2
//     Bit 4 → IC4 GPB4 → Led Clone 2
//     Bit 5 → IC4 GPB7 → Led Sync 2
//   Input pins on IC4 are preserved — only output pins are touched.
// ----------------------------------------------------------------
void IO_SetLEDs(uint16_t ledMask);
void IO_SetIC4LEDs(uint8_t ic4Mask);

typedef enum
{
    BTN_OSC1_SH = 0,
    BTN_OSC1_FOLD,
    BTN_OSC1_CLONE,
    BTN_OSC1_SOLO,
    BTN_OSC1_SYNC,

    BTN_OSC2_SYNC,
    BTN_OSC2_SOLO,
    BTN_OSC2_CLONE,
    BTN_OSC2_FOLD,
    BTN_OSC2_SH,

    BTN_COUNT
} Button_t;

bool IO_IsButtonPressed(Button_t btn);
bool IO_IsButtonReleased(Button_t btn);

// Returns true if the button is currently held down (non-consuming).
// Based on the last interrupt snapshot — suitable for hold-duration checks.
bool IO_IsButtonHeld(Button_t btn);

#endif
