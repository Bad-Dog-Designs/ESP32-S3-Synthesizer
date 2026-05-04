#include "IO.h"
#include <Arduino.h>
#include <Wire.h>

// ================================================================
// IO.cpp — Commons PCB
//
// IC3 (0x20): all outputs — 16 LED bits
// IC4 (0x21): GPA all unused, GPB0-GPB6 inputs (buttons)
//
// LFO retrigger LEDs: IC3 GPA4 (LFO1) and GPA5 (LFO2)
//   Note: originally designed for ATmega A6/A7 but those pins are
//   analog-input only on ATmega328p. Rerouted to spare IC3 GPA pins.
// ================================================================

// ==== MCP Addresses ====
#define MCP_IC3 0x20
#define MCP_IC4 0x21

// ==== Registers ====
#define IODIRA   0x00
#define IODIRB   0x01
#define GPIOA    0x12
#define GPIOB    0x13
#define IOCON    0x0A
#define GPPUA    0x0C
#define GPPUB    0x0D
#define GPINTENA 0x04
#define GPINTENB 0x05
#define INTCONA  0x08
#define INTCONB  0x09
#define INTCAPA  0x10
#define INTCAPB  0x11

#define MCP_INT_PIN 3

// IC4 GPB button bit positions (within GPB byte, 0-based)
// These map to Button_t enum order
static const uint8_t buttonGPBBit[BTN_COUNT] =
{
    0,   // BTN_FILTER_MODE  → IC4 GPB0
    1,   // BTN_MOD_MUTE     → IC4 GPB1
    2,   // BTN_RING_ENABLE  → IC4 GPB2
    3,   // BTN_LFO2_TRIG    → IC4 GPB3
    4,   // BTN_ENV_TRACK    → IC4 GPB4
    5,   // BTN_POLY_MODE    → IC4 GPB5
    6,   // BTN_LFO1_TRIG    → IC4 GPB6
};

// In the 16-bit button word: IC4 GPB maps to bits 8-15
// buttonBitMap[b] = bit position in 16-bit word
static const uint8_t buttonBitMap[BTN_COUNT] =
{
    8,   // BTN_FILTER_MODE  → bit 8  (IC4 GPB0)
    9,   // BTN_MOD_MUTE     → bit 9  (IC4 GPB1)
    10,  // BTN_RING_ENABLE  → bit 10 (IC4 GPB2)
    11,  // BTN_LFO2_TRIG    → bit 11 (IC4 GPB3)
    12,  // BTN_ENV_TRACK    → bit 12 (IC4 GPB4)
    13,  // BTN_POLY_MODE    → bit 13 (IC4 GPB5)
    14,  // BTN_LFO1_TRIG    → bit 14 (IC4 GPB6)
};

// IC4 GPB input mask — bits 0-6 are buttons, bit 7 unused
#define IC4_GPB_INPUT_MASK  0b01111111

volatile bool mcpInterrupt = false;

static uint16_t buttons     = 0;
static uint16_t lastButtons = 0;
static uint16_t pressed     = 0;
static uint16_t released    = 0;

// LFO retrigger LED state (drives IC3 GPA4/GPA5 via IO_SetLEDs)
// Stored here so IO_SetRetrigLEDs can merge into the IC3 word.
static bool retrigLFO1 = false;
static bool retrigLFO2 = false;

// ==== Low-level I2C ====
static void mcpWrite(uint8_t addr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mcpRead(uint8_t addr, uint8_t reg)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

bool IO_IsButtonPressed(Button_t btn)
{
    uint16_t mask = (1u << buttonBitMap[btn]);
    if (pressed & mask) { pressed &= ~mask; return true; }
    return false;
}

bool IO_IsButtonReleased(Button_t btn)
{
    uint16_t mask = (1u << buttonBitMap[btn]);
    if (released & mask) { released &= ~mask; return true; }
    return false;
}

void handleMCPInterrupt() { mcpInterrupt = true; }

void IO_Init(void)
{
    Wire.begin();

    // IC3 — all outputs (LEDs)
    mcpWrite(MCP_IC3, IODIRA, 0x00);
    mcpWrite(MCP_IC3, IODIRB, 0x00);

    // IC4 — GPA all unused (inputs by default), GPB0-6 inputs (buttons)
    mcpWrite(MCP_IC4, IODIRA, 0xFF);              // GPA all input (unused)
    mcpWrite(MCP_IC4, IODIRB, IC4_GPB_INPUT_MASK);

    // Pull-ups on button inputs
    mcpWrite(MCP_IC4, GPPUA, 0x00);              // GPA — no pullups needed (unused)
    mcpWrite(MCP_IC4, GPPUB, IC4_GPB_INPUT_MASK);

    // Mirror INTA/INTB
    mcpWrite(MCP_IC4, IOCON, 0b01000000);

    // Interrupt on change for button pins
    mcpWrite(MCP_IC4, GPINTENA, 0x00);
    mcpWrite(MCP_IC4, GPINTENB, IC4_GPB_INPUT_MASK);

    mcpWrite(MCP_IC4, INTCONA, 0x00);
    mcpWrite(MCP_IC4, INTCONB, 0x00);

    // All IC3 LEDs off at startup
    mcpWrite(MCP_IC3, GPIOA, 0x00);
    mcpWrite(MCP_IC3, GPIOB, 0x00);

    pinMode(MCP_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), handleMCPInterrupt, FALLING);
}

void IO_Task(void)
{
    if (!mcpInterrupt) return;
    mcpInterrupt = false;

    // Read INTCAP — captures pin state at moment of interrupt
    // Only GPB is relevant (GPA unused)
    uint8_t b = mcpRead(MCP_IC4, INTCAPB);

    // Buttons are active LOW — invert and mask
    uint16_t raw     = (uint16_t)b << 8;
    uint16_t mask    = (uint16_t)IC4_GPB_INPUT_MASK << 8;
    buttons          = (~raw) & mask;

    uint16_t changed = buttons ^ lastButtons;
    pressed          = changed & buttons;
    released         = changed & ~buttons;
    lastButtons      = buttons;
}

uint16_t IO_GetButtons(void)         { return buttons;  }
uint16_t IO_GetButtonsPressed(void)  { uint16_t p = pressed;  pressed  = 0; return p; }
uint16_t IO_GetButtonsReleased(void) { uint16_t r = released; released = 0; return r; }

// ----------------------------------------------------------------
// IO_SetLEDs — drives IC3 (all-output)
// bits 7..0  → IC3 GPA (GPA7..GPA0)
// bits 15..8 → IC3 GPB (GPB7..GPB0)
//
// LFO retrigger LEDs (GPA4/GPA5) are merged in from retrigLFO1/2
// state so callers don't need to manage them separately.
// ----------------------------------------------------------------
void IO_SetLEDs(uint16_t ledMask)
{
    // Merge retrigger LED bits into GPA
    uint8_t gpa = (uint8_t)(ledMask & 0xFF);
    if (retrigLFO1) gpa |= (1 << 4);   // GPA4 = LFO1 retrigger LED
    if (retrigLFO2) gpa |= (1 << 5);   // GPA5 = LFO2 retrigger LED

    mcpWrite(MCP_IC3, GPIOA, gpa);
    mcpWrite(MCP_IC3, GPIOB, (ledMask >> 8) & 0xFF);
}

// ----------------------------------------------------------------
// IO_SetRetrigLEDs — set LFO retrigger LED state.
// Called from CAN_Task when an ACK frame arrives from ESP32.
// State is latched and merged into IO_SetLEDs on the next call.
// ----------------------------------------------------------------
void IO_SetRetrigLEDs(bool lfo1, bool lfo2)
{
    retrigLFO1 = lfo1;
    retrigLFO2 = lfo2;
}
