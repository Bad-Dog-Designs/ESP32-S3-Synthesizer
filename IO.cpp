#include "IO.h"
#include <Arduino.h>
#include <Wire.h>

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

// ==== Interrupt ====
#define MCP_INT_PIN 3

// IC4 output pin masks (bits that are LED outputs, not button inputs)
#define IC4_OUTPUT_MASK_A  0b10010100   // GPA2=Clone1, GPA4=Solo1, GPA7=Sync1
#define IC4_OUTPUT_MASK_B  0b10010100   // GPB2=Solo2,  GPB4=Clone2, GPB7=Sync2

static const uint8_t buttonBitMap[BTN_COUNT] =
{
    0,  // BTN_OSC1_SH    → IC4 GPA0
    1,  // BTN_OSC1_FOLD  → IC4 GPA1
    3,  // BTN_OSC1_CLONE → IC4 GPA3
    5,  // BTN_OSC1_SOLO  → IC4 GPA5
    6,  // BTN_OSC1_SYNC  → IC4 GPA6

    8,  // BTN_OSC2_SYNC  → IC4 GPB0
    9,  // BTN_OSC2_SOLO  → IC4 GPB1
    11, // BTN_OSC2_CLONE → IC4 GPB3
    13, // BTN_OSC2_FOLD  → IC4 GPB5
    14  // BTN_OSC2_SH    → IC4 GPB6
};

volatile bool mcpInterrupt = false;

static uint16_t buttons     = 0;
static uint16_t lastButtons = 0;
static uint16_t pressed     = 0;
static uint16_t released    = 0;

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
    uint16_t mask = (1 << buttonBitMap[btn]);
    if (pressed & mask)
    {
        pressed &= ~mask;
        return true;
    }
    return false;
}

bool IO_IsButtonHeld(Button_t btn)
{
    return (buttons & (1u << buttonBitMap[btn])) != 0;
}

bool IO_IsButtonReleased(Button_t btn)
{
    uint16_t mask = (1 << buttonBitMap[btn]);
    if (released & mask)
    {
        released &= ~mask;
        return true;
    }
    return false;
}

void handleMCPInterrupt()
{
    mcpInterrupt = true;
}

void IO_Init(void)
{
    Wire.begin();

    // IC3 — all outputs (LEDs only)
    mcpWrite(MCP_IC3, IODIRA, 0x00);
    mcpWrite(MCP_IC3, IODIRB, 0x00);

    // IC4 — mixed: inputs = buttons, outputs = LEDs
    mcpWrite(MCP_IC4, IODIRA, 0b01101011);   // 0=output, 1=input
    mcpWrite(MCP_IC4, IODIRB, 0b01101011);

    // Pull-ups on input pins
    mcpWrite(MCP_IC4, GPPUA, 0b01101011);
    mcpWrite(MCP_IC4, GPPUB, 0b01101011);

    // Mirror INTA/INTB so either pin triggers
    mcpWrite(MCP_IC4, IOCON, 0b01000000);

    // Interrupt on change for all input pins
    mcpWrite(MCP_IC4, GPINTENA, 0b01101011);
    mcpWrite(MCP_IC4, GPINTENB, 0b01101011);

    mcpWrite(MCP_IC4, INTCONA, 0x00);
    mcpWrite(MCP_IC4, INTCONB, 0x00);

    // Ensure all IC4 LED outputs start low (off)
    uint8_t gpioA = mcpRead(MCP_IC4, GPIOA);
    uint8_t gpioB = mcpRead(MCP_IC4, GPIOB);
    mcpWrite(MCP_IC4, GPIOA, gpioA & ~IC4_OUTPUT_MASK_A);
    mcpWrite(MCP_IC4, GPIOB, gpioB & ~IC4_OUTPUT_MASK_B);

    pinMode(MCP_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), handleMCPInterrupt, FALLING);
}

void IO_Task(void)
{
    if (!mcpInterrupt) return;
    mcpInterrupt = false;

    uint8_t a = mcpRead(MCP_IC4, INTCAPA);
    uint8_t b = mcpRead(MCP_IC4, INTCAPB);

    uint16_t raw  = ((uint16_t)b << 8) | a;
    uint16_t mask = ((uint16_t)0b01101011 << 8) | 0b01101011;

    // Invert: buttons are active LOW
    buttons = (~raw) & mask;

    uint16_t changed = buttons ^ lastButtons;
    pressed          = changed & buttons;
    released         = changed & ~buttons;
    lastButtons      = buttons;
}

uint16_t IO_GetButtons(void)        { return buttons; }
uint16_t IO_GetButtonsPressed(void) { uint16_t p = pressed;  pressed  = 0; return p; }
uint16_t IO_GetButtonsReleased(void){ uint16_t r = released; released = 0; return r; }

// ----------------------------------------------------------------
// IO_SetLEDs — IC3 only (all-output chip, no masking needed)
// Low byte → GPIOA (GPA7..GPA0), high byte → GPIOB (GPB7..GPB0)
// ----------------------------------------------------------------
void IO_SetLEDs(uint16_t ledMask)
{
    mcpWrite(MCP_IC3, GPIOA,  ledMask        & 0xFF);
    mcpWrite(MCP_IC3, GPIOB, (ledMask >> 8)  & 0xFF);
}

// ----------------------------------------------------------------
// IO_SetIC4LEDs — IC4 output pins only (Solo, Clone, Sync LEDs)
//
// ic4Mask bit layout (IC4_LED_BIT_xxx in param_map.h):
//   Bit 0 → IC4 GPA2 → Led Clone 1
//   Bit 1 → IC4 GPA4 → Led Solo 1
//   Bit 2 → IC4 GPA7 → Led Sync 1
//   Bit 3 → IC4 GPB2 → Led Solo 2
//   Bit 4 → IC4 GPB4 → Led Clone 2
//   Bit 5 → IC4 GPB7 → Led Sync 2
//
// Reads current IC4 GPIO state first to preserve input pin values.
// ----------------------------------------------------------------
void IO_SetIC4LEDs(uint8_t ic4Mask)
{
    // Build the desired output byte for GPA (bits 2, 4, 7)
    uint8_t newA = 0;
    if (ic4Mask & (1 << 0)) newA |= (1 << 2);  // Clone 1 → GPA2
    if (ic4Mask & (1 << 1)) newA |= (1 << 4);  // Solo 1  → GPA4
    if (ic4Mask & (1 << 2)) newA |= (1 << 7);  // Sync 1  → GPA7

    // Build the desired output byte for GPB (bits 2, 4, 7)
    uint8_t newB = 0;
    if (ic4Mask & (1 << 3)) newB |= (1 << 2);  // Solo 2  → GPB2
    if (ic4Mask & (1 << 4)) newB |= (1 << 4);  // Clone 2 → GPB4
    if (ic4Mask & (1 << 5)) newB |= (1 << 7);  // Sync 2  → GPB7

    // Read-modify-write: preserve input pin state, update output pins only
    uint8_t gpioA = mcpRead(MCP_IC4, GPIOA);
    uint8_t gpioB = mcpRead(MCP_IC4, GPIOB);

    gpioA = (gpioA & ~IC4_OUTPUT_MASK_A) | (newA & IC4_OUTPUT_MASK_A);
    gpioB = (gpioB & ~IC4_OUTPUT_MASK_B) | (newB & IC4_OUTPUT_MASK_B);

    mcpWrite(MCP_IC4, GPIOA, gpioA);
    mcpWrite(MCP_IC4, GPIOB, gpioB);
}
