#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>

#define ADC_NUM_CHANNELS 18   // 16 mux + A6 + A7

typedef struct
{
    uint16_t value;
    uint16_t lastValue;
    bool     changed;
} ADC_Channel_t;

void     ADC_Init(void);
void     ADC_Task(void);        // call every loop()

// Mux channels 0-15
bool     ADC_GetChannelChanged(uint8_t ch);
uint16_t ADC_GetChannelValue(uint8_t ch);

// Extra analog buttons (A6=bit0, A7=bit1)
uint8_t  ADC_GetExtraPressed(void);
uint8_t  ADC_GetExtraReleased(void);
uint8_t  ADC_GetExtraState(void);

#endif
