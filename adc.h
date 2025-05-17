// ADC0  Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// ADC0 SS3

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initAdc0Ss1();
void initAdc1Ss1();
void setAdc1Ss1Log2AverageCount(uint8_t log2AverageCount);
void setAdc0Ss1Mux();
void setAdc1Ss1Mux();
int16_t readAdc0Ss1();
void initDccmp();
#endif
