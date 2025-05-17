// ADC0 Library
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

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc.h"
#include "nvic.h"

#define ADC_CTL_DITHER          0x00000040

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initAdc0Ss1()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Configure ADC
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC0_EMUX_R = ADC_EMUX_EM1_ALWAYS;               // select SS1 bit in ADCPSSI as trigger
    ADC0_SSCTL1_R = ADC_SSCTL1_END3;                 // mark four samples as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

void initAdc1Ss1()
{
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);

    // Configure ADC
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC1_EMUX_R = ADC_EMUX_EM1_ALWAYS;               // select SS1 bit in ADCPSSI as trigger, to sample always
    ADC1_SSCTL1_R = ADC_SSCTL1_END3;                 // mark four samples as the end
    ADC1_SSMUX1_R |= 0x4210;                         // Set analog input for single sample
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Set SS1 input sample average count
void setAdc1Ss1Log2AverageCount(uint8_t log2AverageCount)
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS1) for programming
    ADC1_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC1_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC1_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS1 for operation
}

// Set SS3 analog input
void setAdc0Ss1Mux()
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS3) for programming
    ADC0_SSMUX1_R |= 0x4210;                         // Set analog input for single sample
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS3 for operation
}

void setAdc1Ss1Mux()
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;                // disable sample sequencer 1 (SS3) for programming
    ADC1_SSMUX1_R |= 0x4210;                         // Set analog input for single sample
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;                 // enable SS3 for operation
}

// Request and read one sample from SS1
int16_t readAdc0Ss1()
{
//    ADC0_PSSI_R |= ADC_PSSI_SS1;                     // set start bit
    return ADC0_SSFIFO1_R;                           // get single result from the FIFO
}

// Initialize the DC Comparator
void initDccmp()
{
    ADC1_SSDC1_R = 0x3210;                            // Select DCC units 0,1,2,3 for S0DCSEL,S1DCSEL,S2DCSEL,S3DCSEL respectively
    ADC1_SSOP1_R |= ADC_SSOP1_S0DCOP | ADC_SSOP1_S1DCOP | ADC_SSOP1_S2DCOP | ADC_SSOP1_S3DCOP;      // send the FIFO samples to the DCC's

    // write the comparison values for comp0 and comp1 in the ADC DCC range registers
    ADC1_DCCMP0_R |= 0x120<<ADC_DCCMP0_COMP0_S | 0x150<<ADC_DCCMP0_COMP1_S;
    ADC1_DCCMP1_R |= 0x120<<ADC_DCCMP1_COMP0_S | 0x150<<ADC_DCCMP1_COMP1_S;
    ADC1_DCCMP2_R |= 0x120<<ADC_DCCMP2_COMP0_S | 0x150<<ADC_DCCMP2_COMP1_S;
    ADC1_DCCMP3_R |= 0x120<<ADC_DCCMP3_COMP0_S | 0x150<<ADC_DCCMP3_COMP1_S;

    // Enable the DCC to raise interrupts to the ADC
    ADC1_DCCTL0_R |= ADC_DCCTL0_CIE | ADC_DCCTL0_CIC_HIGH | ADC_DCCTL0_CIM_ALWAYS;
    ADC1_DCCTL1_R |= ADC_DCCTL1_CIE | ADC_DCCTL1_CIC_HIGH | ADC_DCCTL1_CIM_ALWAYS;
    ADC1_DCCTL2_R |= ADC_DCCTL2_CIE | ADC_DCCTL2_CIC_HIGH | ADC_DCCTL2_CIM_ALWAYS;
    ADC1_DCCTL3_R |= ADC_DCCTL3_CIE | ADC_DCCTL3_CIC_HIGH | ADC_DCCTL3_CIM_ALWAYS;

    ADC1_IM_R |= ADC_IM_DCONSS1;                                        // Enable interrupts in the Interrupt mask register
    enableNvicInterrupt(INT_ADC1SS1);                                            // Enable interrupts for the ADC in NVIC
}
