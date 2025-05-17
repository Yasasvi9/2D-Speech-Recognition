// project test
// Yasasvi Vanapalli
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "adc.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "nvic.h"
#include "stringf.h"
#include "rgb_led.h"
#include "tm4c123gh6pm.h"

#define WINDOW_SIZE 16
#define SAMPLE_SIZE 64
#define MICS 3

int8_t  firstOrLastHit, firstHit, secondHit, lastHit, strayHit, k1, k2;
uint8_t controlTable[1024];
char str[50];
uint16_t pingBuffer[128] = {0,}, pongBuffer[128] = {0,}, window[WINDOW_SIZE];
uint16_t mic1[64], mic2[64], mic3[64], mic4[64];
uint16_t micro1, micro2, micro3, micro4, i, j, k, count = 0;
uint32_t crossCorrXY[(SAMPLE_SIZE-WINDOW_SIZE)+1], crossCorrYZ[(SAMPLE_SIZE-WINDOW_SIZE)+1], crossCorrZX[(SAMPLE_SIZE-WINDOW_SIZE)+1];
uint32_t crossCorrXYMax;
uint32_t crossCorrYZMax;
uint32_t crossCorrZXMax;
uint32_t Txy, Tyz, Tzx, delXY, delYZ, delZX;
uint16_t thetaPV;

#pragma DATA_ALIGN(controlTable,1024)

volatile uint32_t* primarySep;
volatile uint32_t* primaryDep;
volatile uint32_t* primaryCtrlWord;
volatile uint32_t* alternateSep;
volatile uint32_t* alternateDep;
volatile uint32_t* alternateCtrlWord;

// Initialize DMA
void initDma()
{
    SYSCTL_RCGCDMA_R |= SYSCTL_RCGCDMA_R0;                  // Enable clocks to the UDMA
    _delay_cycles(3);

    UDMA_CFG_R |= UDMA_CFG_MASTEN;                          // Enable the masten bit in the UDMA
    UDMA_CTLBASE_R = (uint32_t) (&controlTable[0]);         // pointing the base address of control table to the register
    // Configure channel attributes
    UDMA_PRIOSET_R |= UDMA_PRIOSET_SET_15;                  // Set ADC 0 SS1 to be high priority
    UDMA_ALTCLR_R |= UDMA_ALTCLR_CLR_15;                    // Clear the alternate control structure as instructed by the documentation 9.3.4
    UDMA_USEBURSTSET_R |= UDMA_USEBURSTSET_SET_15;          // Clear bit 15 to allow the DMA to respond to single and burst requests
    UDMA_REQMASKCLR_R |= UDMA_REQMASKCLR_CLR_15;            // Clear bit 15 to allow recognizing requsts for channel 15

    // Configure the channel control structure
    primarySep = (volatile uint32_t*)(uint32_t) (UDMA_CTLBASE_R + 0xF0);             // Source end pointer address
    primaryDep = (volatile uint32_t*)(uint32_t) (UDMA_CTLBASE_R + 0xF4);             // Destination end pointer address
    primaryCtrlWord = (volatile uint32_t*)(uint32_t) (UDMA_CTLBASE_R + 0xF8);                              // Control word address

    alternateSep = (volatile uint32_t*)((uint32_t) (UDMA_CTLBASE_R) + 0x2F0);
    alternateDep = (volatile uint32_t*)((uint32_t) (UDMA_CTLBASE_R) + 0x2F4);
    alternateCtrlWord = (volatile uint32_t*)((uint32_t) (UDMA_CTLBASE_R) + 0x2F8);

    *primarySep = (uint32_t) &ADC0_SSFIFO1_R;
    *primaryDep = (uint32_t) &pingBuffer[127];
    *primaryCtrlWord = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2 | 127<<UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG | UDMA_CHCTL_NXTUSEBURST;

    *alternateSep = (uint32_t) &ADC0_SSFIFO1_R;
    *alternateDep = (uint32_t) &pongBuffer[127];
    *alternateCtrlWord = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2 | 127<<UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG | UDMA_CHCTL_NXTUSEBURST;

    // configure adc 0 to raise interrupts
    ADC0_SSCTL1_R |= ADC_SSCTL0_IE3;                        // Enable interrupts to be raised for every 4th sample
    ADC0_IM_R |= ADC_IM_MASK1;                              // Enable the ADC 0 SS1 to send interrupts to the nvic controller
    enableNvicInterrupt(INT_ADC0SS1);
}

void adc0Isr()
{
    ADC0_ISC_R |= ADC_ISC_IN1;                              // Clear the ADC 0 SS 1 interrupt

    if((*primaryCtrlWord & UDMA_CHCTL_XFERMODE_M) == 0)    // Check if the ping buffer is full and set the DMA to overwrite the ping buffer after completing writes to the pong buffer in the next write cycles
    {
        *primaryCtrlWord = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2 | 127<<UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG | UDMA_CHCTL_NXTUSEBURST;
    }
    else if((*alternateCtrlWord & UDMA_CHCTL_XFERMODE_M) == 0)     // Check if the pong buffer is full and set the DMA to overwrite the pong buffer after completing writes to the ping buffer in the next write cycles
    {
        *alternateCtrlWord = UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_ARBSIZE_2 | 127<<UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG | UDMA_CHCTL_NXTUSEBURST;
        UDMA_ENASET_R |= UDMA_ENASET_SET_15;                        // Enable the DMA if pong buffer is full
    }

    UDMA_CHIS_R |= UDMA_CHIS_M;
}

// DC comparator Isr
void DccIsr()
{
    ADC1_DCISC_R |= ADC_DCISC_DCINT0;                       // DCC interrupt flag clear
    ADC1_IM_R &= ~ADC_IM_DCONSS1;                           // Disable DCC interrupts until correlation data is processsed from the png pong buffers
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                         // Enable Timer 1
}

// 1-shot timer 1 Isr
void timer1Isr()
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                       // clear interrupt flag

    UDMA_ENACLR_R |= UDMA_ENACLR_CLR_M;                      // disable DMA to perform calculations

    // Iterate through the ping pong buffers and segregate samples into separate arrays for each microphone
    for (i=0;i<256;i+=4)
    {
        if(i<128)
        {
            mic1[i/4] = pingBuffer[i];                            // Sample for microphone 1
            mic2[i/4] = pingBuffer[i + 1];                        // Sample for microphone 2
            mic3[i/4] = pingBuffer[i + 2];                        // Sample for microphone 3
            mic4[i/4] = pingBuffer[i + 3];                        // Sample for microphone 4
        }
        else
        {
            j = i-128;                                      // resetting the value of i to copy the correct indices from the pong buffer
            mic1[i/4] = pongBuffer[j];                            // Sample for microphone 1
            mic2[i/4] = pongBuffer[j + 1];                        // Sample for microphone 2
            mic3[i/4] = pongBuffer[j + 2];                        // Sample for microphone 3
            mic4[i/4] = pongBuffer[j + 3];                        // Sample for microphone 4
        }
    }

    // Start the correlation calculations
    for(i=0;i<(SAMPLE_SIZE - WINDOW_SIZE) + 1;i++)
    {
        crossCorrXY[i]=0;
        crossCorrYZ[i]=0;
        crossCorrZX[i]=0;
        for(j=0;j<WINDOW_SIZE;j++)
        {
            if(j<SAMPLE_SIZE)
            {
                crossCorrXY[i]+=mic2[j]*mic3[i+j];
                crossCorrYZ[i]+=mic3[j]*mic4[i+j];
                crossCorrZX[i]+=mic4[j]*mic2[i+j];
            }
        }
    }

    // Calculate the time delays between mic's detecting valid events
    crossCorrXYMax = crossCorrXY[0];
    crossCorrYZMax = crossCorrYZ[0];
    crossCorrZXMax = crossCorrZX[0];

    for(i=1;i<(SAMPLE_SIZE - WINDOW_SIZE) + 1;i++)
    {
        if(crossCorrXY[i]>crossCorrXYMax)
        {
            crossCorrXYMax = crossCorrXY[i];
            Txy = i;
            delXY = Txy*1;
        }
        if(crossCorrYZ[i]>crossCorrYZMax)
        {
            crossCorrYZMax = crossCorrYZ[i];
            Tyz = i;
            delYZ = Tyz*1;
        }
        if(crossCorrZX[i]>crossCorrZXMax)
        {
            crossCorrZXMax = crossCorrZX[i];
            Tzx = i;
            delZX = Tzx*2;
        }
    }

    // Compute the time delays and determine the order of Hits
    uint32_t deltaT[3] = {delXY, delYZ, delZX};


    // Angle calculations
    k1=1, k2=1;
    if(deltaT[0] == deltaT[2])
    {
        firstHit = 0;
        secondHit = 1;
        lastHit = 3;
        thetaPV = 0;
        setRgbColor(1023, 0, 0);                // red
    }
    else if(deltaT[0] == deltaT[1])
    {
        firstHit = 1;
        secondHit = 0;
        lastHit = 2;
        thetaPV = 120;
        setRgbColor(0, 1023, 0);                // green
    }
    else if(deltaT[1] == deltaT[2])
    {
        firstHit = 2;
        secondHit = 1;
        lastHit = 0;
        thetaPV = 240;
        setRgbColor(0, 0, 1023);                // blue
    }
    else if(deltaT[0]<deltaT[1] && deltaT[1]<deltaT[2])          // angle from 0 - 120 deg, closer to micX
    {
        k1 = 1, k2 = 1;
        firstHit = 0;
        secondHit = 1;
        lastHit = 2;
        thetaPV = 0 + k1*(deltaT[1] - deltaT[0]) + k2*(deltaT[1] - deltaT[0])^2;
        setRgbColor(1023, 1023, 0);             // yellow
    }
    else if(deltaT[1]<deltaT[0] && deltaT[0]<deltaT[2])          // angle from 0 - 120 deg, closer to micY
    {
        k1 = 0.5, k2 = 0.5;
        firstHit = 1;
        secondHit = 0;
        lastHit = 2;
        thetaPV = 120 - (k1*(deltaT[0] - deltaT[1]) + k2*(deltaT[0] - deltaT[1])^2);
        setRgbColor(512, 1023, 0);
    }
    else if(deltaT[1]<deltaT[2] && deltaT[2]<deltaT[0])          // angle from 120 - 240 deg, closer to micY
    {
        k1 = 1, k2 = 1;
        firstHit = 1;
        secondHit = 2;
        lastHit = 0;
        thetaPV = 120 + k1*(deltaT[2] - deltaT[1]) + k2*(deltaT[2] - deltaT[1])^2;
        setRgbColor(0, 512, 1023);              // somewhere cyan
    }
    else if(deltaT[2]<deltaT[1] && deltaT[1]<deltaT[0])          // angle from 120-240 deg, closer to micZ
    {
        k1 = 0.5, k2 = 0.5;
        firstHit = 2;
        secondHit = 1;
        lastHit = 0;
        thetaPV = 240 - (k1*(deltaT[1] - deltaT[2]) + k2*(deltaT[1] - deltaT[2])^2);
        setRgbColor(0, 1023, 1023);             // cyan
    }
    else if(deltaT[2]<deltaT[0] && deltaT[0]<deltaT[1])          // angle from 240 - 360 deg, closer to micZ
    {
        k1 = 1, k2 = 1;
        firstHit = 2;
        secondHit = 0;
        lastHit = 1;
        thetaPV = 240 + k1*(deltaT[0] - deltaT[2]) + k2*(deltaT[0] - deltaT[2])^2;
        setRgbColor(512, 0, 1023);              // some magenta
    }
    else if(deltaT[0]<deltaT[2] && deltaT[2]<deltaT[1])          // angle from 240 - 360, closer to micX
    {
        k1 = 1, k2 = 0.5;
        firstHit = 0;
        secondHit = 2;
        lastHit = 1;
        thetaPV = 360 - (k1*(deltaT[2] - deltaT[0]) + k2*(deltaT[2] - deltaT[0])^2);
        setRgbColor(1023, 0, 1023);             // magenta
    }

    ADC1_IM_R |= ADC_IM_DCONSS1;                            // enable the DCC interrupts to continue detecting valid events

    UDMA_ENASET_R |= UDMA_ENASET_SET_15;                    // enable the DMA again
}

// Initialize Timer 1
void initTimer1()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for 1 SHOT mode (count down)
    TIMER1_TAILR_R = 4000;                          // set load value to 10000 for 250us interrupt rate, 4000 for 100us interrupt
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    enableNvicInterrupt(INT_TIMER1A);                // turn-on interrupt 37 (TIMER1A) in NVIC
}

// intialize Hw
void initHw()
{
    // SYSTEM clock to 40MHz
    initSystemClockTo40Mhz();

    // GPIO CONFIGURATIONS
    enablePort(PORTE);
    enablePort(PORTD);
    _delay_cycles(3);

    selectPinAnalogInput(PORTD, 3);                                 // configure GPIO pin PD3 to be analog input
    selectPinAnalogInput(PORTE, 1);                                 // configure GPIO pin PE1 to be analog input
    selectPinAnalogInput(PORTE, 2);                                 // configure GPIO pin PE2 to be analog input
    selectPinAnalogInput(PORTE, 3);                                 // configure GPIO pin PE3 to be analog input

    setPinAuxFunction(PORTD, 3, GPIO_PCTL_PD3_AIN4);                // configure GPIO pin PD3 to be AIN4
    setPinAuxFunction(PORTE, 1, GPIO_PCTL_PE1_AIN2);                // configure GPIO pin PE1 to be AIN2
    setPinAuxFunction(PORTE, 2, GPIO_PCTL_PE2_AIN1);                // configure GPIO pin PE2 to be AIN1
    setPinAuxFunction(PORTE, 3, GPIO_PCTL_PE3_AIN0);                // configure GPIO pin PE3 to be AIN0
}

// main
void main()
{
    initHw();
    initAdc1Ss1();                      // initialize adc 1 ss1
    setAdc1Ss1Mux();                    // configure adc 1 ss1 to take AIN0,1,2,4 as inputs
    initAdc0Ss1();                      // initialize adc 0 ss1
    setAdc0Ss1Mux();                    // configure adc 0 ss1 to take AIN0,1,2,4 as inputs
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initDma();
    initDccmp();
    initTimer1();
    initRgb();
    int angleProcessed = 0;

    UDMA_ENASET_R |= UDMA_ENASET_SET_15;                        // Enable DMA
    USER_DATA data;

    while(1)
    {

        if(kbhitUart0())
        {
            getsUart0(&data);
            putsUart0(data.buffer);
            parseFields(&data);
            putcUart0('\n');

            if(isCommand(&data,"aoa", 0))
            {
                angleProcessed = 1;
            }
        }
        if(angleProcessed == 1)
        {
            snprintf(str, sizeof(str), "AoA : %4"PRIu16"\n", thetaPV);
            putsUart0(str);
        }
    }
}
