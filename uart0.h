// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include "stringf.h"

#ifndef UART0_H_
#define UART0_H_

#define MAX_CHARS 80
#define MAX_FIELDS 6
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart0();
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
void getsUart0(USER_DATA* data);
bool kbhitUart0();

#endif
