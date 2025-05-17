// String Functions Library
// Yasasvi Vanapalli

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40MHz

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "stringf.h"
#include "uart0.h"

// Parse Fields function
void parseFields(USER_DATA* data)
{
    uint8_t bit = 0;
    uint8_t count = 0;
    data->fieldCount = 0;
    char prevField = 'd'; // delimiter
    char currFieldType;

    while(data->buffer[bit] != '\0')
    {
        char currField = data->buffer[bit];
        if((currField >= 'a' && currField <= 'z') || (currField >= 'A' && currField <= 'Z'))
        {
            currFieldType = 'a'; // alphabet
        }
        else if((currField >= '0' && currField <= '9') || (currField == '-') || (currField == '.'))
        {
            currFieldType = 'n'; // numeric
        }
        else
        {
            currFieldType = 'd'; // delimiter
        }
        if(prevField == 'd' && (currFieldType == 'a' || currFieldType == 'n'))
        {
            data->fieldType[count] = currFieldType;
            data->fieldPosition[count] = bit;
            data->fieldCount+=1;
            count+=1;
            prevField = currFieldType;
            if(data->fieldCount == MAX_FIELDS)
            {
                return;
            }
        }
        if(currFieldType == 'd')
        {
            data->buffer[bit] = '\0';
            prevField = 'd';
        }
        bit+=1;
    }
}

// get field string function
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber < data->fieldCount)
    {
        char* string;
        int index = 0;
        uint8_t field = data->fieldPosition[fieldNumber];
        if(data->fieldType[field] == 'a')
        {
            while(data->buffer[field] != '\0')
            {
                string[index] = data->buffer[field];
                index +=1;
                field +=1;
            }
            string[index] = '\0';
            return string;
        }
    }
    else
    {
        return NULL; // field out of range, exit with error code
    }
    return NULL;
}

// get field integer function

uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    uint8_t local = fieldNumber;
    if(local <= data->fieldCount)
    {
        uint8_t field = data->fieldPosition[local];
        if(data->fieldType[local] == 'n')
        {
            uint32_t integer = (uint32_t)atoi(&data->buffer[field]);
            return integer;
        }
    }
    else
    {
        return 0; // field out of range, exit with error code
    }
    return 0;
}

// is command function
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    //if(minArguments >= 2)   //min args >=2 (default)
    if(data->fieldCount-1 >= minArguments)
    {
        int i=0;
        int count=0;
        int index = data->fieldPosition[i];
        while(data->buffer[index]!='\0')
        {
            if(data->buffer[index]==strCommand[count])
            {
                count++;
                index++;
                continue;
             }
             else
             {
                 return false;
             }
        }
        return true;
    }
    else
    {
        return false;
    }
}








