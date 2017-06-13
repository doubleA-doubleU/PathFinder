/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
	
	
	
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, IPL5SOFT) IC4ISR(void) {

    // get the time the interrupt occurred
    long mic = _CP0_GET_COUNT();
    int i;

    int unused = IC4BUF; // the value of timer2 right now, doesn't matter

    // check for coreTimer overflow
    while (mic - V1.prevMic < 0) {
        mic = mic + 4294967295; // largest unsigned 32bit int
    }

    // shift the time into the buffer
    for (i = 0; i < 10; i++) {
        V1.changeTime[i] = V1.changeTime[i + 1];
    }
    V1.changeTime[10] = mic;

    // if the buffer is not full
    if (V1.collected < 11) {
        V1.collected++;
    } else {
        // if the timer values match the waveform pattern when about 7 feet away from the emitter
        if ((V1.changeTime[1] - V1.changeTime[0] > 7000 * 24) && (V1.changeTime[3] - V1.changeTime[2] > 7000 * 24) && (V1.changeTime[6] - V1.changeTime[5] < 50 * 24) && (V1.changeTime[10] - V1.changeTime[9] < 50 * 24)) {
            V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_CORE;
            V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_CORE;
            V1.useMe = 1;
            LATAbits.LATA4 = !LATAbits.LATA4; // blink your LED to know you got new position data info
        }
    }

    V1.prevMic = mic;

    IFS0bits.IC4IF = 0; // clear the interrupt flag
}

 
/*******************************************************************************
 End of File
*/

