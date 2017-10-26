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
	
#define CYCLE 200000	
	
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

void __ISR(_INPUT_CAPTURE_4_VECTOR, IPL5SOFT) IC4ISR(void) {
    static unsigned int pts=0;      // previous time stamp for right sensor
    static int state = -1;          // state of right sensor
    unsigned int ts;                // time stamp for right sensor
    int unusedR = IC4BUF;           // the value of timer2 right now, doesn't matter
    unusedR = IC4BUF;               // read multiple times to ensure the buffer is cleared
    unusedR = IC4BUF;               // ...
    unusedR = IC4BUF;               // ...
    if (PORTBbits.RB7) {            // ignore rising edges
        IFS0bits.IC4IF = 0;         // clear interrupt flag
        return;                     // exit the ISR
    }    
    ts = _CP0_GET_COUNT();          // get the current time
    if (pts > 0) {                  // won't calculate anything the first time through
        unsigned int l = ts - pts;  // calculate elapsed time
        if  (l > CYCLE) {           // indicates a synchronization cycle
            state = 0;             // prepare to read y angle (from right to left)
        } else {
            if (state >= 0) {
                switch (state) {   
                    case 0: {
                        yAng=l*0.0006;  // read y angle in degrees (0 to 120)
                        break;
                    }
                    case 2: {
                        xAng=l*0.0006;  // read x angle in degrees (0 to 120)
                        break;
                    }
                }
                state++;                       // cycle through states
                if (state >=4 ) state = -1;   // reset state
            }
        }
    }
    pts=ts;           // set the previous time stamp for the next time through
    IFS0bits.IC4IF = 0; // clear the interrupt flag
}

 
/*******************************************************************************
 End of File
*/

