/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"
#include <stdio.h>
#include <xc.h>
#include <math.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0, j = 0, startTime = 0, go = 0;
char rx[64]; // the raw data
int rxPos = 0; // how much data has been stored
int gotRx = 0; // the flag
int rxVal = 0; // a place to store the int that was received
int dutyL =0, dutyR = 0; // PWM duty cycles for left and right motors
double xPos = 0.0, yPos = 0.0, xStart = 0.0, yStart = 0.0;
float lapTime = 0.0; // in seconds

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];
    
    // initialize TMR2 (servo 50 Hz), TMR3 (motors 20 kHz), OC1 (left), OC2 (right), OC3 (servo), IC4 (position)
    T2CONbits.TCKPS = 0b110;    // Timer2 prescaler N=64 (1:64)
	PR2 = 24999;                // period = (PR2+1) * N * 12.5 ns = 20 ms, 50 Hz
	TMR2 = 0;                   // initial TMR2 count is 0
    T2CONbits.ON = 1;           // turn on Timer2
    
    T3CONbits.TCKPS = 0;        // Timer3 prescaler N=1 (1:1)
	PR3 = 3999;                 // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
	TMR3 = 0;                   // initial TMR3 count is 0
    T3CONbits.ON = 1;           // turn on Timer3
    
    RPA0Rbits.RPA0R = 0b0101;   // A0 is OC1 (left motor)
    TRISBbits.TRISB2 = 0;       // B2 is digital output for left motor
    LATBbits.LATB2 = 1;         // counterclockwise
	OC1CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC1CON bits are defaults
	OC1CONbits.OCTSEL = 1;      // OC1 uses Timer3
	OC1RS = 0;                  // duty cycle = OC1RS/(PR3+1) = 0%
	OC1R = 0;                   // initialize before turning OC1 on; afterward it is read-only
    OC1CONbits.ON = 1;			// turn on OC1
    
    RPA1Rbits.RPA1R = 0b0101;   // A1 is OC2 (right motor)
    TRISBbits.TRISB3 = 0;       // B3 is digital output for right motor
    LATBbits.LATB3 = 1;         // clockwise (inverted motor leads)
    OC2CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC2CON bits are defaults
	OC2CONbits.OCTSEL = 1;      // OC2 uses Timer3
	OC2RS = 0;                  // duty cycle = OC2RS/(PR3+1) = 0%
	OC2R = 0;                   // initialize before turning OC2 on; afterward it is read-only
    OC2CONbits.ON = 1;			// turn on OC2
    
    RPB14Rbits.RPB14R = 0b0101; // B14 is OC3
    OC3CONbits.OCM = 0b110;     // PWM mode without fault pin; other OC3CON bits are defaults
	OC3CONbits.OCTSEL = 0;      // OC3 uses Timer2
	OC3RS = 1080;               // duty cycle = OC3RS/(PR3+1)
	OC3R = 1080;                // initialize before turning OC3 on; afterward it is read-only
    OC3CONbits.ON = 1;			// turn on OC3
    
    // initialize the sensor variables
    V1.prevMic = 0;
    V1.horzAng = 0;
    V1.vertAng = 0;
    V1.useMe = 0;
    V1.collected = 0;

    TRISBbits.TRISB15 = 1;      // connect the TS3633 ENV pin to B15
    IC4Rbits.IC4R = 0b0011;     // B15 is IC4 (input capture 4)
    IC4CONbits.ICM = 1;         // detect rising and falling edges
    IC4CONbits.ICI = 0;         // interrupt on an edge
    IC4CONbits.ICTMR = 0;       // store the value of timer3, but we're actually just using the interrupt and ignoring timer3
    IC4CONbits.FEDGE = 0;       // first event is falling edge, doesn't really matter
    IC4CONbits.ON = 1;
    IPC4bits.IC4IP = 5;         // step 4: interrupt priority 5
    IPC4bits.IC4IS = 1;         // step 4: interrupt priority 1
    IFS0bits.IC4IF = 0;         // step 5: clear the int flag
    IEC0bits.IC4IE = 1;         // step 6: enable INT0 by setting IEC0<3>
    
    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:      
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }         

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:  
            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
                int ii = 0;
                // loop thru the characters in the buffer
                while (appData.readBuffer[ii] != 0) {
                    // if you got a newline
                    if (appData.readBuffer[ii] == '\n' || appData.readBuffer[ii] == '\r') {
                        rx[rxPos] = 0; // end the array
                        sscanf(rx, "%d", &rxVal); // get the int out of the array
                        gotRx = 1; // set the flag
                        break; // get out of the while loop
                    } else if (appData.readBuffer[ii] == 0) {
                        break; // there was no newline, get out of the while loop
                    } else {
                        // save the character into the array
                        rx[rxPos] = appData.readBuffer[ii];
                        rxPos++;
                        ii++;
                    }
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (gotRx || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 25)) { // 25 Hz
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }
            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            if (gotRx) { // rxVal should be an int between 0 and 640 (setting motor speeds from Center of Mass)

                // establish starting position the first time through the loop
                if (lapTime == 0.0) {
                    xStart = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
                    yStart = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
                }

                if (rxVal <= 0) {
                    dutyL = 30;
                    dutyR = 30;
                    LATBbits.LATB2 = 0;
                    LATBbits.LATB3 = 0;
                    OC3RS = 1080;
                    go = 1;
                } else if ( rxVal > 640) {
                    LATBbits.LATB2 = 1;
                    LATBbits.LATB3 = 1;
                    dutyL = 0;
                    dutyR = 0;
                    OC3RS = 1080;
                    go = 0;
                    lapTime = 0.0;
                } else if (rxVal >= 320) {
                    LATBbits.LATB2 = 1;
                    LATBbits.LATB3 = 1;
                    dutyL = 100;
                    dutyR = 180 - (rxVal/4); // dutyR will be between 20 and 100
                    go = 1;
                } else {
                    LATBbits.LATB2 = 1;
                    LATBbits.LATB3 = 1;
                    dutyL = 20 + (rxVal/4); // dutyL will be between 20 and 100
                    dutyR = 100;
                    go = 1;
                }
                OC1RS = dutyL*40; // convert duty cycle based on period register
                OC2RS = 0.965*dutyR*40;

                rxPos = 0;
                gotRx = 0; // clear the flag
                rxVal = 0; // clear the int

                for (j = 0; j < 64; j++) {
                    rx[j] = 0; // clear the array
                }
            }

            if (go == 1) {
                // send position and lap time to phone
                xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
                yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
                lapTime = lapTime + ((float) (_CP0_GET_COUNT() - startTime)) / 24000000;
                len = sprintf(dataOut, "X: %.3f (ft)\r\nY: %.3f (ft)\r\nLap Time: %.3f (s)\r\n", xPos, yPos, lapTime);
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                
                // wave flag using servo
                OC3RS = 1080 + 300*sin(7*lapTime);
                
            } else {
                len = 1;
                dataOut[0] = 0; // send nothing
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            }

            // victory condition (lap complete)
            if (abs(xPos - xStart) < 0.1 && abs(yPos - yStart) < 0.1 && lapTime > 10.0 && xPos > -20000.0) {
                // servo to 90 degrees
                OC3RS = 1080;

                // drive forward for 1 second
                OC1RS = 100*40;
                OC2RS = 0.965*100*40;
                startTime = _CP0_GET_COUNT();
                while (_CP0_GET_COUNT() - startTime < 1*(48000000/2)) {;}

                // left spin for 2.25 seconds
                LATBbits.LATB2 = 0; // left motor reverse
                LATBbits.LATB3 = 1; // right motor forward
                startTime = _CP0_GET_COUNT();
                while (_CP0_GET_COUNT() - startTime < 2.25*(48000000/2)) {;}

                // right spin for 2.4 seconds
                LATBbits.LATB2 = 1; // left motor forward
                LATBbits.LATB3 = 0; // right motor reverse
                startTime = _CP0_GET_COUNT();
                while (_CP0_GET_COUNT() - startTime < 2.4*(48000000/2)) {;}

                // reset variables
                LATBbits.LATB2 = 1;
                LATBbits.LATB3 = 1;
                OC1RS = 0;
                OC2RS = 0;
                go = 0;
                i = 0;
                lapTime = 0.0;
            }
            startTime = _CP0_GET_COUNT();

            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}

/*******************************************************************************
 End of File
 */