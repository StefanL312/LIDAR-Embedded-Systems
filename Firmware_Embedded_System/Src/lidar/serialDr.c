/**
 * @file    serialDr.c
 * @author  Daniël Martoredjo
 * @date    08 dec 2016
 * @brief   bla bla bla
 * @version 0.2
 * @todo    lol, alot after the update parameter case (line 160)
 */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lidarDefaultHeader.h"

#if SERIALDR == 1
/*extern static Taskhandle_t xUsbDrHandle;
extern static Taskhandle_t xSerialDrHandle;
extern static Taskhandle_t xComInterHandle;
extern static Taskhandle_t xSensDrivHandle;
extern static Taskhandle_t xPhotoIntHandle;
extern static Taskhandle_t xHBridgeHandle;
extern static Taskhandle_t xEasyStepHandle;
extern static Taskhandle_t xSysCtrlHandle;
extern static Taskhandle_t xPosCtrlHandle;*/

/* Macros used ONLY in serialDr.c*/
#define SERIAL_DRIVER_ACK_RETURN    0
#define SERIAL_MAX_DATA_LENGTH      10
#define SERIAL_MAX_FRAME_LENGTH     (SERIAL_MAX_DATA_LENGTH + 6)

#define TESTING_DUMMY_VARIABLES 0

/**
 * @brief      { This will control the uart. (send a string byte, by byte for example) }
 *
 * @param      pvParameters  The pv parameters
 */

/**
* Acknowledgements:
* Has a macro (SERIALDRIVER_ACK_RETURN) to determine if an ack needs to be send after everyframe. 
* If 0 no acks. If 1 all the acks
*
* Operation:
* check for new data from the usart (use queue for this)
* if there is new data, check if there is more than 1 frame available. 
* (3rd byte is length of the frame. If more frames available, ignore old frames)
* prevent uart from writing to the queue when extracting data.
* check if CRC is correct. if not send a resubmission request. if correct send an ack
* process the data frame.
* send appropriate data back to host.
* 
* live-feed:
* rising complications. 
*/

struct serialDataFrame
{
    unsigned char   ucDestinationID;
    unsigned char   ucSourceID;
    unsigned char   ucCommand;              
    unsigned char   ucFrameLengthBytes;             // note that the maximum possible lenght is 11 currently
    unsigned char   ucData[SERIAL_MAX_DATA_LENGTH]; // note that the maximum possible arriving data will be 5. 
                                                    // Can be derived from udFrameLengthBytes. udFrameLengthBytes - all_other_bytes
    unsigned char   ucStateFlags;
    unsigned char   ucCRC;
} serialInDataFrame, serialOutDataFrame;            // Should only be accessed from serialDr task

uint8_t bConnectionEstablished = 0;        // Is there connection opened?
unsigned char ucConnectedHostID = 0x00;     // ID of the connected Host
unsigned char ucPesonalSlaveID = 'S';       // ID of the microcontroller
unsigned char ucPersonalStateFlags = 0x00;  // randostateflags    

struct scanningParam
{
    uint8_t stepAngle;
    uint8_t scansPerAngle;
    uint8_t scanAveragingTimeWindow;
    unsigned char ucSensorReadInFormat;     // 'C': data is calculated to distance, 'R': data is raw sensor readin
} scanningParamRequest;

/* error booleans */
uint8_t ERRORserialInQueue = 0;
uint8_t ERRORserialOutQueue = 0;

/* Global states/request/variables */
uint8_t bScanRequestAvailable = 0;

struct scanDataStructure
{
    uint8_t scanID;
    uint8_t stepAngle;
    uint8_t numberOfSteps;
    uint8_t scansPerAngle;
    uint8_t scanAveragingTimeWindow;
    unsigned char ucSensorReadInFormat;
    uint8_t sensorShortData[42];
    uint8_t sensorLongData[42];
    uint16_t timeFullScan_ms;
    uint8_t timeBetweenSteps_ms;
} tempScan;

/*testing variables*/


/* User defined functoins */
int dFrameConstructor(void);
int dSerialInCRCCheck(void);
int dSerialSend(unsigned char ucCommand, unsigned char ucData[], int dLengthOfData);
int dTransmitSerialOutFrame(void);


void serialDr(void* pvParameters) 
{
	int i, j;
    tempScan.scanID = 0x01;
	tempScan.stepAngle = 5;
	tempScan.numberOfSteps = 42;
	tempScan.scansPerAngle = 1;
	tempScan.scanAveragingTimeWindow = 0;
	tempScan.ucSensorReadInFormat = 'R';
	for(i = 0; i < 42; i++)
	{
		tempScan.sensorShortData[i] = i + 100;
		tempScan.sensorLongData[i] = i + 200;
	}
	tempScan.timeFullScan_ms = 3200;
	tempScan.timeBetweenSteps_ms = 15;
	
    while(1) 
    {
        // serialInQueue
        unsigned char ucTempData[SERIAL_MAX_DATA_LENGTH];
        int temp = dFrameConstructor();
        if (temp == 0)
        {
            // No serial data available or time-out
        }
        else
        {
            // CRC
            temp = dSerialInCRCCheck();
            if ( temp == 0 )
            {
                // CRC incorrect, request frame from host
                // if (!dSerialSend('N', ucTempData, 0)) ERRORserialOutQueue = 1; 
                if (!dSerialSend('U', ucTempData, 0)) ERRORserialOutQueue = 1; 
            }
            else 
            {
                // Check if Destination ID is Personal Slave ID
                if (serialInDataFrame.ucDestinationID == ucPesonalSlaveID)
                {
                    
                    if ( !bConnectionEstablished )
                    {
                        // no connection yet
                        // check if command is connection command
                        if (serialInDataFrame.ucCommand == 'C' || serialInDataFrame.ucCommand == 'c')
                        {
                            // send ack if needed
                            if ( SERIAL_DRIVER_ACK_RETURN )
	                        {
	                            if (!dSerialSend('Y', ucTempData, 0)) ERRORserialOutQueue = 1;
	                        }

                            bConnectionEstablished = 1;
                            ucConnectedHostID = serialInDataFrame.ucSourceID;
                            
                            // send connection confirmed
                            if (!dSerialSend('C', ucTempData, 0)) ERRORserialOutQueue = 1;
                        }
                    }
                    else 
                    {    
                        if ( SERIAL_DRIVER_ACK_RETURN )
                        {
                            if (!dSerialSend('Y', ucTempData, 0)) ERRORserialOutQueue = 1;
                        }
                        switch(serialInDataFrame.ucCommand)
                        {
                            // Close connection
                            case 'd':
                            case 'D':
                                // send connection terminated confirmation
                                if (!dSerialSend('Y', ucTempData, 0)) ERRORserialOutQueue = 1;
                                bConnectionEstablished = 0;
                                ucConnectedHostID = 0x00;
                                break;
                            
                            // Stop last command
                            case 's':
                            case 'S':
                                // STOP!!
                                break;
                                
                            // Parameter command
                            case 'p':
                            case 'P':
                                i = 0;
                                
                                if ( serialInDataFrame.ucData[i++] == 'U')
                                {
                                    // update scanning parameter field
                                    scanningParamRequest.stepAngle               = serialInDataFrame.ucData[i++];
                                    scanningParamRequest.scansPerAngle           = serialInDataFrame.ucData[i++];
                                    scanningParamRequest.scanAveragingTimeWindow = serialInDataFrame.ucData[i++];
                                    scanningParamRequest.ucSensorReadInFormat    = serialInDataFrame.ucData[i++];
                                }
                                
                                i = 0;
                                ucTempData[i++] = serialInDataFrame.ucData[0];
                                ucTempData[i++] = scanningParamRequest.stepAngle;
                                ucTempData[i++] = scanningParamRequest.scansPerAngle;
                                ucTempData[i++] = scanningParamRequest.scanAveragingTimeWindow;
                                ucTempData[i++] = scanningParamRequest.ucSensorReadInFormat;
                                if (!dSerialSend('P', ucTempData, 5)) ERRORserialOutQueue = 1;
                                break;
                            
                            // go do scan
                            case 'a':
                            case 'A':
                                // request a scan from other
                                bScanRequestAvailable = 1;
                                while(bScanRequestAvailable);   // wait for scan to be resolved
                                if (!dSerialSend('Y', ucTempData, 0)) ERRORserialOutQueue = 1; // send ack when done
                                break;
                            
                            // send last scanned data
                            case 'r':
                            case 'R':
                                if (!dSerialSend('Y', ucTempData, 0)) ERRORserialOutQueue = 1; // send acknowledge command

                                if (TESTING_DUMMY_VARIABLES == 1) 
                                {
                                    // use dummy variables for testing purposes

                                    // calculate packets to send
                                    int remainingPakkets = tempScan.numberOfSteps + 1;
                                    // send parameters of the saved scanned data
                                    i = 0;
                                    ucTempData[i++] = tempScan.scanID;
                                    ucTempData[i++] = tempScan.stepAngle;
                                    ucTempData[i++] = tempScan.scansPerAngle;
                                    ucTempData[i++] = tempScan.scanAveragingTimeWindow;
                                    ucTempData[i++] = tempScan.ucSensorReadInFormat;
                                    ucTempData[i++] = remainingPakkets--;
                                    if (!dSerialSend('R', ucTempData, i)) ERRORserialOutQueue = 1; 
                                    
                                    for (remainingPakkets = remainingPakkets, j = 0; remainingPakkets > 0; remainingPakkets--, j+= tempScan.stepAngle)
                                    {
                                        i = 0;
                                        // angle of the sensor
                                        ucTempData[i++] = j;
                                        // ucTempData[i++] = j + 100;
                                        ucTempData[i++] = tempScan.sensorShortData[remainingPakkets-1];
                                        ucTempData[i++] = j + 30;
                                        // ucTempData[i++] = j + 200;
                                        ucTempData[i++] = tempScan.sensorLongData[remainingPakkets];
                                        ucTempData[i++] = remainingPakkets;
                                        if (!dSerialSend('R', ucTempData, i)) ERRORserialOutQueue = 1; 
                                    }
                                    
                                    // last packet
                                    i = 0;
                                    ucTempData[i++] = (uint8_t) tempScan.timeFullScan_ms>>16;
                                    ucTempData[i++] = (uint8_t) tempScan.timeFullScan_ms & 0xFF;
                                    ucTempData[i++] = tempScan.timeBetweenSteps_ms;
                                    ucTempData[i++] = 0;
                                    if (!dSerialSend('R', ucTempData, i)) ERRORserialOutQueue = 1; 
                                    
                                }
                                else
                                {
                                    // real data for transmission
                                    
                                    // Pull data from the FLASH memory 

                                }
                                
                                // end of chain
                                if (!dSerialSend('E', ucTempData, i)) ERRORserialOutQueue = 1; 
                                
                                break;
                                
                            // live streaming/scanning request
                            case 'f':
                            case 'F':
                                // set live-stream settings and start livestream
                                // Maybe new task??
                                break;
                                
                            // received ack on last send frame
                            case 'y':
                            case 'Y':
                                // oki
                                break;
                            
                            // received nack, resend last frame
                            case 'n':
                            case 'N':
                            case 'u':
                            case 'U':
                                // resend last frame
                                dTransmitSerialOutFrame();
                                break;
                                
                            default:
                                // unrecognized command
                                // FUUUUUUCKKK
                                // send 'U' command
                                if (!dSerialSend('U', ucTempData, 0)) ERRORserialOutQueue = 1; 
                        }
                    }
                }
            }    
        }

        vTaskDelay(100);
    }
}

int dFrameConstructor(void) 
{
    if (serialInQueue != 0)
    {
        int i, j, dTempFrameLength = SERIAL_MAX_FRAME_LENGTH, dTempDataLength = SERIAL_MAX_DATA_LENGTH;
        unsigned char ucTempFrame[SERIAL_MAX_FRAME_LENGTH], ucTempQueueBuffer;
        
        // extract one frame from queue
        for(i = 0; i < (SERIAL_MAX_FRAME_LENGTH - (SERIAL_MAX_FRAME_LENGTH - dTempFrameLength)); i++)
        {
             if( xQueueReceive(serialInQueue, &(ucTempQueueBuffer), (TickType_t) 10) == pdFALSE)
            {
                // exit function. Frame error.
                return 0;
            }
            else
            {
                ucTempFrame[i] = ucTempQueueBuffer;
            }
            
            if(i == 3)
            {
                dTempFrameLength = ucTempFrame[i]; // Frame length
            }
        }
        
        
        // Contruct frame
        i = 0;
        serialInDataFrame.ucDestinationID = ucTempFrame[i++];
		serialInDataFrame.ucSourceID = ucTempFrame[i++];
		serialInDataFrame.ucCommand = ucTempFrame[i++];
		serialInDataFrame.ucFrameLengthBytes = ucTempFrame[i++];
        dTempDataLength = (int) (serialInDataFrame.ucFrameLengthBytes - (SERIAL_MAX_FRAME_LENGTH - SERIAL_MAX_DATA_LENGTH));
		for (j = 0; j < (dTempDataLength); j++, i++)
		{
            serialInDataFrame.ucData[i] = ucTempFrame[i];
		}
        serialInDataFrame.ucStateFlags = ucTempFrame[i++];
        serialInDataFrame.ucCRC = ucTempFrame[i++];
    }
    return 1;
}

int dSerialInCRCCheck(void)
{
    int i;
    int dTempDataLength = (int) (serialInDataFrame.ucFrameLengthBytes - (SERIAL_MAX_FRAME_LENGTH - SERIAL_MAX_DATA_LENGTH));
    unsigned char ucTempCRC = serialInDataFrame.ucData[0];
    for(i = 1; i < ( dTempDataLength ); i++)
    {
        ucTempCRC ^= serialInDataFrame.ucData[i];
    }
    
    if ( ( serialInDataFrame.ucCRC == 
          ( serialInDataFrame.ucDestinationID ^
            serialInDataFrame.ucSourceID ^
            serialInDataFrame.ucCommand ^
            serialInDataFrame.ucFrameLengthBytes ^             
            ucTempCRC ^
            serialInDataFrame.ucStateFlags ) ) )
    {
        return 1;
    }
    return 0;
}

int dSerialSend(unsigned char ucCommand, unsigned char ucData[], int dLengthOfData)
{
    int i;
    if (ucConnectedHostID && !bConnectionEstablished)
    {
        // connected and valid ID.
        // construct serialOutDataFrame
        
        serialOutDataFrame.ucDestinationID      = ucConnectedHostID;
		serialOutDataFrame.ucSourceID           = ucPesonalSlaveID;
		serialOutDataFrame.ucCommand            = ucCommand;
		serialOutDataFrame.ucFrameLengthBytes   = SERIAL_MAX_FRAME_LENGTH - (SERIAL_MAX_DATA_LENGTH - dLengthOfData);
		for (i = 0; i < (dLengthOfData); i++)
		{
            serialOutDataFrame.ucData[i]        = ucData[i];
		}
        serialOutDataFrame.ucStateFlags         = ucPersonalStateFlags;
        
        // Calculate CRC
        unsigned char ucTempCRC = serialOutDataFrame.ucData[0];
        for(i = 1; i < ( dLengthOfData ); i++)
        {
            ucTempCRC ^= serialOutDataFrame.ucData[i];
        }
        serialOutDataFrame.ucCRC                = ( serialOutDataFrame.ucDestinationID ^
                                                    serialOutDataFrame.ucSourceID ^
                                                    serialOutDataFrame.ucCommand ^
                                                    serialOutDataFrame.ucFrameLengthBytes ^  
                                                    ucTempCRC ^                                                     
                                                    serialOutDataFrame.ucStateFlags );

        if ( !dTransmitSerialOutFrame() ) /*error*/ return 0;
        return 1;
    }
    return 0;
}

int dTransmitSerialOutFrame(void)
{
    int i;
    uint8_t dLengthOfData = (uint8_t) ( serialOutDataFrame.ucFrameLengthBytes - (SERIAL_MAX_FRAME_LENGTH - SERIAL_MAX_DATA_LENGTH) );
    unsigned char txBuffer;
    // Push to serialOutQueue
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucDestinationID,     ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucSourceID,          ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucCommand,           ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucFrameLengthBytes,  ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    for(i = 0; i < ( dLengthOfData ); i++)
    {
        if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucData[i],       ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    }
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucStateFlags,        ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    if( xQueueSend( serialOutQueue, ( void * ) &serialOutDataFrame.ucCRC,               ( TickType_t ) 10 ) != pdTRUE ) /*error*/   return errQUEUE_FULL;
    
    // Poll the RX Uart by sending the first byte manually
    if( xQueueReceive(serialOutQueue, &(txBuffer), (TickType_t) 10) != pdTRUE ) /*error*/   return 0;
    USART1->TDR = txBuffer;

    return 1;
}

#undef SERIAL_DRIVER_ACK_RETURN    
#undef SERIAL_MAX_DATA_LENGTH      
#undef SERIAL_MAX_FRAME_LENGTH   

#endif