/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include <arm_math.h>
/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"
#include "ip_addr.h"


#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#include "modbus_helper.h"
#include "MB_HoldingReg_MAP.h"
#include "MB_InputReg_MAP.h"
#include "version.h"
#include "device.h"
#include "dds.h"
#include "generator.h"
#include "measurer.h"
#include "intFlash.h"
#include "udpClient.h"

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

extern const char* firmwareDateTime;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define REG_INPUT_START         0
#define REG_INPUT_NREGS         1792 // 0x700
//#define REG_HOLDING_START       2000
#define REG_HOLDING_NREGS       512
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16_t   usRegInputStart = REG_INPUT_START;
u16_t   usRegInputBuf[REG_INPUT_NREGS];
//static u16_t   usRegHoldingStart = 0;
u16_t   usRegHoldingBuf[REG_HOLDING_NREGS];
/* Private function prototypes -----------------------------------------------*/
void fillMbHolingBuf(USHORT usAddress, USHORT usNRegs, USHORT* usRegHoldingBuf);
void setStringToHoldings(int startAddress, int count, const char* strSource, char* destination);
/* Private functions ---------------------------------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

static UCHAR    ucMBAddress;
static eMBMode  eMBCurrentMode;
static uint32_t zero_value = 0;

static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;
/* external variables */
extern QDeviceTypeDef qDevice;
extern ADC_HandleTypeDef hadc1;
/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

static __inline uint16_t getRegisterValueFromBuf(uint8_t* pucRegBuffer);

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;

        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;
            pvMBFrameStopCur = eMBRTUStop;
            peMBFrameSendCur = eMBRTUSend;
            peMBFrameReceiveCur = eMBRTUReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            eStatus = eMBRTUInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
            pvMBFrameStartCur = eMBASCIIStart;
            pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            if( !xMBPortEventInit(  ) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                eMBCurrentMode = eMode;
                eMBState = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBCurrentMode = MB_TCP;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
//        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
//        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll( void )
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static USHORT   usLength;
    static eMBException eException;

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED ){
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent ) == TRUE ){
        switch ( eEvent ){
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR ){
                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {
                    ( void )xMBPortEventPost( EV_EXECUTE );
                }
            }
            break;

        case EV_EXECUTE:
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ ){
                /* No more function handlers registered. Abort. */
                if( xFuncHandlers[i].ucFunctionCode == 0 ){
                    break;
                }
                else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode ){
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */
            if( ucRcvAddress != MB_ADDRESS_BROADCAST ){
                if( eException != MB_EX_NONE ){
                    /* An exception occured. Build an error frame. */
                    usLength = 0;
                    ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                    ucMBFrame[usLength++] = eException;
                }
                /*if( ( eMBCurrentMode == MB_ASCII ) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS )
                {
                    vMBPortTimersDelay( MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS );
                }*/
                eStatus = peMBFrameSendCur( ucMBAddress, ucMBFrame, usLength );
            }
            break;

        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ){
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    u16_t			varPointer, finishAddr = usAddress+usNRegs;
    u16_t* 			pointerU16;
    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) ){
        iRegIndex = ( int )( usAddress - usRegInputStart );


        for (varPointer = usAddress ; varPointer < finishAddr ; ++varPointer) {
        	pointerU16 = 0;
//        	if (varPointer % 2 == 0) {
//				if (varPointer >= INPUT_REG_BASE_UINT32_T) {
					switch (varPointer) {
					case INPUT_REG_BASE_UINT32_T:
					if(qDevice.qMeasurer.streamMeasurementStatus == DISABLE){
						SCB_InvalidateDCache_by_Addr((uint32_t *) &qDevice.adcEntitie.adcValues[0], SINGLE_MEASUREMENT_DMA_BUFFER_SIZE);
						HAL_ADC_Start_DMA(&hadc1, (uint32_t *) qDevice.adcEntitie.adcValues, 2);
						qDevice.adcEntitie.singleMeasurementFlag = SET;
						while(qDevice.adcEntitie.singleMeasurementFlag == SET);
						usRegInputBuf[iRegIndex++] = qDevice.adcEntitie.adcValues[0];
						break;
					case (INPUT_REG_BASE_UINT32_T+1): usRegInputBuf[iRegIndex++] = qDevice.adcEntitie.adcValues[1]; break;

					case (INPUT_REG_BASE_UINT32_T+2):
						pointerU16 = (USHORT*)&qDevice.adcEntitie.measurementLines[0].measurements[0];
						usRegInputBuf[iRegIndex++] = *(pointerU16++);
						usRegInputBuf[iRegIndex++] = *(pointerU16++);
					break;
					case (INPUT_REG_BASE_UINT32_T+4):
						pointerU16 = (USHORT*)&qDevice.adcEntitie.measurementLines[0].measurements[1];
						usRegInputBuf[iRegIndex++] = *(pointerU16++);
						usRegInputBuf[iRegIndex++] = *(pointerU16++);
					break;
					}


					default: pointerU16 = (u16_t*)zero_value;
					}
//				}

//        		if(pointerU16 != 0){
//        			usRegInputBuf[iRegIndex++] = *(pointerU16++);
//        			usRegInputBuf[iRegIndex++] = *pointerU16;
//        		}
//        	}
        }

        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 ){
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }

    }
    else{
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode ){
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex = 0;
    USHORT varPointer, finishAddr;
    float32_t		tmpF;
    uint16_t		value16, i;
    uint32_t mbOffset;
    USHORT* dataOffset;
    uint8_t *pointer = pucRegBuffer;

    if ( usAddress >= 0 ) {

        switch ( eMode ){
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
        	fillMbHolingBuf(usAddress, usNRegs, usRegHoldingBuf);
            while( usNRegs > 0 ){
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
        	finishAddr = usAddress + usNRegs;
			for (varPointer = usAddress ; varPointer < finishAddr ; ++varPointer) {
				switch (varPointer) {
//				case(hDacValue):
//					value16 = getRegisterValueFromBuf(pointer);
//					if(value16 >= 0 || value16 < 4096){
//						qDevice.dacValue = value16;
//						HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value16);
//					}
//					break;
//				case(hMainLoopTrigger):
//					value32 = arrayPionterToUint32(pointer);
//					qDevice.mainLoopTrigger = value32;
//					qDevice.main_cycle_counter = 0;
//					break;
				case (hMeasuringParamtersBase+hSinFrequencyAD9958Offset):
					tmpF = arrayMBToFloat(pointer);
//					if (tmpF > DDS_MAX_SIN_FREQUENCY)tmpF = DDS_MAX_SIN_FREQUENCY;
					if (tmpF < 0) tmpF = 0;
					qDevice.qMeasurer.measuringParameters.frequency = tmpF;
					if (qDevice.qMeasurer.measuringParameters.frequency	== 0) {
//						DDSEnableChannel(DDS_CSR_CHANNEL0 | DDS_CSR_CHANNEL1);
//						DDSSetFrequency(0);
//						DDSSetPhase(0);
//						DDSIOUpdate();
					} else {
//						initGenerator(&qDevice.qMeasurer, &qDevice.qGenerator);
//						generatorStart(&qDevice.qGenerator);
					}
					break;
				case (hMeasuringParamtersBase+hSinAmplitudeAD9958Offset):
					tmpF = arrayMBToFloat(pointer);
					qDevice.qMeasurer.measuringParameters.amplitude = tmpF;
//					setGeneratorAmplitude(&tmpF);
//					generatorStart(&qDevice.qGenerator);
					break;
				case (hMeasuringParamtersBase+hPointsCountOffset):
					value16 = getRegisterValueFromBuf(pointer);
					for (i = 1; i < 8; i++) {
						if (pow(2, i) == value16) {
							qDevice.qMeasurer.measuringParameters.pointCount = (uint32_t) value16;
							break;
						} else
							qDevice.qMeasurer.measuringParameters.pointCount = DEFAULT_POINT_COUNT;
					}
//					initGenerator(&qDevice.qMeasurer, &qDevice.qGenerator);
//					generatorStart(&qDevice.qGenerator);
					break;
				case (hMeasuringParamtersBase+hMeasuresCountOffset):
					qDevice.qMeasurer.measuringParameters.measurementCount =
							(uint32_t) getRegisterValueFromBuf(pointer);
//					initGenerator(&qDevice.qMeasurer, &qDevice.qGenerator);
//					generatorStart(&qDevice.qGenerator);
					break;

				case hPllClkRatioAD9958:
					qDevice.qGenerator.DDS_PLL_value = getRegisterValueFromBuf(pointer);
//					initGenerator(&qDevice.qMeasurer, &qDevice.qGenerator);
					break;

				case hDDSOneVoltCoef:
					qDevice.qGenerator.AD9958ToOneVoltCoef = getRegisterValueFromBuf(pointer);
					break;

//				case hTcpIpAddress:
//					qDevice.eth_params.IPaddr.addr = arrayPionterToUint32(pointer);
//					break;
//				case hTcpIpMask:
//					qDevice.eth_params.IPmask.addr = arrayPionterToUint32(pointer);
//					break;
//				case hTcpIpGate:
//					qDevice.eth_params.IPgate.addr = arrayPionterToUint32(pointer);
//					break;
//				case (hEthParamsBase + hUdpServerAddrOffset):
//						value32 = arrayPionterToUint32(pointer);
//						if(value32 != qDevice.eth_params.udpServerAddr.addr){
//							qDevice.eth_params.udpServerAddr.addr = value32;
//							udpClientDisconnect();
//							udpClientConnect(qDevice.eth_params.udpServerAddr, qDevice.eth_params.udpPort);
//					break;
//				case (hEthParamsBase + hServerUdpPortOffset):
//					value16 = getRegisterValueFromBuf(pointer);
//					if(value16 != qDevice.eth_params.udpPort){
//						qDevice.eth_params.udpPort = value16;
//						udpClientDisconnect();
//						udpClientConnect(qDevice.eth_params.udpServerAddr, qDevice.eth_params.udpPort);
////						printf("Set new UDP port %d\r\n", value16);
//					}
//					break;
//				case hModBusTcpPort:
//					qDevice.eth_params.modBusTcpPort = getRegisterValueFromBuf(pointer);
//					break;
				}

				if (varPointer >= hEthParamsBase && varPointer <= (hEthParamsBase + hEthParamsSize)) {
					value16 = *((USHORT*)(pucRegBuffer + 2*(varPointer-usAddress)));
					value16 = __REV16(value16);
					dataOffset = (USHORT*)(&qDevice.eth_params) + (varPointer - hEthParamsBase);
					*dataOffset = value16;
				}

				else if (usAddress >= MANAGMENT_REGISTER_BASE && usAddress <= MANAGMENT_REGISTER_BASE + MANAGMENT_REGISTERS_SIZE) {
					switch (varPointer) {
					case hreboot:
						qDevice.deviceResetFlag = 2;
						break;
					case hErase_page: break;
					case hSaveToFLASH:
						save_To_FLASH(&qDevice);
						qDevice.deviceResetFlag = 2;
						break;
					case hStreamMeasuring:
						value16 = getRegisterValueFromBuf(pointer);
						if (value16 == ENABLE) {
							startStreamMeasuering();
						}else if (value16 == DISABLE){
							stopStreamMeasuering();
						}
						break;
					case hResetMeasuringWathdogTimer:
						qDevice.qMeasurer.uninterruptableMeasurementWatchdog = RESET;
						break;
					case hTick:
						byTick();
					break;
					}
				}
				else if (varPointer >= hSerialsBase && varPointer <= (hSerialsBase + hSerialsSize)) {
					value16 = *((USHORT*)(pucRegBuffer + 2*(varPointer-usAddress)));
					value16 = __REV16(value16);
					dataOffset = (USHORT*)(&qDevice.serials) + varPointer - hSerialsBase;
					*dataOffset = value16;
				}
        }
    }
    }else{
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

void fillMbHolingBuf(USHORT usAddress, USHORT usNRegs, USHORT* usRegHoldingBuf){
	uint16_t addrIndex = 0, finishAddr = usAddress + usNRegs, zeroReg = 0;
	USHORT* pointer = 0;
	uint32_t mbOffset, value32;
	USHORT* dataOffset;
	float32_t fValue;

	for (addrIndex = usAddress; addrIndex < finishAddr; ++addrIndex) {

		dataOffset = &zeroReg;

//		if(addrIndex == hDacValue){
//			*(usRegHoldingBuf++) = qDevice.dacValue;
//		}
//		if(addrIndex == hMainLoopTrigger){
//			pointer = (USHORT*)&qDevice.mainLoopTrigger;
//			*(usRegHoldingBuf++) = *(pointer++);
//			*(usRegHoldingBuf++) = *(pointer++);
//		}

		if ((addrIndex >= hMeasuringParamtersBase) && (addrIndex <= (hMeasuringParamtersBase + hMeasuringParemtersSize))) {
			mbOffset = addrIndex - hMeasuringParamtersBase;
			dataOffset = (USHORT*)&qDevice.qMeasurer.measuringParameters + mbOffset;
			*(usRegHoldingBuf++) = *dataOffset;
		}

		if ((addrIndex >= hEthParamsBase) && (addrIndex <= (hEthParamsBase + hEthParamsSize))) {
			mbOffset = addrIndex - hEthParamsBase;
			dataOffset = (USHORT*)&qDevice.eth_params + mbOffset;
			*(usRegHoldingBuf++) = *dataOffset;
		}


		if ((addrIndex >= hGitCommitBase) && (addrIndex <= (hGitCommitBase + GIT_COMMIT_SIZE)) && (addrIndex%2==0)) {
			mbOffset = addrIndex - hGitCommitBase;
			switch(mbOffset){
			case hGitShortHashOffset:
				uint32_t gitHash = GIT_SHORT_HASH;
				pointer = (USHORT*)&gitHash;
				*(usRegHoldingBuf++) = *(pointer++);
				*(usRegHoldingBuf++) = *(pointer++);
				break;
			case hGitBranchOffset:
				setStringToHoldings(hGitBranchOffset, 20, GIT_BRANCH, (char*)usRegHoldingBuf);
				usRegHoldingBuf += 20;
				break;
			case hGitCommitMessageOffset:
				setStringToHoldings(hGitCommitMessageOffset, 20, GIT_COMMIT_MESSAGE, (char*)usRegHoldingBuf);
				usRegHoldingBuf += 20;
				break;
			case hGitStampOffset:
				setStringToHoldings(hGitStampOffset, 20, GIT_COMMIT_STAMP, (char*)usRegHoldingBuf);
				usRegHoldingBuf += 20;
				break;
			}
		}

		if ((addrIndex >= hAdcParamsBase) && (addrIndex <= (hAdcParamsBase + hAdcParamsSize))) {
			mbOffset = addrIndex - hAdcParamsBase;
			dataOffset = (USHORT*)&qDevice.adcEntitie.settings + mbOffset;
			*(usRegHoldingBuf++) = *dataOffset;
		}

		if ( (addrIndex >= hPermanentParamBase) && (addrIndex < (hPermanentParamBase + hPermanentParamsSize))) {
			mbOffset = addrIndex - hPermanentParamBase;
			switch(mbOffset){
				case hProtocolVersion: *(usRegHoldingBuf++) = UDP_PROTOCOL_VERSION; break;
				case hElectrodesCount: *(usRegHoldingBuf++) = ELECTRODES_COUNT; break;
				case hAdcBitRate: *(usRegHoldingBuf++) = ADC_BITRATE; break;
				case hPllClkRatioAD9958: *(usRegHoldingBuf++) = qDevice.qGenerator.DDS_PLL_value; break;
				case hADC_reference:
					fValue = ADC_REFERENCE;
					pointer = (USHORT*)&fValue;
					*(usRegHoldingBuf++) = *(pointer++);
					*(usRegHoldingBuf++) = *(pointer++);
					break;
				case hU_Gen_Max:
					fValue = 1.234;
					pointer = (USHORT*)&fValue;
					*(usRegHoldingBuf++) = *(pointer++);
					*(usRegHoldingBuf++) = *(pointer++);
					break;
				case hDeviceType: *(usRegHoldingBuf++) = MSI_TYPE; break;
				case hLpcCount:  *(usRegHoldingBuf++) = LPC_MCU_SIZE; break;
			}
		}

		if (addrIndex == hDDSOneVoltCoef) {
			*(usRegHoldingBuf++) = (uint16_t)qDevice.qGenerator.AD9958ToOneVoltCoef;
		}

		if ((addrIndex >= hLpcDeviceArrayBase) && (addrIndex <= (hLpcDeviceArrayBase + LPC_MCU_SIZE * LPC_DEVICE_SIZE))){
			value32 = (addrIndex - hLpcDeviceArrayBase)/LPC_DEVICE_SIZE;
			mbOffset = (addrIndex - hLpcDeviceArrayBase)%LPC_DEVICE_SIZE;
			if(mbOffset < hLpcDeviceGitBaseSift){
				dataOffset = (USHORT*)&qDevice.lpcMcus[value32].settings + mbOffset;
			}else if(mbOffset >= hLpcDeviceGitBaseSift && mbOffset < hLpcDeviceAdcValuesBaseSift){
				dataOffset = (USHORT*)&qDevice.lpcMcus[value32].gitCommit + mbOffset - hLpcDeviceGitBaseSift;
			}else if(mbOffset >= hLpcDeviceAdcValuesBaseSift && mbOffset < hLpcDeviceStatusBaseSift){
				dataOffset = (USHORT*)&qDevice.lpcMcus[value32].adcValues + mbOffset - hLpcDeviceAdcValuesBaseSift;
			}else if(mbOffset >= hLpcDeviceStatusBaseSift && mbOffset < (hLpcDeviceStatusBaseSift+4)){
				dataOffset = (USHORT*)&qDevice.lpcMcus[value32].status + mbOffset - hLpcDeviceStatusBaseSift;
			}else{
				dataOffset = &zeroReg;
			}
			*(usRegHoldingBuf++) = *dataOffset;
		}


		if ((addrIndex >= hSerialsBase) && (addrIndex <= (hSerialsBase + hSerialsSize))) {
			mbOffset = addrIndex - hSerialsBase;
			dataOffset = (USHORT*)&qDevice.serials + mbOffset;
			*(usRegHoldingBuf++) = *dataOffset;
		}

		if (addrIndex == hFirmwareDateTime) {
			setStringToHoldings(hFirmwareDateTime, 20, firmwareDateTime, (char*)usRegHoldingBuf);
		}
//		*(usRegHoldingBuf++) = *dataOffset;

	}
}

void setStringToHoldings(int startAddress, int count, const char* strSource, char* destination){
	int i = 0;
	int sourceSize = strlen(strSource);
	const char* pointer = strSource;
	for (i = 0; i < count; ++i) {
		if	(i < sourceSize){
			*(destination++) = *pointer;
		}else{
			*(destination++) = 0;
		}
		pointer++;
	}
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode ){
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete ){
    return MB_ENOREG;
}


static __inline uint16_t getRegisterValueFromBuf(uint8_t* pucRegBuffer){
	return (uint16_t)((*pucRegBuffer << 8) | (*(pucRegBuffer+1)));
}
