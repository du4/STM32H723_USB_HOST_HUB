#ifndef __MB_HOLDING_REG_MAP_H__
#define __MB_HOLDING_REG_MAP_H__
//								NAME												ADDRESS					SIZE, 16bit register////////////////////////	2 register size	////////////////////////////////////////////////
#define					hDEVICE_BASE												0x1600

//#define					TWO_REGISTER_BASE											0x1000
#define					ALIGMENT													32

#define				hMeasuringParamtersBase										hDEVICE_BASE
#define 				hSinFrequencyAD9958Offset								0											//		2
#define 				hSinAmplitudeAD9958Offset								2											//		2
#define 				hPointsCountOffset										4											//		1
#define 				hMeasuresCountOffset									6											//		1
#define				hMeasuringParemtersSize										8

#define				hEthParamsBase						(hDEVICE_BASE + 1*ALIGMENT)
#define 				hDeviceIPaddrOffset				0											//		2
#define 				hDeviceIPmaskOffset				2											//		2
#define 				hDeviceIPGateOffset				4											//		2
#define 				hUdpServerAddrOffset			6											//		2
#define 				hServerUdpPortOffset			8											//		1
#define 				hModBusTcpPortOffset			9											//		1
#define 				hDhcpState						10											//		1
#define 			hEthParamsSize						11

#define				hGitCommitBase													(hDEVICE_BASE + 2*ALIGMENT)
#define					hGitShortHashOffset											0							//			2
#define					hGitBranchOffset											2							//			20
#define					hGitCommitMessageOffset										22							//			20
#define					hGitStampOffset												42							//			20
#define				GIT_COMMIT_SIZE													62

#define				hAdcParamsBase						(hDEVICE_BASE + 4*ALIGMENT)
#define					hAdcPeriodOffset		0
#define					hAdcCoefOffset			1
#define					hAdcToMa4MaCoefOffset	3
#define					hAdcToMa20MaCoefOffset	4
#define				hAdcParamsSize				5

#define				hPermanentParamBase					(hDEVICE_BASE + 5*ALIGMENT)
#define					hProtocolVersion		0
#define					hElectrodesCount		1
#define					hAdcBitRate				2
#define					hPllClkRatioAD9958		3
#define					hADC_reference			4
#define					hU_Gen_Max				6
#define					hDeviceType				8
#define					hLpcCount				9
#define				hPermanentParamsSize				10

#define					hGeneratorOffset								(hDEVICE_BASE + 6*ALIGMENT)
#define 				hDDSOneVoltCoef									hGeneratorOffset

#define 			hMultiplexerProgramBase								(hDEVICE_BASE + 7*ALIGMENT)		//		16steps*16electrodes
#define				MUX_PROGRAM_SIZE									8//64

#define				hTomographConfigBase								(hDEVICE_BASE + 9*ALIGMENT)
	#define hTomographConfigStepCountOffset				0
	#define hTomographConfigCutsPerUsbPacketOffset		1
	#define hTomographCutRateOffset						2
	#define hTomographSampleFilterLengthOffset			4
	#define hTomographRrefOffset						6
	#define hTomographBitSettingsOffset					8
	#define hTomographRoleOffset						9
	#define hCutToMuxPeriodRate							10
#define				TOMOGRAPH_CONFIG_SIZE								22

///// LPC devicec //////
#define					hLpcDeviceArrayBase											0x2000

#define					hLpcDeviceSettingsBaseSift									0x0		// 88 bytes
	#define					LPC_DEVICE_SETTINGS_SIZE									44
#define					hLpcDeviceGitBaseSift										LPC_DEVICE_SETTINGS_SIZE	// 100 bytes
	#define					LPC_DEVICE_GIT_SIZE											50
#define					hLpcDeviceAdcValuesBaseSift									(hLpcDeviceGitBaseSift + LPC_DEVICE_GIT_SIZE)	// 8 bytes
#define					hLpcDeviceStatusBaseSift									(hLpcDeviceAdcValuesBaseSift + 8)	// 4 bytes

#define 			LPC_DEVICE_SIZE													128 // 106 registers




#define				hSerialsBase												0x3100
#define					hParentSerialNumberOffset									0			  	  //4
#define					hMsiPartSerialNumberOffset									4			  	  //4
#define					hMsiSerialNumberOffset										8			  	  //4
#define					hEthMacAddressOffset										12			  	  //4
#define				hSerialsSize												16


#define					hHardwareDeviceError										0x0


/// uint32_t
//#define 				hFirmwareVersion											(TWO_REGISTER_BASE+0)		//			2
//#define 				hAD7793_Zerro_Offset										(TWO_REGISTER_BASE+2)		//			2
//#define 				hAD7793_FullScale_Calibration								(TWO_REGISTER_BASE+4)		//			2
//#define 				hAD7793_ZeroPointValue										(TWO_REGISTER_BASE+6)		//			2
//#define 				hExtUartBaudRate											(TWO_REGISTER_BASE+8)		//			2
//#define 				hDDSGeneratorFrequancy										(TWO_REGISTER_BASE+10)		//			2
//#define 				hDeviceID													(TWO_REGISTER_BASE+12)		//			2
//#define 				hSensorID													(TWO_REGISTER_BASE+14)		//			2
//#define					hTimeStampHi												(TWO_REGISTER_BASE+20)		//			2
//#define					hTimeStampLo												(TWO_REGISTER_BASE+22)		//			2
//#define					hTimeCorrection												(TWO_REGISTER_BASE+24)		//			2
//#define 				hTcpIpAddress												(TWO_REGISTER_BASE+30)		//			2
//#define 				hTcpIpMask													(TWO_REGISTER_BASE+32)		//			2
//#define 				hTcpIpGate													(TWO_REGISTER_BASE+34)		//			2
//#define 				hUdpServerAddress											(TWO_REGISTER_BASE+36)		//			2
//#define 				hUninterruptableMeasuringWatchdog							(TWO_REGISTER_BASE+38)		//			2
//#define					TWO_REGISTERS_UINT_COUNT									40


// float32_t
//#define					TWO_REGISTER_BASE_FLOAT										0x1500
//#define 				hSinFrequencyAD9958											(TWO_REGISTER_BASE_FLOAT+0)	//			2
//#define 				hSinAmplitudeAD9958											(TWO_REGISTER_BASE_FLOAT+2)	//			2
//#define					hCh0FrequencyAD9958											(TWO_REGISTER_BASE_FLOAT+4)	//			2
//#define					hCh1FrequencyAD9958											(TWO_REGISTER_BASE_FLOAT+6)	//			2
//#define					hCh0PhaseAD9958												(TWO_REGISTER_BASE_FLOAT+8) //			2
//#define					hCh1PhaseAD9958												(TWO_REGISTER_BASE_FLOAT+10)//			2
//#define 				hU_Coef														(TWO_REGISTER_BASE_FLOAT+12)//			2
//#define 				hI_Coef														(TWO_REGISTER_BASE_FLOAT+14)//			2
//#define					hReferenceR													(TWO_REGISTER_BASE_FLOAT+18)//			2
//#define					hGeneratorResistance										(TWO_REGISTER_BASE_FLOAT+22)//			2
//#define					hSensorCurrentResistance									(TWO_REGISTER_BASE_FLOAT+24)//			2
//#define					TWO_REGISTERS_FLOAT_COUNT									26


//#define 				hChannelBase												(TWO_REGISTER_BASE_FLOAT + 0x100)
//#define					hChannelKuOffset											0
//#define					hChannelKiOffset											2
//#define					hChannelRrefOffset											4
//#define					hChannelDdsOneVoltCoeffOffset								6
//#define					hChannelRgenOffset											8
//#define					hChannelSensorCurrentResistanceOffset						10
//#define					hChannelUmultOffset											12
//#define					hChannelImultOffset											14
//#define					hChannelMeasuringParameterOffset							16
//#define					hChannelCastCoefOffset										24
//#define 				hChannelOffset												100


////////////////////////	1 register size	//////////////////////////////////////////////////
//#define					ONE_REGISTER_BASE											0x2000
//#define 				hDeviceAddress												(ONE_REGISTER_BASE+0)			//		1
//#define 				hADC7793GroupingCoef										(ONE_REGISTER_BASE+1)			//		1
//#define 				hPointsCount												(ONE_REGISTER_BASE+2)			//		1
//#define 				hMeasuresCount												(ONE_REGISTER_BASE+3)			//		1
//#define 				hMeasurerType												(ONE_REGISTER_BASE+4)			//		1
//#define 				hACR_AD9958ToOneVoltCoef									(ONE_REGISTER_BASE+5)			//		1
//#define					hContinuesMeasuresCount										(ONE_REGISTER_BASE+8)			//		1
//#define					hMultiplexrControlByte										(ONE_REGISTER_BASE+9)			//		1
//#define					hSPI13CLKDivider											(ONE_REGISTER_BASE+10)			//		1
//#define					hCannelsCount												(ONE_REGISTER_BASE+11)			//		1
//#define					hMuxG														(ONE_REGISTER_BASE+12)			//		1
//#define					hMuxU														(ONE_REGISTER_BASE+13)			//		1
//#define					hGMaxIndex													(ONE_REGISTER_BASE+14)			//		1
//#define					hMuxDelay													(ONE_REGISTER_BASE+15)			//		1
//#define					hUdpPort													(ONE_REGISTER_BASE+16)			//		1
//#define					hModBusTcpPort												(ONE_REGISTER_BASE+17)			//		1
//#define					hAD7793_Period												(ONE_REGISTER_BASE+18)			//		1
//#define					hAutoIp														(ONE_REGISTER_BASE+19)			//		1
//#define					hContinuousMeasurementStatus								(ONE_REGISTER_BASE+20)			//		1
//#define					hTemperatureSensorType										(ONE_REGISTER_BASE+21)
//#define 				hChannelIndex												(ONE_REGISTER_BASE+22)			//		1
//#define					hChannelCount												(ONE_REGISTER_BASE+24)
//#define					hSyncType													(ONE_REGISTER_BASE+25)
//#define					ONE_REGISTERS_SIZE											26






//	Management
#define					MANAGMENT_REGISTER_BASE										0x3000
#define					hreboot														(MANAGMENT_REGISTER_BASE+0)			  //1
#define					hErase_page													(MANAGMENT_REGISTER_BASE+1)			  //1
#define					hSaveToFLASH												(MANAGMENT_REGISTER_BASE+2)			  //1
#define					hCalibrateTemperatureZeroPoint								(MANAGMENT_REGISTER_BASE+3)			  //1
#define					hADCreadBegin												(MANAGMENT_REGISTER_BASE+4)			  //1
#define					hCalibrateAD7793											(MANAGMENT_REGISTER_BASE+5)			  //1
#define					hStreamMeasuring											(MANAGMENT_REGISTER_BASE+6)			  //1
#define					hContiniousReadingCutOff									(MANAGMENT_REGISTER_BASE+7)			  //1
#define					hSaveSerialsToFlash											(MANAGMENT_REGISTER_BASE+8)			  //1
#define 				hResetMeasuringWathdogTimer									(MANAGMENT_REGISTER_BASE+9)
#define 				hSaveSetupCoefsToFLash										(MANAGMENT_REGISTER_BASE+10)
#define 				hTick														(MANAGMENT_REGISTER_BASE+13)
#define					MANAGMENT_REGISTERS_SIZE									15


//	4 register base
#define					FOUR_REGISTER_BASE											0x3100

//	string
#define 				hFirmwareDateTime											0x3200

#endif
