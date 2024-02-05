#ifndef	__MB_INPUT_REG_MAP_H__
#define	__MB_INPUT_REG_MAP_H__
//			Input Registers MAP
//					NAME			ADDRESS												TYPE			SIZE, 16bit register
#define		INPUT_REG_BASE_FLOAT	0x0000
#define		iZ_re					(INPUT_REG_BASE_FLOAT+0)			//				float			2
#define		iZ_im					(INPUT_REG_BASE_FLOAT+2)			//				float			2
#define		iPressure				(INPUT_REG_BASE_FLOAT+4)			//				float			2
#define		iTemperature			(INPUT_REG_BASE_FLOAT+6)			//				float			2
#define		irPt1000				(INPUT_REG_BASE_FLOAT+8)			//				float			2

#define		iU_re					(INPUT_REG_BASE_FLOAT+10)			//				float			2
#define		iU_im					(INPUT_REG_BASE_FLOAT+12)			//				float			2
#define		iI_re					(INPUT_REG_BASE_FLOAT+14)			//				float			2
#define		iI_im					(INPUT_REG_BASE_FLOAT+16)			//				float			2
//#define		iC						(INPUT_REG_BASE_FLOAT+18)		//				float			2
//#define		iY						(INPUT_REG_BASE_FLOAT+20)		//				float			2
//#define		iU_ampl					(INPUT_REG_BASE_FLOAT+22)		//				float			2
//#define		iI_ampl					(INPUT_REG_BASE_FLOAT+24)		//				float			2
#define		iMeasurementValidation	(INPUT_REG_BASE_FLOAT+26)			//				unt16_t			1
#define		iCpuTemperature			(INPUT_REG_BASE_FLOAT+28)			//				unt16_t			1
#define		iTemperatureAuxVoltage	(INPUT_REG_BASE_FLOAT+30)			//				float			2

#define 	iBaseAdcUIpair			0x0100								//

#define 	iBaseUIpair				0x0300

#define 	iZreimArray				0x0500

#define		INPUT_REG_BASE_UINT32_T	0x0600
#define 	iADC7793_Value			(INPUT_REG_BASE_UINT32_T+0)			//				uint32_t		2

#endif










