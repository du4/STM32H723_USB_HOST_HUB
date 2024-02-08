/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUT_STATUS_H
#define __CUT_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#define NOT_THE_SAME_CUT_INDEX_ERROR		0

#define SET_TOMOGRAPH_CUT_STATUS(status, BIT_INDEX)		(status |= (1<<BIT_INDEX))

#endif
