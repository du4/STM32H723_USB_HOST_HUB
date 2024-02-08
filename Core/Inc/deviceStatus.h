/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_STATUS_H
#define __DEVICE_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif



#define ROOT_HUB_CONNTCTION_ERROR			16
#define CHILD0_HUB_CONNTCTION_ERROR			15
#define CHILD1_HUB_CONNTCTION_ERROR			14
#define NOT_ALL_USB_LPC_FOUNDED_ERROR		13

#define SET_TOMOGRAPH_DEVICE_STATUS(status, BIT_INDEX)		(status |= (1<<BIT_INDEX))

#endif
