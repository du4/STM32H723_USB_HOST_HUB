/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_cdc.h"
#include "usbh_hub.h"

/* USER CODE BEGIN Includes */
#include "device.h"
#include "deviceStatus.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostHS;
ApplicationTypeDef usb_stack_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
extern CDC_StateTypedef CDC_STATE;
extern int cdcHandlesSize;
/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static uint32_t checkUsbDevices(USBH_HandleTypeDef *phost);
/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS) != USBH_OK){
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostHS, USBH_HUB_CLASS) != USBH_OK){
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostHS, USBH_CDC_CLASS) != USBH_OK){
    Error_Handler();
  }

  if (USBH_Start(&hUsbHostHS) != USBH_OK){
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void){
  /* USB Host Background task */
  USBH_Process(&hUsbHostHS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id){
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  usb_stack_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:

  uint32_t status = checkUsbDevices(phost);
  if(status == 0){
	  usb_stack_state = APPLICATION_READY;
	  CDC_STATE = CDC_BUSY;
  }
  break;

  case HOST_USER_CONNECTION:
  usb_stack_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

static uint32_t checkUsbDevices(USBH_HandleTypeDef *phost){
	uint32_t status = 0;
	/* LOG via printf HUBs and USB Devices */
	printf("USB devices list:\r\n\tRootHub: address=%d;\r\n", phost->rootHubHanlde->target.dev_address);
	if(phost->rootHubHanlde == NULL){
		printf("Error! No root HUB has founded!\r\n");
		SET_TOMOGRAPH_DEVICE_STATUS(status, ROOT_HUB_CONNTCTION_ERROR);
	}else{
		if(phost->rootHubHanlde->hubs[0] == NULL){
			printf("Error! No child0 HUB has founded!\r\n");
			SET_TOMOGRAPH_DEVICE_STATUS(status, CHILD0_HUB_CONNTCTION_ERROR);
		}
		if(phost->rootHubHanlde->hubs[1] == NULL){
			printf("Error! No child1 HUB has founded!\r\n");
			SET_TOMOGRAPH_DEVICE_STATUS(status, CHILD1_HUB_CONNTCTION_ERROR);
		}

		if(cdcHandlesSize != LPC_MCU_SIZE){
			printf("Error! %d LPC MCUs have founded, but it must be equals to %d\r\n", cdcHandlesSize, LPC_MCU_SIZE);
			SET_TOMOGRAPH_DEVICE_STATUS(status, NOT_ALL_USB_LPC_FOUNDED_ERROR);
		}


		if(status > 0)return status;

		printf("\tChild Hubs:\r\n");

		for (size_t hub = 0; hub < MAX_HUB_PORTS; ++hub) {
			if(phost->rootHubHanlde->Targets[hub].dev_address != 0){
				int hAddress = phost->rootHubHanlde->Targets[hub].dev_address;
				int hubPort = phost->rootHubHanlde->Targets[hub].tt_prtaddr;
				printf("\t\t address=%d; root hub port=%d\r\n", hAddress, hubPort);
				for (size_t chhub = 0; chhub < CHILD_HUBS; ++chhub) {
					if(phost->rootHubHanlde->hubs[chhub]->target.dev_address == hAddress){
						for (size_t port = 0; port < MAX_HUB_PORTS; ++port) {
							HUB_HandleTypeDef* ch = phost->rootHubHanlde->hubs[chhub];
							if(ch->Targets[port].dev_address != 0){
								printf("\t\t\tDevice address=%d; hub port=%d\r\n", ch->Targets[port].dev_address, ch->Targets[port].tt_prtaddr);
							}
						}
					}
				}
			}
		}
	}

}

/**
  * @}
  */

/**
  * @}
  */

