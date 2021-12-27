/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_usbx_device.c
 * @author  MCD Application Team
 * @brief   USBX Device applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_class_rndis.h"
//#include "ux_device_descriptors.h"
#include "ux_device_stack.h"
#include "ux_system.h"
#include "ux_utility.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Thread priority */
#define DEFAULT_THREAD_PRIO 10

/* Thread preemption priority */
#define DEFAULT_PREEMPTION_THRESHOLD DEFAULT_THREAD_PRIO

/* USB App Stack Size */
#define USBX_APP_STACK_SIZE 1024

/* Usb Memory Size */
#define USBX_MEMORY_SIZE (38 * 1024)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_BYTE_POOL ux_byte_pool;
TX_THREAD ux_app_thread;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;  // not in example
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void usbx_app_thread_entry(ULONG arg);
/* USER CODE END PFP */
/**
  * @brief  Application USBX Device Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Device_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Device_MEM_POOL */
  (void)byte_pool;
  /* USER CODE END MX_USBX_Device_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Device_Init */
  /* Device framework full speed length */
  ULONG device_framework_fs_length;

  /* Device string framework length */
  ULONG string_framework_length;

  /* Device string framework length */
  ULONG languge_id_framework_length;

  /* Device framework full speed */
  UCHAR *device_framework_full_speed;

  /* Framework string */
  UCHAR *string_framework;

  /* Language id framework */
  UCHAR *language_id_framework;

  /* USBX  app memory pointer. */
  CHAR *ux_app_pointer;

  /* Allocate the USBX_MEMORY_SIZE. */
  ret = tx_byte_allocate(byte_pool, (VOID **)&ux_app_pointer, USBX_MEMORY_SIZE,
                         TX_NO_WAIT);

  /* Check USBX_MEMORY_SIZE allocation */
  if (ret != TX_SUCCESS) {
    printf("USBX_MEMORY_SIZE allocation failed: 0x%02x\n", ret);
    Error_Handler();
  }

  /* Initialize USBX Memory */
  ux_system_initialize(ux_app_pointer, USBX_MEMORY_SIZE, UX_NULL, 0);

  /* Get_Device_Framework_Full_Speed and get the length */
  device_framework_full_speed = USBD_Get_Device_Framework_Speed(
      USBD_FULL_SPEED, &device_framework_fs_length);

  /* Get_String_Framework and get the length */
  string_framework = USBD_Get_String_Framework(&string_framework_length);

  /* Get_Language_Id_Framework and get the length */
  language_id_framework =
      USBD_Get_Language_Id_Framework(&languge_id_framework_length);

  /* The code below is required for installing the device portion of USBX.
     In this application */
  ret = _ux_device_stack_initialize(
      NULL, 0, device_framework_full_speed, device_framework_fs_length,
      string_framework, string_framework_length, language_id_framework,
      languge_id_framework_length, UX_NULL);
  /* Check device stack init */
  if (ret != UX_SUCCESS) {
    printf("Device stack init failed: 0x%02x\n", ret);
    Error_Handler();
  }

  // create threads
  /* USER CODE END MX_USBX_Device_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
void usbx_app_thread_entry(ULONG arg) {
  /* Initialization of USB device */
  MX_USB_Device_Init();
}

void MX_USB_Device_Init() {
  /* USER CODE BEGIN USB_Device_Init_PreTreatment_0 */
  /* USER CODE END USB_Device_Init_PreTreatment_0 */

  /* Initialize the device controller HAL driver */
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN USB_Device_Init_PreTreatment_1 */
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x100);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x10);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x10);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x20);
  /* USER CODE END USB_Device_Init_PreTreatment_1 */

  /* Initialize and link controller HAL driver to USBx */
  _ux_dcd_stm32_initialize((ULONG)USB_OTG_FS, (ULONG)&hpcd_USB_OTG_FS);

  /* Start USB device by connecting the DP pullup */
  HAL_PCD_Start(&hpcd_USB_OTG_FS);

  /* USER CODE BEGIN USB_Device_Init_PostTreatment */
  /* USER CODE END USB_Device_Init_PostTreatment */
}
/* USER CODE END 1 */
