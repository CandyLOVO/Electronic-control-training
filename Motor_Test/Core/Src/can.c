/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
static CAN_TxHeaderTypeDef can_tx_message;
static CAN_TxHeaderTypeDef can_rx_message;
static uint8_t can_send_data[8];
static uint8_t can_receive_data[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
//		CAN_FilterTypeDef can_filter_st;
//    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterIdLow = 0x0000;
//    can_filter_st.FilterMaskIdHigh = 0x0000;
//    can_filter_st.FilterMaskIdLow = 0x0000;
//    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    can_filter_st.SlaveStartFilterBank = 14;
//    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1,&can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

//void can_cmd_send(int motor1,int motor2,int motor3,int motor4)
//{
//	uint32_t send_mail_box;
//	can_tx_message.StdId = 0x202;
//	can_tx_message.IDE = CAN_ID_STD;
//	can_tx_message.RTR = CAN_RTR_DATA;
//	can_tx_message.DLC = 0x08;
//	
//	can_send_data[0] = motor1 >> 8;
//	can_send_data[1] = motor1;
//	can_send_data[2] = motor2 >> 8;
//	can_send_data[3] = motor2;
//	can_send_data[4] = motor3 >> 8;
//	can_send_data[5] = motor3;
//	can_send_data[6] = motor4 >> 8;
//	can_send_data[7] = motor4;
//	
//	HAL_CAN_AddTxMessage(&hcan1,&can_tx_message,can_send_data,&send_mail_box);
//}

void can_cmd_receive()
{
	uint32_t receive_mail_box;
	can_rx_message.StdId = 0x202;
	can_rx_message.IDE = CAN_ID_STD;
	can_rx_message.RTR = CAN_RTR_DATA;
	can_rx_message.DLC = 0x08;
}

void Motor_Set_Current(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];
	tx_header.StdId = 0x203;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x02; //0x02对应一个电机 0x08对应四个电机
	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] = (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] = (v2)&0xff;
	tx_data[4] = (v3>>8)&0xff;
	tx_data[5] = (v3)&0xff;
	tx_data[6] = (v4>>8)&0xff;
	tx_data[7] = (v4)&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);
}
/* USER CODE END 1 */
