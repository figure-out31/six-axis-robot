/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "stdbool.h"
#include "stdarg.h"        /* va_list / va_start / va_end */
#include "stdio.h"         /* vsnprintf */
#include "string.h"        /* memcpy() */
#include "FreeRTOS.h"      /* taskENTER_CRITICAL / taskEXIT_CRITICAL */
#include "task.h"
#include "robot.h"

/* ============================================================
 *    USART3 DMA 接收缓冲区（由空闲中断 + DMA 共同管理）
 * ============================================================ */
volatile char uart3_rx_buff[UART3_RX_BUFF_SIZE + SAFE_BUFF_SIZE];
volatile uint32_t uart3_rx_pos = 0;

static uint8_t usart3_rx_data[UART3_RX_BUFF_SIZE + SAFE_BUFF_SIZE];
static uint16_t usart3_rx_len = 0;
static bool usart3_rx_done = false;

/**
 * @brief 获取 USART3 最新收到的数据帧
 * @param data  输出缓冲区（由调用者提供）
 * @param len   返回实际数据长度
 * @return true=有新数据, false=无新数据
 */
bool usart3_get_rx_data(uint8_t *data, uint16_t *len)
{
  if (usart3_rx_done == false) return false;

  taskENTER_CRITICAL();
  memcpy(data, usart3_rx_data, usart3_rx_len);
  *len = usart3_rx_len;
  usart3_rx_done = false;
  taskEXIT_CRITICAL();
  return true;
}

/**
 * @brief 线程安全的 printf（通过临界区保护，防止多任务输出交叉）
 *
 * 用法与 printf 相同。
 * 注意: 不能在 ISR 中调用，ISR 请使用 safe_printf_from_isr()。
 */
void safe_printf(const char *format, ...)
{
  static char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  taskENTER_CRITICAL();
  printf("%s", buf);
  taskEXIT_CRITICAL();
}

/**
 * @brief 中断安全的 printf（无锁，直接输出）
 *
 * 在 ISR 中使用，不走临界区保护。
 */
void safe_printf_from_isr(const char *format, ...)
{
  static char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  for (char *p = buf; *p != '\0'; p++)
  {
    while ((USART1->SR & USART_SR_TXE) == 0);
    USART1->DR = (uint8_t)(*p);
  }
}

/**
 * @brief printf 重定向到 USART1（Newlib _write 回调）
 */
int __io_putchar(int ch)
{
  while ((USART1->SR & USART_SR_TXE) == 0);
  USART1->DR = (uint8_t)ch;
  return ch;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /*
   * 启动 USART3 的 DMA 接收 + 空闲中断检测
   * 当接收到一帧数据后出现总线空闲（1个字节时间内无新数据），
   * 硬件自动触发空闲中断，在回调中标记接收完成。
   */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)uart3_rx_buff, UART3_RX_BUFF_SIZE + SAFE_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);   /* 关闭半传输中断，只在空闲或满时回调 */
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief USART3 接收事件回调（HAL 空闲中断 / DMA传输完成时触发）
 *
 * DMA 配置为循环模式（CIRCULAR），数据不断写入 uart3_rx_buff。
 * 空闲中断触发时，HAL 传入当前 DMA 接收到的总字节数 Size。
 *
 * DMA 循环写满缓冲区后会绕回开头继续写，所以读的时候必须用取模运算。
 * 为防止旧数据重复发送，用 last_size 记录上一次的 Size，
 * 只把增量部分（新收到的字节）发给 cmd_service。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3 && Size > 0)
  {
    static uint16_t last_size = 0;      //上一帧总字节数
    uint16_t  buf_size = UART3_RX_BUFF_SIZE + SAFE_BUFF_SIZE;

    /* 调试：打印 USART3 收到的原始字节，确认串口链路是否通 */
    safe_printf_from_isr("[U3RX] Size=%d : %.*s\n", Size,
                         (int)(Size < 48 ? Size : 48), (const char *)uart3_rx_buff);

    /* 拷贝 DMA 缓冲区的数据到解析缓冲区（避免 DMA 覆盖） */
    memcpy(usart3_rx_data, (const void *)uart3_rx_buff, Size);
    usart3_rx_len = Size;
    usart3_rx_done = true;   /* 通知上层有数据到达 */

    /* === 把收到的数据打包发给 cmd service === */
    if (g_robot.cmd_queue != NULL)
    {
      struct robot_cmd cmd = {.type = CMD_TYPE_UART3};
      uint16_t new_len ;
      uint16_t idx = 0;

      /* 检测 DMA 计数器重置（HAL 重新调用 ReceiveToIdle_DMA 后 Size 归零） */
      if (Size < last_size)
      {
        last_size = 0;
      }
      new_len = Size - last_size;

      while (idx < new_len && idx <ROBOT_CMD_LENGTH - 1)
      {
        uint16_t pos = (last_size + idx) % buf_size;
        cmd.cmd[idx] = uart3_rx_buff[pos];
        idx++;
      }
      cmd.cmd[idx] = '\0';

      if (idx > 0)
      {
        /* FromISR 版本：ISR 中往队列发数据 */
        BaseType_t xHigherPriorityTaskWoken =pdFALSE;
        xQueueSendToBackFromISR(g_robot.cmd_queue, &cmd, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
    last_size = Size;
  }
}

/* USER CODE END 1 */
